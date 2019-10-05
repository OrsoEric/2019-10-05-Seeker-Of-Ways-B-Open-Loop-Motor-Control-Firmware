/****************************************************************************
**	ORANGEBOT PROJECT
*****************************************************************************
**	HotBlack Shield - DC Motor Testing
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			1
**	Version:			0.1 ALPHA
**	Compiler flags:
**		-Os
**		-std=C++11
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**		2019-10-03
**	Started from MazeRunner FPV demo
**	Upgrade to Uniparser V4 library
**	Pinmap of DC motors on HotBlack Shield V1
**		2019-10-04
**	Increased UART speed to 250Kb/s
**	Success test with 2019-10-04 SoW-B test serial
**	Communication timeout stops motors if no valid messages have been received
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**	Robot: SeekerOfWaysB
**	Hardware: HotBlackShield V1
**	Features:
**		Twin DC Motor PWM
**		Twin quadrature encoder reading (TODO)
**		Led Controls
****************************************************************************/

/****************************************************************************
**	USED PIN
**	TIPS: compile this field before writing any code: it really helps!
*****************************************************************************
**		LEDS
**	PB6			:	LED0#
**	PB7			:	LED1#
**
**		Raspberry Pi
**	PD0, RXI0	:	RPI_TXO
**	PD1, TXO0	:	RPI_RXI
**	RESET		:	RPI_P14 (true->reset microcontroller)
**
**		H-Bridges
**	PB2			:	Power Enable
**	PD4, OC1B	:	H-Bridge A PWM-
**	PD5, OC1A	:	H-Bridge A PWM+
**	PD6, OC2B	:	H-Bridge B PWM-
**	PD7, OC2A	:	H-Bridge B PWM+
**
**		Encoders
**	PC6			:	ENC-A A Channel
**	PC7			:	ENC-A B Channel
**	PA6			:	ENC-A Z Channel
**	PB0			:	ENC-B A Channel
**	PB1			:	ENC-B B Channel
**	PA7			:	ENC-B Z Channel
**
**		Servo Motors
**	PC0			:	SRV0 Servomotor
**	PC1			:	SRV1 Servomotor
**	PC2			:	SRV2 Servomotor
**	PC3			:	SRV3 Servomotor
**
**		Arduino Shield
**	PA0, ADC0	:	SHIELD-A0, ADC0
**	PA1, ADC1	:	SHIELD-A1, ADC1
**	PA2, ADC2	:	SHIELD-A2, ADC2
**	PA3, ADC3	:	SHIELD-A3, ADC3
**	PA4, ADC4	:	SHIELD-A4, ADC4
**	PA5, ADC5	:	SHIELD-A5, ADC5
**	PD2, RXI1	:	SHIELD-D0 SHIELD_TXO (Arduino Uno RX pin)
**	PD3, TXO1	:	SHIELD-D1 SHIELD_RXI (Arduino Uno TX pin)
**	PC0			:	SHIELD-D2
**	PD6, OC2B	:	SHIELD-D3, OC2B
**	PC1			:	SHIELD-D4
**	PB4, OC0B	:	SHIELD-D5, OC0B
**	PB3, OC0A	:	SHIELD-D6, OC0A
**	PC2			:	SHIELD-D7
**	PC3			:	SHIELD-D8
**	PD5, OC1A	:	SHIELD-D9, OC1A
**	PD4, OC1B	:	SHIELD-D10, OC1B
**	PD7, OC2A	:	SHIELD-D11, OC2A
**	PC4			:	SHIELD-D12
**	PC5			:	SHIELD-D13
**
****************************************************************************/

/****************************************************************************
**	USED PHERIPERALS
**	TIPS: compile this field before writing any code: it really helps!
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**	>
****************************************************************************/

/****************************************************************************
**	TODO
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	ENVROIMENT VARIABLES
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

#include "global.h"

#include "uniparser.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

using Orangebot::Uniparser;

/****************************************************************************
**	DEFINE
****************************************************************************/

//define for the mail loop
#define EVER (;;)

/****************************************************************************
**	MACRO
****************************************************************************/

/****************************************************************************
**	STRUCTURES
****************************************************************************/

/****************************************************************************
**	PROTOTYPE: FUNCTIONS
****************************************************************************/

//Placeholders set PWM commands
extern void set_pwm_a( bool dir, uint8_t pwm );
extern void set_pwm_b( bool dir, uint8_t pwm );

//Handler for the ping command. Keep alive connection
extern void ping_handler( void );
//Handler for the get board signature command. Send board signature via UART
extern void signature_handler( void );
//Handler for the motor speed set command. It's going to be called automatically when command is received
extern void set_speed_handler( int16_t right, int16_t left );

/****************************************************************************
**	PROTOTYPE: GLOBAL VARIABLES
****************************************************************************/

/****************************************************************************
**	GLOBAL VARIABLES:
****************************************************************************/

	///----------------------------------------------------------------------
	///	STATUS FLAGS
	///----------------------------------------------------------------------

//Volatile flags used by ISRs
volatile Isr_flags flags;

	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe uart_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 uart_tx_buf;
//allocate the working vector for the buffer
U8 v0[ UART_RX_BUF_SIZE ];
//allocate the working vector for the buffer
U8 v1[ UART_TX_BUF_SIZE ];

	///--------------------------------------------------------------------------
	///	PARSER
	///--------------------------------------------------------------------------

//Board Signature
U8 *board_sign = (U8 *)"Seeker-Of-Ways-B-00002";
//communication timeout counter
U8 uart_timeout_cnt = 0;
//Communication timeout has been detected
bool f_timeout_detected = false;

	///--------------------------------------------------------------------------
	///	MOTORS
	///--------------------------------------------------------------------------

//Desired setting for the DC motor channels
Dc_motor_pwm dc_motor[DC_MOTOR_NUM];
//Two DC Motor channels
Dc_motor_pwm dc_motor_target[DC_MOTOR_NUM];
//conversion between clockwise/counterclockwise to forward/backward
bool dc_motor_dir[DC_MOTOR_NUM];

/****************************************************************************
**	MAIN
****************************************************************************/

int main( void )
{
	///----------------------------------------------------------------------
	///	VARS
	///----------------------------------------------------------------------

	//Temp var
	uint8_t u8t;
	//prescaler
	uint8_t pre = 0;
	//Raspberry PI UART RX Parser
	Uniparser rpi_rx_parser = Uniparser();

	///----------------------------------------------------------------------
	///	INIT
	///----------------------------------------------------------------------

	//Init all pins, init all devices
	global_init();
	//Power to the H-Bridges
	H_BRIDGE_ON();

		///UART RX BUFFER INIT
	//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( uart_rx_buf, v0, UART_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( uart_tx_buf, v1, UART_TX_BUF_SIZE);

		///DC Motor init
	//Initialize first DC motor channel
	dc_motor[0].pwm = (uint8_t)0x00;
	dc_motor[0].f_dir = false;
	//Initialize other DC motors PWM
	for (u8t = 1; u8t < DC_MOTOR_NUM;u8t++)
	{
		dc_motor[u8t] = dc_motor[0];
	}
	//Initialize target DC motor channels
	for (u8t = 0; u8t < DC_MOTOR_NUM;u8t++)
	{
		dc_motor_target[u8t] = dc_motor[0];
	}
	//Save the conversion to achieve forward direction
	dc_motor_dir[DC_MOTOR_RIGHT] = DC_MOTOR_RIGHT_DIR;
	dc_motor_dir[DC_MOTOR_LEFT] = DC_MOTOR_LEFT_DIR;

	///----------------------------------------------------------------------
	///	REGISTER PARSER COMMANDS
	///----------------------------------------------------------------------

	//! Register commands and handler for the universal parser class. A masterpiece :')
	//Register ping command. It's used to reset the communication timeout
	rpi_rx_parser.add_cmd( "P", (void *)&ping_handler );
	//Register the Find command. Board answers with board signature
	rpi_rx_parser.add_cmd( "F", (void *)&signature_handler );
	//Set motor speed command. mm/s
	rpi_rx_parser.add_cmd( "PWMR%SL%S", (void *)&set_speed_handler );

	///----------------------------------------------------------------------
	///	MAIN LOOP
	///----------------------------------------------------------------------

	//Main Loop
	for EVER
	{

		///----------------------------------------------------------------------
		///	100.16 [Hz] Tick
		///----------------------------------------------------------------------

		//If: tick
		if (flags.led_update == 1)
		{
			///----------------------------------------------------------------------
			///	Led Blink
			///----------------------------------------------------------------------

			//Clear flag
			flags.led_update = 0;
			//Update communication timeout counter
			uart_timeout_cnt++;
			//
			if (uart_timeout_cnt >= RPI_COM_TIMEOUT)
			{
				//Clip timeout counter
				uart_timeout_cnt = RPI_COM_TIMEOUT;
				//If: it's the first time the communication timeout is detected
				if (f_timeout_detected == false)
				{
					//Stop the motors
					set_dc_motor_speed( (int16_t)0, (int16_t)0 );
				}
				//raise the timeout flag
				f_timeout_detected = true;	
			}
			else
			{
				//This is the only code allowed to reset the timeout flag
				f_timeout_detected = false;
			}

			///----------------------------------------------------------------------
			///	DC Motors Control
			///----------------------------------------------------------------------
			
			update_pwm();

			///----------------------------------------------------------------------
			///	Slow Code 1Hz
			///----------------------------------------------------------------------

			//advance prescaler (10 counts then reset)
			pre = (pre <= (100-1))?(pre +1):(0);
			//when prescaler is reset (1s)
			if (pre == 0)
			{
				//Toggle leds
				LED1_TOGGLE();
				//Send data
				AT_BUF_PUSH( uart_tx_buf, 'Z' );
			}
		}	//Endif: 100mS tick

		///----------------------------------------------------------------------
		/// UART RX
		///----------------------------------------------------------------------
		//	Handle RX from RS232
		//	Loopback

		//if: uart rx buffer is not empty
		if ( AT_BUF_NUMELEM( uart_rx_buf ) > 0)
		{
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			u8t = AT_BUF_PEEK( uart_rx_buf );
			AT_BUF_KICK_SAFER( uart_rx_buf );

				///Loopback
			//Push into tx buffer
			//AT_BUF_PUSH( uart_tx_buf, u8t );

				///Command parser
			//feed the input RX byte to the parser
			rpi_rx_parser.exe( u8t );
		}	//end if: uart rx buffer is not empty

		///----------------------------------------------------------------------
		/// UART TX
		///----------------------------------------------------------------------

		//if: the Uart0 HW buffer is empty and the UART tx buffer is not empty
		if ( (UART0_TX_READY()) && (AT_BUF_NUMELEM( uart_tx_buf ) > 0) )
		{
			//Get the byte to be filtered out
			u8t = AT_BUF_PEEK( uart_tx_buf );
			AT_BUF_KICK( uart_tx_buf );
			//Write on UART tx buffer.
			UDR0 = u8t;
		}	//End If: uart tx
	}	//end for: for EVER

	///----------------------------------------------------------------------
	///	MAIN LOOP
	///----------------------------------------------------------------------

	return 0;
}	//end main

/****************************************************************************
** FUNCTIONS:
****************************************************************************/

/****************************************************************************
** set_pwm_a
*****************************************************************************
**	Set the PWM of motor A
**	dir	: 0 = clockwise | 1 = counterclockwise
**	pwm	: 0 = stop | 0xff = full speed
****************************************************************************/

inline void set_pwm_a( bool dir, uint8_t pwm )
{
	OCR1AL = (dir == false)?((uint8_t)0x00):(pwm);
	OCR1BL = (dir == true)?((uint8_t)0x00):(pwm);

	return;
}

/****************************************************************************
** set_pwm_b
*****************************************************************************
**	Set the PWM of motor A
**	dir	: 0 = clockwise | 1 = counterclockwise
**	pwm	: 0 = stop | 0xff = full speed
****************************************************************************/

inline void set_pwm_b( bool dir, uint8_t pwm )
{
	OCR2A = (dir == false)?((uint8_t)0x00):(pwm);
	OCR2B = (dir == true)?((uint8_t)0x00):(pwm);

	return;
}

/***************************************************************************/
//!	@brief convert from a dc motor PWM structure to a speed number
//!	convert_pwm_to_s16 | Dc_motor_pwm
/***************************************************************************/
//! @param input | Dc_motor_pwm	input PWM structure
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return int16_t |			speed number
/***************************************************************************/

inline int16_t convert_pwm_to_s16( Dc_motor_pwm input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	int16_t ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Extract PWM setting
	ret = input.pwm;
	//if: direction is backward
	if (input.f_dir != f_dir)
	{
		//Correct sign
		ret = -ret;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return ret;
}

/***************************************************************************/
//!	@brief convert from a speed number to a dc motor PWM structure
//!	convert_s16_to_pwm | int16_t, bool
/***************************************************************************/
//! @param input | int16_t 		Input speed number
//! @param f_dir | bool			conversion between clockwise/counterclockwise to forward/backward
//! @return Dc_motor_pwm		PWM structure
/***************************************************************************/

inline Dc_motor_pwm convert_s16_to_pwm( int16_t input, bool f_dir )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	Dc_motor_pwm ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For each DC motor channel
	//If: right direction is backward
	if (input < 0)
	{
		//Assign the pwm to the right temp channel
		ret.pwm = -input;
		ret.f_dir = !f_dir;
	}
	//If: right direction is forward
	else
	{
		//Assign the pwm to the right temp channel
		ret.pwm = input;
		ret.f_dir = f_dir;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return ret;
}

/***************************************************************************/
//!	@brief update DC motor PWM
//!	update_pwm | void
/***************************************************************************/
//! @return void
//!	@details
//! Move PWM toward target PWM
//! Apply slew rate limiter
/***************************************************************************/

void update_pwm( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp counter
	uint8_t t;
	//true if speed has changed
	//bool f_change[DC_MOTOR_NUM];

	Dc_motor_pwm target_speed, actual_speed;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For each DC motor channel
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Fetch current settings
		target_speed = dc_motor_target[t];
		actual_speed = dc_motor[t];
	
		//If directions are different
		if (target_speed.f_dir != actual_speed.f_dir)
		{
			//if pwm is above slew rate
			if (actual_speed.pwm > DC_MOTOR_SLEW_RATE)
			{
				//Slow down by the slew rate
				actual_speed.pwm -= DC_MOTOR_SLEW_RATE;
			}
			//if pwm is below or at slew rate
			else
			{
				//Set neutral speed
				actual_speed.pwm = 0;
			}
			//if pwm is zero
			if (actual_speed.pwm == 0)
			{
				//I'm authorized to change direction
				actual_speed.f_dir = target_speed.f_dir;
			}
		}	//End If directions are different
		//if direction is the same
		else
		{
			//if pwm is above target
			if (actual_speed.pwm > target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, -DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm < target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if pwm is below target
			else if (actual_speed.pwm < target_speed.pwm)
			{
				//Decrease speed by PWM
				actual_speed.pwm = AT_SAT_SUM( actual_speed.pwm, +DC_MOTOR_SLEW_RATE, DC_MOTOR_MAX_PWM, 0 );
				//if: overshoot
				if (actual_speed.pwm > target_speed.pwm)
				{
					//I reached the target
					actual_speed.pwm = target_speed.pwm;
				}
			}
			//if: I'm already at the right speed
			else
			{
				//do nothing
			}
		}
	
	
		//Write back setting
		dc_motor[t] = actual_speed;	
	}
	
	set_pwm_a( dc_motor[DC_MOTOR_A].f_dir, dc_motor[DC_MOTOR_A].pwm );
	set_pwm_b( dc_motor[DC_MOTOR_B].f_dir, dc_motor[DC_MOTOR_B].pwm );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: signature_handler | void

/***************************************************************************/
//!	@brief ping command handler
//!	ping_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the ping command. Keep alive connection
/***************************************************************************/

void ping_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;

	LED0_TOGGLE();

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: ping_handler | void

/***************************************************************************/
//!	@brief board signature handler
//!	signature_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the get board signature command. Send board signature via UART
/***************************************************************************/

void signature_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;

	LED0_TOGGLE();
	//Init while
	t = 0;
	//while: no termination and tx buffer width is not exceeded
	while ((t < UART_TX_BUF_SIZE) && (board_sign[t]!= '\0'))
	{
		//Send the next signature byte
		AT_BUF_PUSH(uart_tx_buf, board_sign[t]);
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: signature_handler | void

/***************************************************************************/
//!	@brief set the target speed of the DC motors
//!	set_speed_handler | int16_t, int16_t
/***************************************************************************/
//! @return false: OK | true: fail
//!	@details
//! Handler for the motor speed set command. It's going to be called automatically when command is received
/***************************************************************************/

void set_speed_handler( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Reset communication timeout handler
	uart_timeout_cnt = 0;
	//Signal that a valid packet has been received
	LED0_TOGGLE();
	//Update the target seed of the motors
	set_dc_motor_speed( right, left );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: set_speed_handler | void


/***************************************************************************/
//!	@brief set the target speed of the DC motors
//!	set_dc_motor_speed | int16_t, int16_t
/***************************************************************************/
//! @return false: OK | true: fail
//!	@details
//! Left engine is connected to motor channel A
//! Right engine is connected to motor channel B
/***************************************************************************/

void set_dc_motor_speed( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp counter
	uint8_t t;
	//Temp PWM computation
	Dc_motor_pwm pwm_tmp[DC_MOTOR_NUM];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Compute Pwm
	pwm_tmp[DC_MOTOR_RIGHT] = convert_s16_to_pwm( right, DC_MOTOR_RIGHT_DIR );
	pwm_tmp[DC_MOTOR_LEFT] = convert_s16_to_pwm( left, DC_MOTOR_LEFT_DIR );

	AT_BUF_PUSH( uart_tx_buf, left>=0?'+':'-' );
	AT_BUF_PUSH( uart_tx_buf, pwm_tmp[DC_MOTOR_LEFT].f_dir==false?'F':'B' );

	//Scan DC motors
	for (t = 0;t < DC_MOTOR_NUM;t++)
	{
		//Set target speed
		dc_motor_target[t] = pwm_tmp[t];
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: set_speed_handler | void