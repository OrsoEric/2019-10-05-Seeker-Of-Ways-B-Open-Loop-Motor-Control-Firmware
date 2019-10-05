#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	//type definition using the bit width and signedness
	#include <stdint.h>
	//define the ISR routune, ISR vector, and the sei() cli() function
	#include <avr/interrupt.h>
	//name all the register and bit
	#include <avr/io.h>
	//hard delay
	#include <util/delay.h>
	//General purpose macros
	#include "at_utils.h"
	//AT Mega specific MACROS
	#include "at_mega_port.h"

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define UART_RX_BUF_SIZE	16
	#define UART_TX_BUF_SIZE	16

		///--------------------------------------------------------------------------
		///	COMMANDS
		///--------------------------------------------------------------------------

	//Communication timeout in system ticks
	#define RPI_COM_TIMEOUT		100

		///--------------------------------------------------------------------------
		///	MOTORS
		///--------------------------------------------------------------------------

	//Number of DC motor channels handled by the HotBlack Shield V1
	#define DC_MOTOR_NUM		2
	//DC Motor channel. Enumerate DC motor channels
	#define DC_MOTOR_A			0
	#define DC_MOTOR_B			1
	//Seeker Of Ways-B Motor Layout. On which channel the left and right motors are connected
	#define DC_MOTOR_RIGHT		DC_MOTOR_B
	#define DC_MOTOR_LEFT		DC_MOTOR_A
	//Seeker Of Ways-B Motor Direction
	//	false = turn clockwise to go forward
	//	true = turn counterclockwise to go forward
	#define DC_MOTOR_RIGHT_DIR	false	
	#define DC_MOTOR_LEFT_DIR	true
	//Maximum slew rate. PWM increment per tick
	#define DC_MOTOR_SLEW_RATE	1
	//Maximum PWM setting
	#define DC_MOTOR_MAX_PWM	255

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB6 )

	#define LED1_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB7 )

		///----------------------------------------------------------------------
		///	H-BRIDGE
		///----------------------------------------------------------------------

	#define H_BRIDGE_OFF()	\
		SET_BIT( PORTB, PB2 )

	#define H_BRIDGE_ON()	\
		SET_BIT( PORTB, PB2 )

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;

	//PWM and direction of a DC motor
	typedef struct _Dc_motor_pwm Dc_motor_pwm;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		U8 led_update	: 1;	//100Hz System Tick
		U8 				: 7;	//unused bits
	};

	//PWM and direction of a DC motor
	struct _Dc_motor_pwm
	{
		uint8_t pwm;			//DC Motor PWM setting. 0x00 = stop | 0xff = maximum
		uint8_t f_dir;			//DC Motor direction. false=clockwise | true=counterclockwise
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void global_init( void );
	//
	extern void timer0_init( void );
	//Two PWM channels for H-Bridge A
	extern void timer1_init( void );
	//Two PWM channels for H-Bridge B
	extern void timer2_init( void );
	//UART communication
	extern void usart0_init( void );
	//Init the ADC module
	extern void adc_init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/

		///----------------------------------------------------------------------
		///	DC MOTORS
		///----------------------------------------------------------------------

	//convert from a speed number to a DC motor PWM structure
	extern Dc_motor_pwm convert_s16_to_pwm( int16_t input, bool f_dir );
	//set the target speed of the DC motors
	extern void set_dc_motor_speed( int16_t right, int16_t left );
	//update DC motor PWM. apply slew rate limiter.
	extern void update_pwm( void );

	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags flags;
		
		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------
		//	Buffers structure and data vectors

	//Safe circular buffer for UART input data
	extern volatile At_buf8_safe uart_rx_buf;
	//Safe circular buffer for uart tx data
	extern At_buf8 uart_tx_buf;
	//allocate the working vector for the buffer
	extern U8 v0[ UART_RX_BUF_SIZE ];
	//allocate the working vector for the buffer
	extern U8 v1[ UART_TX_BUF_SIZE ];

		///--------------------------------------------------------------------------
		///	PARSER
		///--------------------------------------------------------------------------

	//Board Signature
	extern U8 *board_sign;
	//communication timeout counter
	extern U8 uart_timeout_cnt;
	//Communication timeout has been detected
	extern bool f_timeout_detected;

		///--------------------------------------------------------------------------
		///	MOTORS
		///--------------------------------------------------------------------------

	//Desired setting for the DC motor channels
	extern Dc_motor_pwm dc_motor_target[DC_MOTOR_NUM];
	//Two DC Motor channels current setting
	extern Dc_motor_pwm dc_motor[DC_MOTOR_NUM];
	//conversion between clockwise/counterclockwise to forward/backward
	extern bool dc_motor_dir[DC_MOTOR_NUM];

#else
	#warning "multiple inclusion of the header file global.h"
#endif
