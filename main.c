#define RUNBYSPEED

#include <pololu/orangutan.h>
#include <pololu/OrangutanDigital.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// System tasks
#include "menu.h"
#include "pd.h"
#include "trajectory_interpolator.h"

/*
 * http://www.pololu.com/docs/0J20
 * http://www.pololu.com
 * http://forum.pololu.com
 */

volatile uint32_t G_time_ms = 0;

volatile float G_currentMotorSpeed = 0.0;
volatile float G_currentMotorPosition = 0.0;
volatile float G_desiredMotorSpeed = 0.0;
volatile float G_desiredMotorPosition = 0.0;
volatile char loggingOn = 0x0;
volatile uint32_t lastLog = 0;



/************************************************************************/
/* Max Frequency: ~306Hz (~3.3ms)                                       */
/************************************************************************/
inline void init_motor() {	
	//
	// Set Timer 2 (8-bit) PWD
	//
	// 00 (OC2A disconnected) 10 (Clear OC2B on match, Set OC2B bottom, non-inverting) xx (reserved) 11 (Fast PWM 0xFF)
	TCCR2A = 0x23; // 0010_0011
	// FOC2A : FOC2B : - : - : WGM22 : CS22 : CS21 : CS20
	// x : x : x : x : 0 : 1 : 0 : 1   -- prescale
	TCCR2B |= 0x7;
	OCR2B = 0; // start at 0 duty cycle	
	//TIMSK2 |= 0x4; // xxxx_x101
	
	// Output to motor
	DDRD |= (1 << 6); // x1xx_xxxx
	PORTD &= ~(1 << 6);	
	
	// Direction PC6
	MOTOR_FORWARD;	
	
	// Counter pin changes
	//  NOTE: Seem to work whether they are set input or output
	DDRA &= ~0x01 & ~(1<<1);
	PORTA |= 0x01 | 0x01;
	
	// Set pin change interrupt for A0 and A1
	PCICR = 0x01;
	PCMSK0 = 0x03;
}

inline void init_clock() {
	//
	// Set 16-bit Timer/Counter 1 for 1ms resolution
	//
	//   Clock 20Mil * 1/prescaler * 1/CompareMatchRegister = 1000 Hz (1ms)
	//   20M / 8 / 2500 = 1000

	// Timer/Counter Control Registers 3 A - xxxxxx00 bit
	TCCR3A &= ~(1 << WGM31);
	TCCR3A &= ~(1 << WGM30);
	// Timer/Counter Control Register 3 B - xxx01010
	TCCR3B &= ~(1 << WGM33); // xxx0xxxx
	TCCR3B |= (1 << WGM32); // xxxx1xxx
	// 8 prescaler
	TCCR3B &= ~(1 << CS32);  // xxxxx0xx
	TCCR3B |= (1 << CS31);  // xxxxxx1x
	TCCR3B &= ~(1 << CS30);  // xxxxxxx0
	// Timer Interrupt Mask Register - xxxxxx1x
	TIMSK3 |= (1 << OCIE3A); // xxxxxx1x
	// Output Compare Registers 3 A
	OCR3A = 2500;
}

/************************************************************************/
/* Occurs every phase change in motor signal. 4 phases per count        */
/************************************************************************/
volatile int global_counts_m1;
volatile char global_error_m1;
volatile char global_last_m1a_val;
volatile char global_last_m1b_val;
ISR(PCINT0_vect) {
	char t = PINA & 0x03;
	
	unsigned char m1a_val, m1b_val;
	switch(t) {
		case 0x00:
		m1a_val = 0;
		m1b_val = 0;
		break;
		case 0x01:
		m1a_val = 1;
		m1b_val = 0;
		break;
		case 0x02:
		m1a_val = 0;
		m1b_val = 1;
		break;
		case 0x03:
		m1a_val = 1;
		m1b_val = 1;
		break;
	}
	
	char plus_m1 = m1a_val ^ global_last_m1b_val;
	char minus_m1 = m1b_val ^ global_last_m1a_val;
	
	if(plus_m1)
		global_counts_m1 += 1;
	if(minus_m1)
	 	global_counts_m1 -= 1;

	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
		global_error_m1 = 1;
	
	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;	
}

/************************************************************************/
/* kEEP TIME                                                            */
/************************************************************************/
volatile float global_motorSpeeds[3];
volatile int global_motorSpeedsIdx = 0;
ISR(TIMER3_COMPA_vect) {
	++G_time_ms;
	if(G_time_ms % 100 == 0) {
		// 100 ms
		float lastPosition = G_currentMotorPosition;
		G_currentMotorPosition = global_counts_m1 / 4.0f;
		global_motorSpeedsIdx = (global_motorSpeedsIdx + 1) % 3;
		float diff = lastPosition - G_currentMotorPosition;
		global_motorSpeeds[global_motorSpeedsIdx] = (diff < 0 ? -diff : diff) * 10;
		G_currentMotorSpeed = (global_motorSpeeds[0] + global_motorSpeeds[1] + global_motorSpeeds[2]) / 3.0f;
	}
}

int main()
{
	init_motor();
	init_clock();
	init_pd();

	clear();	// clear the LCD
	init_menu();
	
	//enable interrupts
	sei();

	int len;
	char buf[64];
	while (1) {
		serial_check();
		check_for_new_bytes_received();
		
		if(loggingOn && G_time_ms % 1000 == 0 && lastLog != G_time_ms) {
			lastLog = G_time_ms;
			#ifdef RUNBYSPEED
			len = sprintf(buf, "Speed --> Pr:%.2f Pm:%.2f T:%d counts:%d pos:%.2f\r\n>", G_currentMotorSpeed, G_desiredMotorSpeed, OCR2B, global_counts_m1, G_currentMotorPosition);
			#else
			len = sprintf(buf, "Distance --> Pr:%.2f Pm:%.2f T:%d counts:%d\r\n>", G_currentMotorPosition, G_desiredMotorPosition, OCR2B, global_counts_m1);
			#endif
			print_usb(buf, len);
		}
		
	} //end while loop
}