#define ECHO2LCD

#include <pololu/orangutan.h>
#include <pololu/OrangutanDigital.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// System tasks
#include "menu.h"

/*
 * http://www.pololu.com/docs/0J20
 * http://www.pololu.com
 * http://forum.pololu.com
 */

volatile uint32_t motorCount = 0;
volatile char motorPhaseBuffer[5];
volatile int motorPhaseBufferIdx;

volatile int global_counts_m1;
volatile char global_error_m1;

volatile char global_last_m1a_val;
volatile char global_last_m1b_val;

int main()
{
	ConfigurePulseWithModulationClocks();	

	clear();	// clear the LCD
	init_menu();
	
	// Add terminator
	//motorPhaseBuffer = malloc(sizeof(char)*5);
	motorPhaseBuffer[4] = '\0';
	
	//enable interrupts
	sei();

	int len;
	char buf[32];
	while (1) {
		serial_check();
		check_for_new_bytes_received();
		
		if(motorPhaseBufferIdx == 0) {
			lcd_goto_xy(0,0);
			print(motorPhaseBuffer);
		}
		
		len = sprintf(buf, "%d %d %d %d", global_counts_m1, global_error_m1, global_last_m1a_val, global_last_m1b_val);
		lcd_goto_xy(0,1);
		buf[len] = '\0';
		print(buf);
	} //end while loop
}

/*
 * PWM Pulse-width Modulation - Used to control voltage over time a duty time
 */
inline void ConfigurePulseWithModulationClocks() {	
	// 
	// Set Timer 2 (8-bit) PWD
	//
	// 00 (OC2A disconnected) 10 (Clear OC2B on match, Set OC2B bottom, non-inverting) xx (reserved) 11 (Fast PWM 0xFF) 
	TCCR2A = 0x23; // 0010_0011
	// FOC2A : FOC2B : - : - : WGM22 : CS22 : CS21 : CS20
	// x : x : x : x : 0 : 1 : 0 : 0   -- 1024 prescale 
	TCCR2B |= 0x7;
	OCR2B = 10;
	TIMSK2 |= 0x4; // xxxx_x101
	
	// Output to motor
	DDRD |= (1 << 6); // x1xx_xxxx
	PORTA &= ~(1 << 6);
	
	// Counter pin changes
	//  NOTE: Seem to work whether they are set input or output
	DDRA &= ~0x01 & ~(1<<1);
	PORTA |= 0x01 | 0x01;
	
	// Set pin change interrupt for A0 and A1
	PCICR = 0x01;
	PCMSK0 = 0x03;
}


ISR(TIMER2_COMPB_vect) {
	
}

volatile char tog = 1;
ISR(PCINT0_vect) {
	red_led(tog);
	tog ^= 0x01;
		
	char t = PINA & 0x03;
	char * tmpChar = malloc(sizeof(char));
	itoa((PINA ^ ~0x03) & PINA, tmpChar, 10);
	motorPhaseBuffer[motorPhaseBufferIdx] = *tmpChar;
	motorPhaseBufferIdx = (motorPhaseBufferIdx + 1) % 4;	
	
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
	
	free(tmpChar);
	
	//m1a_val = isInputHigh(PINA | (1 << PINA0));
	//m1b_val = isInputHigh(PINA | (1 << PINA1));
	
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