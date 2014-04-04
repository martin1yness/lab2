#define ECHO2LCD

#include <pololu/orangutan.h>
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

// tick count for scheduler, yellow task, green task
volatile uint32_t G_timer3Ticks = 0;
volatile uint32_t G_timer0Ticks = 0;
volatile uint32_t G_timer1Ticks = 0;

int main()
{
	ConfigurePulseWithModulationClocks();	

	clear();	// clear the LCD
	init_menu();

	//enable interrupts
	sei();

	while (1) {
		serial_check();
		check_for_new_bytes_received();
	
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
	
	// PD6
	PORTD |= (1 << 6); // PORTB6
	PORTD &= ~(1 << 1); // PORTB6
	DDRD |= (1 << 6); // x1xx_xxxx
	DDRD &= ~(1 << 1); // xxxx_xx0x
	
	PORTA = 0x0;
	DDRA = 0x0;
	PORTB = 0x0;
	DDRB = 0x0;
	PORTC = 0x0;
	DDRC = 0x0;
}

volatile uint32_t trueCount = 0;
volatile uint32_t falseCount = 0;
ISR(TIMER2_COMPB_vect) {
	lcd_goto_xy(0,0);
	char* buf = malloc(sizeof(char)*32);
	itoa(PINC, buf, 10);
	print(buf);
	
	lcd_goto_xy(5,0);
	itoa(trueCount, buf, 10);
	print(buf);
	
	lcd_goto_xy(5,1);
	itoa(falseCount, buf, 10);
	print(buf);
	
	free(buf);
	
	if( PIND ^ ~(1 << 1) == 0x02 ) {		
		lcd_goto_xy(0,1);
		print("True");
		++trueCount;
	} else {
		lcd_goto_xy(0,1);
		print("False");
		++falseCount;
	}	
}

volatile uint32_t motorCount = 0;
volatile char redTog = 0x0;
ISR(PCINT2_vect) {
	++motorCount;
	red_led(redTog);
	redTog ^= 0x01;
	lcd_goto_xy(0,0);
	char* buf = malloc(sizeof(char)*32);
	itoa(motorCount, buf, 10);
	print(buf);
	free(buf);
}

//ISR(TIMER2_OVF_vect) {
//	red_led(1);
//}