#define RUNBYSPEED

#include<float.h>
#include "pd.h"
#include "main.h"
#include "menu.h"

/************************************************************************/
/*
/* Initializes the PD controller ~306.4 Hz -> every ~3.3                */
/*   Max Engine Speed ~55 counts/sec -> every 20ms
/************************************************************************/
inline void init_pd() {
	//
	// Set 8-Bit Time/Counter 2 for 1ms resolution
	//
	//   20mil * 1/prescaler * 1/CompareMatchRegister = 1000 Hz (1ms)
	//   20mil / 256 / 255 = ~306.4 Hz (~3.3ms)
	//
	// setting of WGM01 to get CTC mode
	TCCR0A = 0x08; // 0000_0100
	//  00 (n/a) 00 (read-only) 0 (Fast PWM) 010 (8 prescaler)
	TCCR0B = 0x02; // 0000_0101
	//  00000 (reserved) 0 (compare b interrupt) 1 (compare a interrupt) 0 (overflow interrupt)
	TIMSK0 = 0x02; // 0000_0010
	// Output Compare Registers 0 A
	OCR0A = 255;
}

/************************************************************************
T = Kp(Pr - Pm) - Kd*Vm where

T = Output motor signal (torque)
Pr = Desired motor position or speed
Pm = Current motor position or speed
Vr = Desired motor velocity
Vm = Current motor velocity (computed based on finite differences)
Kp = Proportional gain
Kd = Derivative gain  

T = Kp(Vr - Vm) + Kd( d (Vr-Vm) / dt )                                    

Let Kp = 10
T - 10e / ( (e - e') / .05 ) = Kd

 Frequency ~306 Hz  ~3.3ms                                             
/************************************************************************/
volatile uint16_t everySix = 0; // every sitxth execution is just under 20ms
volatile float G_lastError = 0;

volatile float G_gainProportional = 5.6f;
volatile float G_gainDerivative = -0.72f;
volatile float G_torque = 0;
ISR(TIMER0_COMPA_vect) {
	if(++everySix % 6 != 0) {
		return;		
	}
		
	#ifdef RUNBYSPEED
	float error = (G_desiredMotorSpeed - G_currentMotorSpeed);
	MOTOR_FORWARD;	
	#else
	float error = (G_desiredMotorPosition - G_currentMotorPosition);
	if(error < 0) {
		MOTOR_BACKWARD;
	} else {
		MOTOR_FORWARD;
	}
	#endif
	
	// 55 counts/sec = 255 torque
	// .83 counts/sec = 10 torque
	// 21 counts/sec = 100 torque
	//
	// Systems of Equations:
	// 20 = Kp*1 + Kd*-1/.05 subtrac 200/30 = Kp*30/30 + KD*-21/.05/30   (14)
	// 4.3333 = -Kd6 -> Kd ~ -.72222
	// 20 = Kp + 14.444 ~ 5.6  or 
	G_torque = G_gainProportional * error + G_gainDerivative * ( (G_lastError - error) / 0.05f);			
	G_lastError = error;
	
	//return;
	
	int sum = OCR2B + G_torque;
	if(sum > 255) {
		OCR2B = 255;
	} else if(sum < 0) {
		OCR2B = 0;
	} else {
		OCR2B = sum;
	}
}