#define RUNBYSPEED

#include<float.h>
#include "pd.h"
#include "main.h"
#include "menu.h"

/************************************************************************/
/* Initializes the PD controller ~306.4 Hz                              */
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

 Frequency ~306 Hz  ~3.3ms                                             
/************************************************************************/
ISR(TIMER0_COMPA_vect) {
	#ifdef RUNBYSPEED
	float error = (G_desiredMotorSpeed - G_currentMotorSpeed);
	#else
	float error = (G_desiredMotorPosition - G_currentMotorPosition);
	#endif
	
	return;
	
	if(error < 0) {
		error = -error;
		MOTOR_BACKWARD;
	} else {
		MOTOR_FORWARD;
	}
	float T = G_gainProportional * error + G_gainDerivative * ( error / FREQUENCY);
	if(T > 254) {
		OCR2B = 254;
	} else {
		OCR2B = T;
	}
}