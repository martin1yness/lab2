/************************************************************************/
/* Proportional Derivative                                              */
/************************************************************************/

#ifndef pd_h
#define pd_h

#define MOTOR_FORWARD (PORTC |= (1 << 6))
#define MOTOR_BACKWARD (PORTC &= ~(1 << 6))
#define MOTOR_DIR_TOG (PORTC ^= (1 << 6))

// How often torque is adjusted (change in time in regards to distance traveled dt)
#define FREQUENCY 0.0033f

// desired counts/sec
volatile float G_desiredMotorSpeed;
// desired counts
volatile float G_desiredMotorPosition;
// Proportional Gain (Kp)
volatile float G_gainProportional;
// Derivative Gain (Kd)
volatile float G_gainDerivative;

void init_pd();

#endif