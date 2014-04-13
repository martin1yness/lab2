
#ifndef __MAIN_H
#define __MAIN_H

#include <avr/io.h>         //gives us names for registers
#include <avr/interrupt.h>

#include <inttypes.h> //gives us uintX_t

#include <string.h>

#define SET_TIMER1_PRESCALER(x)     (TCCR1B |= ##x)
#define SET_TIMER1_COMPAREMATCH(x)  (OCR1A = ##x)
#define MOTOR_SPEED_SAMPLES 3

// unit: counts/sec
volatile float G_currentMotorSpeed;
// counts since startup
volatile float G_currentMotorPosition;
// Milliseconds since start
volatile uint32_t G_time_ms;
// Position in multiples of 4 phases
volatile int global_counts_m1;

volatile char loggingOn;

// If true, trajectory will be executed
volatile char executeTrajectoryFlag;

#endif