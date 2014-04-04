#ifndef __LEDs_H
#define __LEDs_H

#include <avr/io.h>         //gives us names for registers
#include <avr/interrupt.h>

#include <inttypes.h> //gives us uintX_t

#include <string.h>

#define SET_TIMER1_PRESCALER(x)     (TCCR1B |= ##x)
#define SET_TIMER1_COMPAREMATCH(x)  (OCR1A = ##x)

#endif