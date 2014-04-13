#include "trajectory_interpolator.h"
#include "main.h"
#include "pd.h"
#include<stdlib.h>

volatile char trajactoryFlag = 0x0;

/*
  Execute a predefined trajectory:
  1. 360 degrees (32 counts)
  2. Pause .5 seconds
  3. -360 degrees (-32 counts)
  4. Pause .5 seconds
  5. 10 degress (360 / 32 = 10 / x -> 10*32 = 360x -> x = 320 / 360 counts)
*/
volatile uint32_t startTime = 0;
volatile char step = 0x0;
void executeTrajectory() {
	if(startTime == 0) {
		startTime = G_time_ms;	
	}
	
	switch(step) {
		case 0x0: // init
			step = 0x1;
			G_desiredMotorPosition = G_currentMotorPosition + 32;
			trajactoryFlag = 0x0;
			print_usb("[*** Step 1 ***]", 16);
			break;
		case 0x1: // wait for rotation completion
			if(trajactoryFlag == 0x1) {
				trajactoryFlag = 0x0;
				step = 0x2;
				startTime = G_time_ms;
				print_usb("[*** Step 2 ***]", 16);
			}			
			break;
		case 0x2: // Wait some time
			if(G_time_ms - startTime >= 5000) {
				step = 0x3;
				print_usb("[*** Step 3 ***]", 16);		
			}
			break;
		case 0x3:
			// Rotate backwards 360
			step = 0x4;
			G_desiredMotorPosition = G_currentMotorPosition - 32;
			trajactoryFlag = 0x0;
			print_usb("[*** Step 4 ***]", 16);
			break;
		case 0x4:
			if(trajactoryFlag == 0x1) {
				print_usb("[*** Step 5 ***]", 16);
				trajactoryFlag = 0x0;
				step = 0x5;
				startTime = G_time_ms;
			}			
			break;
		case 0x5:
			// wait .5 s
			if(G_time_ms - startTime >= 5000) {
				print_usb("[*** Step 6 ***]", 16);
				step = 0x6;
			}
			break;
		case 0x6:
			// rotate 10 degress (can't do this soo we'll do ~11.25)
			step = 0x7;
			G_desiredMotorPosition = G_currentMotorPosition + 1;
			trajactoryFlag = 0x0;
			print_usb("[*** Step 7 ***]", 16);
			break;
		case 0x7:
			if(trajactoryFlag == 0x1) {
				trajactoryFlag = 0x0;
				print_usb("[*** Step 8 ***]", 16);
				step = 0x8;
			}						
			break;
		case 0x8:
			executeTrajectoryFlag = 0x0;
			print_usb("[*** FINISH ***]", 16);
			break;
	}
	
}