
#ifndef __TRAJECTORY_INTERPOLATOR_H
#define __TRAJECTORY_INTERPOLATOR_H


void executeTrajectory();

// Signal fired when PD controller completes an action
volatile char trajactoryFlag;

#endif
