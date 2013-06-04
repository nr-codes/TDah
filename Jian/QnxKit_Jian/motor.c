#include "motor.h"

// This function calculates the current necessary to drive the motor
// given the desired angular velocity and acceleration
double calcI(double vel, double accel){
	
	if(accel<0){
		return (INERTIA*accel + DYN_FRIC_NEG*vel + STAT_FRIC_NEG*sign(vel))/KM_NEG; // current
	}
	else{
		return (INERTIA*accel + DYN_FRIC_POS*vel + STAT_FRIC_POS*sign(vel))/KM_POS; // current
	}
}

// This function returns the sign of the input argument
double sign(double vel){
	return vel > 0 ? 1 : (vel < 0 ? -1 : 0);
}

