/*!
 * @file
 * */
#include "alt_types.h"
#include "io.h"
#include "system.h"
#include "PWM_Generator.h"

#include "Steering.hpp"

alt_u8 Steering::max_angle_delta = 40;
float Steering::val_per_deg = 0;

void Steering::Init(alt_u8 max_angle) {
	if(max_angle > MAX_STEERING_ANGLE) {
		max_angle_delta = MAX_STEERING_ANGLE;
	}
	else {
		max_angle_delta = max_angle;
	}
	val_per_deg = 19.0/60.0*((float)max_angle_delta/(float)max_steering_value);
}

void Steering::Set(alt_8 angle) {
	if(angle > max_steering_value) {
		angle = max_steering_value;
	}
	else if(angle < max_steering_value * -1) {
		angle = max_steering_value * -1;
	}
	PWMGen_Set_DutyCycle(STEERING_PWM_BASE, NEUTRAL_POS_VALUE - (val_per_deg*angle));
}
