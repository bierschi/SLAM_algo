/*
 * Timing.h
 *
 *  Created on: 13.09.2018
 *      Author: grauvogs
 */

#ifndef TIMING_H
#define TIMING_H

// CPU frequency in Hz
#define MAIN_CPU_CLOCK_FREQ	  64000000UL

// main sample time for all main functions in milliseconds
#define MAIN_SAMPLE_TIME_MS   50u
#define MAIN_SAMPLE_TIME_S	  (((float) MAIN_SAMPLE_TIME_MS) / 1000.0f)

#define SERVO_TIMER_REGISTER_MAX_PERIOD  10000u

#define MOTOR_TIMER_REGISTER_MAX_PERIOD	 1000u

#endif /* INC_TIMING_H_ */
