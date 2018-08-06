/*!
 * @file
 * */
#ifndef TASKS_NIOS_HPP_
#define TASKS_NIOS_HPP_

#include "alt_types.h"
#include "alf_sharedmemory.hpp"

#include "FreeRTOS.h"
#include "task.h"

/*
 * define here the prototypes for all tasks
 * */

/*
 * @brief task for reading the ultrasonic sensors
 * */
void readUltraSonic( void* p);
/*
 * @brief task for reading the mpu values
 * */
void readMPU ( void* p );
/*
 * @brief task for reading the rotary encoder
 * */
void readRotary( void* p );
/*
 * @brief task for setting the motor speed and the steering angle
 * */
void setMotor_and_Steering( void* p);
/*
 * @brief task for setting the drive info into the shared memory of the NIOS and ARM
 * */
void setDriveInfo(void* p);

/*
 * @brief the interrupt routine for the mailbox communication
 * */
void Mailbox_isr(void* ptr, alt_u32 a);

/*
 * @brief global sharedMem variable
 * */
extern Alf_SharedMemoryComm sharedMem;
extern TaskHandle_t writeTask;

#endif /* TASKS_NIOS_HPP_ */
