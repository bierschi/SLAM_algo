/*!
 * @file
 * */
#include "tasks_nios.hpp"

#include "alt_types.h"

#include "mpu6050.hpp"
#include "ultrasonic.hpp"
#include "Steering.hpp"
#include "Drive.hpp"
#include "alf_data_info.hpp"
#include "Rotary_Encoder.h"

#include "FreeRTOS.h"
#include "task.h"

//#define COLLECT_RUNTIME_INFO

#ifdef COLLECT_RUNTIME_INFO
char RunTimeStats_Buffer[2048];
volatile unsigned long TimerTicks = 0;
#endif

/* global section */
Alf_SharedMemoryComm sharedMem{};
TaskHandle_t writeTask; // currently not used

/* local section, configuration parameters for driving */
static const float pulses_to_meter = 1.0;
static const alt_u8 max_steering_angle = 40;
static const alt_u8 max_drive_speed = 35;
static const alt_u8 emergency_stop_distance = 12;
static const alt_u8 close_range_distance = 25;
static const alt_u8 close_range_speed = 20;

/*!
 *@brief this value is reduced every time the drive task is executed and is reset to the starting value in the mailbox interrupt service routine;
 *	this allows a timeout mechanism when the communication is failing to stop the vehicle. (meaning no new driving values incoming)
 * */
static volatile alt_u8 timeout = 20;

/*
 * Task communication over static variables
 * */
static mpu6050::AccelerometerData global_acc_data = {0, 0, 0};
static mpu6050::GyroscopeData global_gyro_data = {0, 0, 0};
static mpu6050::temp global_temp_data = 0;

static alt_u16 global_us_front_left_data = 0;
static alt_u16 global_us_front_right_data = 0;
static alt_u16 global_us_rear_left_data = 0;
static alt_u16 global_us_rear_right_data = 0;


void readMPU( void* p)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;

	/* mpu */
	mpu6050 mpu(MPU6050_Addresses::DEVICE_0);
	mpu.InitMPU6050(AccelerometerSettings::RANGE_2G, GyroscopeSettings::RANGE_250_DEG);


	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		// Wait for the next cycle ( every 50ms )
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		// if the IIC communication is failing due to the FreeRTOS scheduler (interrupted) make this a critical section by suspending other task
		// or by using the FreeRTOS critical section handling, currently not a problem
		mpu.ReadAccelerometer(global_acc_data);
		mpu.ReadGyroscope(global_gyro_data);
		mpu.ReadTemperature(global_temp_data);

		global_drive_info.acceleration = global_acc_data.acc_x;
		global_drive_info.lateral_acceleration = global_acc_data.acc_y;
		global_drive_info.z_acceleration = global_acc_data.acc_z;
		global_drive_info.Gyroscope_X = global_gyro_data.gyro_x;
		global_drive_info.Gyroscope_Y = global_gyro_data.gyro_y;
		global_drive_info.Gyroscope_Z = global_gyro_data.gyro_z;
		global_drive_info.temperature = global_temp_data;
	}
}

void readUltraSonic ( void* p )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 75;

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	 UltraSonicDevice us_front_left(UltraSonicAddress::DEVICE_00);
	 UltraSonicDevice us_front_right(UltraSonicAddress::DEVICE_01);
	 UltraSonicDevice us_rear_left(UltraSonicAddress::DEVICE_03);
	 UltraSonicDevice us_rear_right(UltraSonicAddress::DEVICE_02);

	 const alt_u8 maxSpeed = max_drive_speed;
	 bool close_range_front = false;
	 bool close_range_rear = false;

	 while(1)
	 {
		 us_front_left.writeCMDRegister(UltraSonicCommands::START_MEAS_CM, true);
		 // Wait for the next cycle ( every 65ms )
		 vTaskDelayUntil( &xLastWakeTime, xFrequency );

		 us_front_left.readRegister(UltraSonicRegisterRead::ECHO_0x01, global_us_front_left_data);
		 us_front_right.readRegister(UltraSonicRegisterRead::ECHO_0x01, global_us_front_right_data);
		 us_rear_left.readRegister(UltraSonicRegisterRead::ECHO_0x01, global_us_rear_left_data);
		 us_rear_right.readRegister(UltraSonicRegisterRead::ECHO_0x01, global_us_rear_right_data);

		 // check front
		 if(global_us_front_left_data < emergency_stop_distance || global_us_front_right_data < emergency_stop_distance)
		 { // emergency stop front
			 Drive::setBlock_Front(true);
			 close_range_front = true;
		 }
		 else if (global_us_front_left_data < close_range_distance || global_us_front_right_data < close_range_distance)
		 { // close range, careful
			 close_range_front = true;
			 Drive::setBlock_Front(false);
		 }
		 else
		 { // nothing in sight -> full throttle
			 Drive::SetMaxSpeed(maxSpeed);
			 Drive::setBlock_Front(false);
			 close_range_front = false;
		 }

		 // check rear
		 if(global_us_rear_left_data < emergency_stop_distance || global_us_rear_right_data < emergency_stop_distance)
		 { // emergency stop back
			 Drive::SetBlock_Rear(true);
			 close_range_rear = true;
		 }
		 else if (global_us_rear_left_data < close_range_distance || global_us_rear_right_data < close_range_distance)
		 { // close range, careful
			 close_range_rear = true;
			 Drive::SetBlock_Rear(false);
		 }
		 else
		 { // nothing in sight -> full throttle
			 Drive::SetMaxSpeed(maxSpeed);
			 Drive::SetBlock_Rear(false);
			 close_range_rear = false;
		 }

		 // evaluate close range results
		 if(close_range_front || close_range_rear)
			 Drive::SetMaxSpeed(close_range_speed);
		 else
			 Drive::SetMaxSpeed(maxSpeed);
	 }
}

void readRotary ( void* p )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	 while(1)
	 {
         TickType_t old_value = xLastWakeTime;
		 // Wait for the next cycle ( every 50ms )
		 vTaskDelayUntil( &xLastWakeTime, xFrequency );
         
         //Calculation of current speed
         alt_u32 curr_value = ROT_ENC_GetRotations(ROTARY_ENCODER_0_BASE);
         ROT_ENC_ClearCounter();
         alt_u8 speed = (curr_value * pulses_to_meter) / ((xLastWakeTime-old_value)*(1.0/configTICK_RATE_HZ));
         global_drive_info.speed = speed;
	 }
}

void setMotor_and_Steering ( void* p )
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;

	Steering::Init(max_steering_angle);
	Drive::SetMaxSpeed(max_drive_speed);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	Alf_Drive_Command drive{};

	alt_u8 real_speed = 0;
	alt_u8 real_direction = 0;

	while(1)
	{
		// Wait for the next cycle ( every 20ms )
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		sharedMem.Read(drive);

		real_speed = drive.speed;
		real_direction = drive.direction;

		// set light pins active low
		IOWR(GARFIELD_LIGHTING_BASE, 0, static_cast<alt_u8>(drive.light) == true ? 0 : 1);
		IOWR(GARFIELD_LIGHTING_BASE, 1, static_cast<alt_u8>(drive.light) == true ? 0 : 1);
		IOWR(GARFIELD_LIGHTING_BASE, 2, static_cast<alt_u8>(drive.light) == true ? 0 : 1);
		IOWR(GARFIELD_LIGHTING_BASE, 3, static_cast<alt_u8>(drive.light) == true ? 0 : 1);


		Steering::Set(drive.angle);

		if(Drive::GetBlock_Front() == true && real_direction == 0)
		{ //drive forward -> stop motor
			real_speed = 0;
			real_direction = 0;
		}
		if(Drive::GetBlock_Rear() == true && real_direction == 1)
		{ //drive backward -> stop motor
			real_speed = 0;
			real_direction = 1;
		}
		// timeout happened, meaning no interrupt occurred and therefore no new data was received
		if(timeout == 0)
		{
			Drive::SetDriveSpeed(real_direction, 0);
		}
		else
		{
			--timeout;
			Drive::SetDriveSpeed(real_direction, real_speed);
		}
	}
}

void setDriveInfo(void* p)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		// Wait for the next cycle ( every 200ms )
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		// write the current drive info into the shared memory
		sharedMem.Write(global_drive_info);

#ifdef COLLECT_RUNTIME_INFO
		if( xTaskGetTickCount() > 60000)
		{
			vTaskSuspendAll();
			vTaskGetRunTimeStats(RunTimeStats_Buffer);
			while(1);
		}
#endif
	}
}

void Mailbox_isr(void* ptr, alt_u32 a)
{
	alt_u32 h = alt_irq_disable_all();
	sharedMem.ReadInterruptHandler();
	IORD(MAILBOX_ARM2NIOS_0_BASE, 0);
	timeout = 20;
	alt_irq_enable_all(h);
}
