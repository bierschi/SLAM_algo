/*!
 * @file
 * */
#include "alt_types.h"
#include "io.h"
#include "system.h"

#include "Drive.hpp"
#include "PWM_Generator.h"
#include "Rotary_Encoder.h"

const alt_u8 Drive::mot_dir_pin = 0;
alt_u8 Drive::max_speed_percent = 30;
bool Drive::block_front_dir = false;
bool Drive::block_rear_dir = false;
alt_u8 Drive::current_speed = 0;
alt_u8 Drive::current_direction = 0;

void Drive::SetDriveSpeed(alt_u8 direction, alt_u8 speed) {

	if(direction == 1) { //backwards
		alt_u8 i = IORD_8DIRECT(GARFIELD_GPIO_BASE,0);
		i |= 1 << mot_dir_pin;
		IOWR(GARFIELD_GPIO_BASE, 0, i);
		current_direction = 1;
	}
	else { //default is forward
		alt_u8 i = IORD_8DIRECT(GARFIELD_GPIO_BASE,0);
		i &= ~(1 << mot_dir_pin);
		IOWR(GARFIELD_GPIO_BASE, 0, i);
		current_direction = 0;
	}
	current_speed = speed;
	PWMGen_Set_DutyCycle(DRIVE_PWM_BASE, (((alt_u32)max_speed_percent*(alt_u32)speed))/100);
}

void Drive::SetMaxSpeed(alt_u8 max_percent_speed) {
	if(max_percent_speed >= 100) {
		max_percent_speed = 100;
	}
	max_speed_percent = max_percent_speed;
}

void Drive::setBlock_Front(bool val)
{
	Drive::block_front_dir = val;
}
void Drive::SetBlock_Rear(bool val)
{
	Drive::block_rear_dir = val;
}
alt_u8 Drive::GetCurrent_direction(void)
{
	return current_direction;
}
alt_u8 Drive::GetCurrent_speed(void)
{
	return current_speed;
}
alt_u8 Drive::GetMax_Speed_Percent(void)
{
	return max_speed_percent;
}
bool Drive::GetBlock_Front(void)
{
	return block_front_dir;
}
bool Drive::GetBlock_Rear(void)
{
	return block_rear_dir;
}
