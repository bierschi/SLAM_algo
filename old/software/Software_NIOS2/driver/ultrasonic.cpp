/*!
 * @file
 * */
#include "i2c_opencores.h"
#include "system.h"

#include "ultrasonic.hpp"

UltraSonicDevice::UltraSonicDevice(const UltraSonicAddress deviceAddress) : __deviceAddress(deviceAddress)
{

}


alt_u8 UltraSonicDevice::writeCMDRegister(const UltraSonicCommands val, const bool broadcast) const
{
	alt_u8 result = 1;

	if( broadcast == false )
		result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	else
		result =I2C_start(I2C_OPENCORES_0_BASE, 0x00, 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(val), 1);

	return result;
}

alt_u8 UltraSonicDevice::writeGAINRegister(const alt_u8 val) const
{
	alt_u8 result = 1;

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::MAX_GAIN), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(val), 1);

	return result;
}

alt_u8 UltraSonicDevice::writeRANGERegister(const alt_u8 val) const
{
	alt_u8 result = 1;

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::RANGE), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(val), 1);

	return result;
}

alt_u8 UltraSonicDevice::readRegister(const UltraSonicRegisterRead reg, alt_u16& readPtr) const
{
	alt_u8 result = 1;
	alt_u8 hlp;

	// start IIC communication
	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	// write which register must be read
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(reg), 1);

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	// read the actual register
	hlp = I2C_read(I2C_OPENCORES_0_BASE, 1);

	if( reg != UltraSonicRegisterRead::SW_REVISION && reg != UltraSonicRegisterRead::LIGHT_SENSOR)
	{
			result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
			I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(reg) + 1, 1); // read the low byte

			result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
			readPtr = I2C_read(I2C_OPENCORES_0_BASE, 1) + ( hlp << 8);
	}
	else
        {
            readPtr = hlp;
        }

	return result ;
}

alt_u8 UltraSonicDevice::readMeasurement(alt_u8* ultrasonic_measurement, const alt_u8 length) const
{
	alt_u8 result = 1;

	if( length == 0 )
		result = 0;
	else
	{
		for(alt_u8 i = 1; i <= (length > 34 ? 34 : length); ++i) // start from register 1 to length, but max is 34
		{
			// write which register must be read; start with light sensor register
			result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
			I2C_write(I2C_OPENCORES_0_BASE, i, 1);

			// read the actual value from the register
			result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
			*ultrasonic_measurement = I2C_read(I2C_OPENCORES_0_BASE, 1);
			if(*ultrasonic_measurement++ == 0x00) // no more objects were found, return
				break;
		}
	}
	return result;
}

alt_u8 UltraSonicDevice::changeAddress(const UltraSonicAddress newAddress)
{
	alt_u8 result = 1;

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND),0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicCommands::CHANGE_ADDRESS_COMMAND_1),1);

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND),0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicCommands::CHANGE_ADDRESS_COMMAND_2),1);

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND),0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicCommands::CHANGE_ADDRESS_COMMAND_3),1);

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND),0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(newAddress),1);


	if( result == 0){ //all write operations were successful, save new device address
		__deviceAddress = newAddress;
	}

	return result;
}

alt_u8 UltraSonicDevice::checkUltraSonicState(bool& check) const
{
	alt_u8 result = 1;

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(UltraSonicRegistersWrite::COMMAND), 1);

	result = I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	if( I2C_read(I2C_OPENCORES_0_BASE, 1) == 0xFF )
		check = true;
	else
		check = false;

	return result;
}
