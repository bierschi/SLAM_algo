/*!
 * @file
 * */
#include "i2c_opencores.h"
#include "system.h"

#include "mpu6050.hpp"

mpu6050::mpu6050(MPU6050_Addresses deviceAddress) : __deviceAddress(deviceAddress), gyro_sens_factor(0.0), acc_sens_factor(0.0)
{

}


alt_u8 mpu6050::InitMPU6050(AccelerometerSettings acc_sens, GyroscopeSettings gyro_sens)
{
	alt_u8 result = 1;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	// wakes up the mpu6050 from sleep mode by writing to pwr_mgmt register
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::PWR_MGMT_1), 0);
	I2C_write(I2C_OPENCORES_0_BASE, 0x00, 1);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_CONFIG), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(acc_sens), 1);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_CONFIG), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(gyro_sens), 1);

	switch (gyro_sens)
	{
	case GyroscopeSettings::RANGE_250_DEG:
		gyro_sens_factor = gyroscope_sensitivity_250_degree;
		break;
	case GyroscopeSettings::RANGE_500_DEG:
		gyro_sens_factor = gyroscope_sensitivity_500_degree;
		break;
	case GyroscopeSettings::RANGE_1000_DEG:
		gyro_sens_factor = gyroscope_sensitivity_1000_degree;
		break;
	case GyroscopeSettings::RANGE_2000_DEG:
		gyro_sens_factor = gyroscope_sensitivity_2000_degree;
		break;
	}

	switch ( acc_sens )
	{
	case AccelerometerSettings::RANGE_2G:
		acc_sens_factor = accelerometer_sensitivity_2g;
		break;
	case AccelerometerSettings::RANGE_4G:
		acc_sens_factor = accelerometer_sensitivity_4g;
		break;
	case AccelerometerSettings::RANGE_8G:
		acc_sens_factor = accelerometer_sensitivity_8g;
		break;
	case AccelerometerSettings::RANGE_16G:
		acc_sens_factor = accelerometer_sensitivity_16g;
		break;
	}

	return result;
}

alt_u8 mpu6050::ReadAccelerometer(AccelerometerData& acc_data)
{
	alt_u8 result = 1;
	alt_16 acc_data_tmp = 0;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_XOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_XOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	acc_data.acc_x = acc_data_tmp * acc_sens_factor;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_YOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_YOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	acc_data.acc_y = acc_data_tmp * acc_sens_factor;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_ZOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::ACCEL_ZOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	acc_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	acc_data.acc_z = acc_data_tmp * acc_sens_factor;

	return result;
}

alt_u8 mpu6050::ReadGyroscope(GyroscopeData& gyro_data)
{
	alt_u8 result = 1;
	alt_16 gyro_data_tmp = 0;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_XOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_XOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	gyro_data.gyro_x = gyro_data_tmp * gyro_sens_factor;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_YOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_YOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	gyro_data.gyro_y = gyro_data_tmp * gyro_sens_factor;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_ZOUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::GYRO_ZOUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	gyro_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	gyro_data.gyro_z = gyro_data_tmp * gyro_sens_factor;

	return result;
}

alt_u8 mpu6050::ReadTemperature(temp& temp_data)
{
	alt_u8 result = 1;
	alt_16 temp_data_tmp = 0;

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::TEMP_OUT_H), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	temp_data_tmp = (I2C_read(I2C_OPENCORES_0_BASE, 1) << 8);

	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::TEMP_OUT_L), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	temp_data_tmp |= I2C_read(I2C_OPENCORES_0_BASE, 1);

	temp_data = (temp_data_tmp / 340) + 36.53;

	return result;
}

alt_u8 mpu6050::readStatus(void)
{
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 0);
	I2C_write(I2C_OPENCORES_0_BASE, static_cast<alt_u8>(MPU6050_Register::INT_STATUS), 1);
	I2C_start(I2C_OPENCORES_0_BASE, static_cast<alt_u32>(__deviceAddress), 1);
	return (I2C_read(I2C_OPENCORES_0_BASE, 1) & 1);
}

