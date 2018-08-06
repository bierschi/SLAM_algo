#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "alt_types.h"


/*!
 * @brief defines all possible mpu6050 registers
 * reset values are 0x00, except PWR_MGMT_1 = 0x40 and WHO_AM_I = 0x68
 * */
enum class MPU6050_Register : alt_u8 {
	SMPRT_DIV 			= 0x19,
	CONFIG				= 0x1A,
	GYRO_CONFIG			= 0x1B,
	ACCEL_CONFIG		= 0x1C,
	INT_PIN_CFG			= 0x37,
	INT_ENABLE			= 0x38,
	INT_STATUS			= 0x3A,
	ACCEL_XOUT_H		= 0x3B,
	ACCEL_XOUT_L		= 0x3C,
	ACCEL_YOUT_H		= 0x3D,
	ACCEL_YOUT_L		= 0x3E,
	ACCEL_ZOUT_H		= 0x3F,
	ACCEL_ZOUT_L		= 0x40,
	TEMP_OUT_H			= 0x41,
	TEMP_OUT_L			= 0x42,
	GYRO_XOUT_H			= 0x43,
	GYRO_XOUT_L			= 0x44,
	GYRO_YOUT_H			= 0x45,
	GYRO_YOUT_L			= 0x46,
	GYRO_ZOUT_H			= 0x47,
	GYRO_ZOUT_L			= 0x48,
	SIGNAL_PATH_RESET	= 0x68,
	USER_CTRL			= 0x6A,
	PWR_MGMT_1			= 0x6B,
	PWR_MGMT_2			= 0x6C,
	FIFO_COUNT_H		= 0x72,
	FIFO_COUNT_L		= 0x73,
	FIFO_R_W			= 0x74,
	WHO_AM_I			= 0x75,
};
/*!
 * @brief defines the two possible default i2c addresses (hardware setting)
 * */
enum class MPU6050_Addresses : alt_u8 {
	DEVICE_0 = 0xD0,
	DEVICE_1 = 0xD2,
};
/*!
 * @brief defines the possible accelerometer register settings
 * */
enum class AccelerometerSettings : alt_u8 {
	RANGE_2G  = 0x00,
	RANGE_4G  = 0x08,
	RANGE_8G  = 0x10,
	RANGE_16G = 0x18,
};
/*!
 * @brief defines the possible gyroscope register settings
 * */
enum class GyroscopeSettings : alt_u8 {
	RANGE_250_DEG  = 0x00,
	RANGE_500_DEG  = 0x08,
	RANGE_1000_DEG = 0x10,
	RANGE_2000_DEG = 0x18,
};

/*!
 * @brief represents the mpu6050 hardware device
 * */
class mpu6050
{
public:
	/*!
	 * @brief AccelerometerData
	 * */
	struct AccelerometerData {
		float acc_x;
		float acc_y;
		float acc_z;
	};
	/*!
	 * @brief GyroscopeData
	 * */
	struct GyroscopeData {
		float gyro_x;
		float gyro_y;
		float gyro_z;
	};
	/*!
	 * @brief typedef for the temperature value
	 * */
	using temp = float;

	/*!
	 * @brief constructs a mpu6050 object with the given address
	 * @param[in] deviceAddress: the used iic address for communication
	 * */
	mpu6050(const MPU6050_Addresses deviceAddress);
	/*!
	 * @brief initializes the mpu with the given settings
	 * @param[in] acc_sens: the sensitivity for the accelerometer (between 2G and 16G)
	 * @param[in] gyro_sens: the sensitivity for the gyroscope (between 250° and 2000°)
	 * @return currently return always 1; the idea was if the iic device acks the address set result to 0, but this mechanism is currently disabled
	 * */
	alt_u8 InitMPU6050(const AccelerometerSettings acc_sens, const GyroscopeSettings gyro_sens);
	/*!
	 * @brief reads the current acc data
	 * @param[out] acc_data provides the memory buffer for the data
	 * @return s.a.
	 * */
	alt_u8 ReadAccelerometer(AccelerometerData& acc_data);
	/*!
	 * @brief reads the current gyro data
	 * @param[out] gyro_data provides the memory buffer for the data
	 * @return s.a.
	 * */
	alt_u8 ReadGyroscope(GyroscopeData& gyro_data);
	/*!
	 * @brief reads the current temperature
	 * @param[out] temp_data provides the memory buffer for the data
	 * @return s.a.
	 * */
	alt_u8 ReadTemperature(temp& temp_data);
	/*!
	 * @brief reads the status register and returns the current measurement status (temp, gyro and acc), the register is automatically reseted by the read operation
	 * @return  1: if the current measurement is finished
	 * 			0: no measurement is finished
	 * */
	alt_u8 readStatus(void);

private:
	/*!
	 * @brief disables the default cstr
	 * */
	mpu6050() = delete;
	/*!
	 * @brief the iic address for the device
	 * */
	MPU6050_Addresses __deviceAddress;
	/*!
	 * @brief the factor for multiplying the results with
	 * */
	float gyro_sens_factor;
	/*!
	 * @brief the factor for multiplying the results with
	 * */
	float acc_sens_factor;
	/*!
	 * @brief the possible values for the different accelerometer settings
	 * */
	static constexpr float accelerometer_sensitivity_2g = 1.0 / 0x4000;
	static constexpr float accelerometer_sensitivity_4g = 1.0 / 0x2000;
	static constexpr float accelerometer_sensitivity_8g = 1.0 / 0x1000;
	static constexpr float accelerometer_sensitivity_16g = 1.0 / 0x0800;
	/*!
	 * @brief the possible values for the different gyroscope settings
	 * */
	static constexpr float gyroscope_sensitivity_250_degree = 1 / 131.0;
	static constexpr float gyroscope_sensitivity_500_degree = 1 / 65.5;
	static constexpr float gyroscope_sensitivity_1000_degree = 1 / 32.8;
	static constexpr float gyroscope_sensitivity_2000_degree = 1 / 16.4;
};



#endif /* MPU6050_HPP_ */
