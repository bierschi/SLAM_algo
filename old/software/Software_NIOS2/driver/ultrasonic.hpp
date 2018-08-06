/*!
 * @file
 * @brief this file contains the definition of all ultrasonic specific modules (currently only UltraSonicDevice)
 */
#ifndef ULTRASONIC_HPP_
#define ULTRASONIC_HPP_

#include "alt_types.h"

/*!
 * @brief defines all possible IIC addresses for the SRF08 ultra sonic range finder
 * */
enum class UltraSonicAddress : alt_u8  {
	DEVICE_00 = 0xE0,
	DEVICE_01 = 0xE2,
	DEVICE_02 = 0xE4,
	DEVICE_03 = 0xE6,
	DEVICE_04 = 0xE8,
	DEVICE_05 = 0xEA,
	DEVICE_06 = 0xEC,
	DEVICE_07 = 0xEE,
	DEVICE_08 = 0xF0,
	DEVICE_09 = 0xF2,
	DEVICE_10 = 0xF4,
	DEVICE_11 = 0xF6,
	DEVICE_12 = 0xF8,
	DEVICE_13 = 0xFA,
	DEVICE_14 = 0xFC,
	DEVICE_15 = 0xFE,
};

/*!
 * @brief defines all possible write registers
 * */
enum class UltraSonicRegistersWrite : alt_u8 {
	COMMAND 	= 0x00, // can init a ranging
	MAX_GAIN 	= 0x01, // maximum gain value (range + max_gain must be tested for best results)
	RANGE		= 0x02, // time need for one measurement
};


/*!
 * @brief defines all possible read registers
 * */
enum class UltraSonicRegisterRead : alt_u8 {
	SW_REVISION		= 0x00,
	LIGHT_SENSOR	= 0x01,
	ECHO_0x01		= 0x02,
	ECHO_0x02		= 0x04,
	ECHO_0x03		= 0x06,
	ECHO_0x04		= 0x08,
	ECHO_0x05		= 0x0A,
	ECHO_0x06		= 0x0C,
	ECHO_0x07		= 0x0E,
	ECHO_0x08		= 0x10,
	ECHO_0x09		= 0x12,
	ECHO_0x0A		= 0x14,
	ECHO_0x0B		= 0x16,
	ECHO_0x0C		= 0x18,
	ECHO_0x0D		= 0x1A,
	ECHO_0x0E		= 0x1C,
	ECHO_0x0F		= 0x1E,
	ECHO_0x10		= 0x20,
	ECHO_0x11		= 0x22,
};

/*!
 * @brief defines all possible commands for the ultrasonic sensor
 * */
enum class UltraSonicCommands : alt_u8 {
	START_MEAS_INCHES			= 0x50,
	START_MEAS_CM				= 0x51,
	START_MEAS_TIME_MICROSEC 	= 0x52,
	START_MEAS_INCHES_ANN		= 0x53,
	START_MEAS_CM_ANN			= 0x54,
	START_MEAS_TIME_MICROSEC_ANN= 0x55,
	CHANGE_ADDRESS_COMMAND_1	= 0xA0,
	CHANGE_ADDRESS_COMMAND_2	= 0xAA,
	CHANGE_ADDRESS_COMMAND_3	= 0xA5,
};

/*!
 *@brief represents a ultrasonic hardware device
 * */
class UltraSonicDevice {
private:
	/*! @brief no default cstr is allowed */
	UltraSonicDevice() = delete;

	/*! @brief holds the current device address */
	UltraSonicAddress __deviceAddress;
public:
	/*! @brief constructs a ultrasonic device with the given address */
	UltraSonicDevice(const UltraSonicAddress deviceAddress);
	/*!
	 * @brief function to write to the command srf08 register;
	 * @param[in] val: value which will be written to reg
	 * @param[in] broadcast: if true the command will be sent with address 0x00, which indicates a broadcast
	 * @return result (status) of this operation
	 * */
	alt_u8 writeCMDRegister(const UltraSonicCommands val, const bool broadcast = false) const;
	/*!
	 * @brief function to write to the gain srf08 register
	 * @param[in] val: value which will be written to reg
	 * @return result (status) of this operation
	 * */
	alt_u8 writeGAINRegister(const alt_u8 val) const;
	/*!
	 * @brief function to write to the range srf08 register
	 * @param[in] val: value which will be written to reg
	 * @return result (status) of this operation
	 * */
	alt_u8 writeRANGERegister(const alt_u8 val) const;
	/*!
	 * @brief function to read from specific srf08 register (reads always high and low byte if available)
	 * @param[in] reg: register to read from
	 * @param[out] readPtr: stores the read value from reg
	 * @return result (status) of this operation
	 * */
	alt_u8 readRegister(const UltraSonicRegisterRead reg, alt_u16& readPtr) const;

	/*!
	 * @brief function to read one complete range measurement
	 * @param[out] ultrasonic_measurement: buffer to store the current measurement
	 * @param[in] length: maximal length to read
	 * @return result (status) of this operation
	 * */
	alt_u8 readMeasurement(alt_u8* ultrasonic_measurement, const alt_u8 length) const;

	/*!
	 * @brief function to change the IIC address of the ultrasonic devicer
	 * @param[in] newAddress: the new address that should be given to the device
	 * @return result (status) of this operation
	 * */
	alt_u8 changeAddress(const UltraSonicAddress newAddress);

	/*!
	 * @brief function to check if the device does currently a ranging
	 * @param[out] check: will be set to true if ranging is currently ongoing otherwise set to false
	 * @return result (status) of this operation
	 * @warning do not use this function with the RTOS,
	 * */
	alt_u8 checkUltraSonicState(bool& check) const;

};

#endif /* ULTRASONIC_HPP_ */
