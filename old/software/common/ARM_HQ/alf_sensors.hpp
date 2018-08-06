/*!
 * @file
 * @brief contains datatypes and functionalitys for sensors on the alf vehicle
 */

#ifndef HAL_ALF_SENSORS_HPP_
#define HAL_ALF_SENSORS_HPP_

#include <stdint.h>
#include <string>

//#include <urg_connection.h>

/*!
 * @brief Represents the laser scanner on the alf vehicle and provide common settings etc.
 * @attention this settings are only vaild with the URG-04LX
 */
/*class Alf_Urg_Sensor {
public:
	/// how much measurement points does the sensor have
	static const uint16_t measurement_points = 768;
	/// the baudrate to communicate with the scanner
	static const long alf_urg_baudrate = 115200;
	/// the port on which the scanner is connected with the hardware
	static const std::string alf_urg_device_port;
	/// which communication type we use
	static const urg_connection_type_t alf_urg_connection_type = URG_SERIAL;

private:
};
*/


#endif /* HAL_ALF_SENSORS_HPP_ */
