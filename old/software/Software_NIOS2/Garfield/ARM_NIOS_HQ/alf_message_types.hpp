/*!
 * @file
 * @brief contains enumeration for easy identification of various messages
 */

#ifndef HAL_ALF_MESSAGE_TYPES_HPP_
#define HAL_ALF_MESSAGE_TYPES_HPP_

/*!
 * @brief contains the IDs for all of the messages which can be sended
 */
enum ALF_MESSAGE_TYPES{
	/// initalisation data of the laser scanner
	ALF_INIT_ID = 2,
	/// a measurement is sended
	ALF_MEASUREMENT_DATA_ID = 1,
    /// ALF drive command
    ALF_DRIVE_COMMAND_ID = 3,
    /// ALF drive info
    ALF_DRIVE_INFO_ID = 4,
	/// the communication should stop or interrupt now
	ALF_END_ID = 255
};

typedef enum ALF_MESSAGE_TYPES alf_mess_types;


#endif /* HAL_ALF_MESSAGE_TYPES_HPP_ */
