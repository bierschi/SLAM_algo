/*!
 * @file
 * @brief contains various means for error coding
 */

#ifndef COMMON_ALF_ERNO_H_
#define COMMON_ALF_ERNO_H_

/*!
 * @brief contains error codes for all errors which could occur during execution of the application and the information could be
 * 		  interesting for error handling
 */
enum ALF_ERROR_CODES{
	ALF_BUFFER_READ_IS_WRITE = -100,
	ALF_BUFFER_NOTHING_TO_READ,
	ALF_BUFFER_IS_FULL,
	ALF_NOTHING_IN_BUFFER,
	ALF_NO_COMMUNICATION_FILE,
	ALF_IO_ERROR,
	ALF_SOCKET_NOT_READY,
	/// the serverconnection can not be opened, there are some errors in catching the port, opening the file etc.
	ALF_SOCKET_SERVER_NOT_READY,
	ALF_CANNOT_SEND_MESSAGE,
	ALF_CANNOT_READ_SOCKET,
	//insert new error codes here
	ALF_NO_WELL_FPGABridge_MAPPING,
	ALF_LOCK_MEMORY_FAILED,
	ALF_WRITE_SHARED_MEMORY_DISABLED,
	//no changes downto here
	ALF_UNKNOWN_ERROR = -1,
	/// alright, there was no error in the functionality
	ALF_NO_ERROR = 1
};

/*!
 * @brief the error codes are available within a type
 */
typedef enum ALF_ERROR_CODES alf_error;


#endif /* COMMON_ALF_ERNO_H_ */
