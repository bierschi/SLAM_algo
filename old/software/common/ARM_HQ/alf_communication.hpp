/*!
 * @file
 * @brief a library for handling all the communication between a client and a server. This file contains all types of
 * communications like writing to files or socket communication over LAN
 */

#ifndef ALF_COMMUNICATION
#define ALF_COMMUNICATION

#include <string>
#include <sstream>
#include <iostream>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <regex>

#include "alf_data.hpp"
#include "alf_data_info.hpp"
#include "alf_log.hpp"
#include "alf_erno.h"
#include "alf_message_types.hpp"

#include "Client_Server_impl.hpp"

using std::string;


/*!
 * @brief CommunicationClass that handles all the communication. Possible template parameters are at the moment std::fstream, Client and Server. No other com-types are supported
 */
template<class _comType>
class Alf_Communication{
private:
	_comType __comHandler;
	/*!
	 * @brief delimiters for the messages needed by the Alf_Communication
	 */
	static constexpr char __delim = '|';
	static constexpr char __end_delim = ';';
	/*!
	 * @brief functions for every template type that is allowed, only for type handling
	 */
	uint8_t readLine(Client& comtype,string& s) { return comtype.readOverSocket(s); }
	uint8_t readLine(Server& comtype,string& s) { return comtype.readOverSocket(s); }
	uint8_t readLine(std::fstream& comtype, string& s);

	alf_error __writeLine(Client& comtype, string &s) { return comtype.sendOverSocket(s); };
	alf_error __writeLine(Server& comtype, string &s) { return comtype.sendOverSocket(s); };
	alf_error __writeLine(std::fstream& comtype, string &s);

	void closeCom(std::fstream& comtype) { comtype.close(); }
	void closeCom(Client& comtype) { comtype.closeConnection(); }
	void closeCom(Server& comtype) { comtype.closeConnection();	}

public:
	/*!
	 * @brief Init, for communication as a file
	 * @param[in] filename - for the file, which will be used as communication
	 * @return true when everything fine, false otherwise
	 */
	bool Init(const string& filename);
	/*!
	 * @brief Init, for communication as a client
	 * @param[in] server - the server IP as a string for the connection
	 * @param[in] portno - the portnumber for the communication
	 * @return true when everything fine, false otherwise
	 */
	bool Init(const string& server, const uint32_t& portno);
	/*!
	 * @brief Init, for communication as a server
	 * @param[in] portno - the portnumber for the communication
	 * @return true when everything fine, false otherwise
	 */
	bool Init(const uint32_t& portno);
	/*!
	 * @brief Writes len bytes from data
	 * @param[in] file - the fstream, where the bytes should be written
	 * @param[in] data - the pointer to the data which shall be written to the file
	 * @param[in] len - number of bytes from data, which should be written
	 * @return - true when everything is fine, false otherwise
	 */
	bool Write(std::fstream& file, const char* data, const uint32_t& len);
	/*!
	 * @overload
	 * @param[in] cl - the client, writes to a socket
	 * @param[in] data - the pointer to the data which shall be written to the file
	 * @param[in] len - number of bytes from data, which should be written
	 * @return - true when everything is fine, false otherwise
	 */
	bool Write(Client& cl, const char* data, const uint32_t& len);
	/*!
	 * @overload
	 * @param[in] ser - the server, writes to a socket
	 * @param[in] data - the pointer to the data which shall be written to the file
	 * @param[in] len - number of bytes from data, which should be written
	 * @return - true when everything is fine, false otherwise
	 */
	bool Write(Server& ser, const char* data, const uint32_t& len);

	/*!
	 * @brief This function writes the a buffer to the communication type. Only calling the internal Write(Alf_Urg_Measurement&) function until the buffer is empty
	 * @param[inout] buffer - the queue which includes all of the measurmenets which was taken to the moment, the function is called. It will be changed on calling this function.
	 * @return - alf_error code
	 */
	alf_error Write(Alf_Urg_Measurements_Buffer &buffer);

	/*!
	 * @brief Creates a string with all, for our application, relevant information for one laser-scanner measurement.
	 * 		  The structure of this string is described in Alf_Messages.ods, outside this inline documentation
	 * @param[in] meas - one laser scanner measurement
	 * @return
	 */
	alf_error Write(Alf_Urg_Measurement &meas);

    /*!
     * @brief Creates a string with all, for application relevant information for steering the Alf
     * @param[in] Command - command object
     * @return
     */
    alf_error Write(Alf_Drive_Command &command);

    /*!
     * @brief Creates a string with all, for application relevant information for driving infos
     * @param[in] Info - Info object
     * @return
     */
    alf_error Write(Alf_Drive_Info &info);

	/*!
	 * @brief Writes the init message over the choosen communication type with information about the urg sensor
	 * @return
	 * 			- ALF_NO_ERROR if all is ok and it works
	 * 			- ALF_CANNOT_SEND_MESSAGE if the communication does not work
	 */
	alf_error WriteInitMessage();

	/*!
	 * @brief Reads len bytes and stores them into readPtr
	 * @param[in] file - the fstream from which shall be readed
	 * @param[inout] readPtr - where the function shall store the readed data
	 * @param[in] len - how much bytes shall be readed
	 * @return -
	 */
	bool Read(std::fstream& file, char* readPtr, const uint32_t& len);
	/*!
	 * @overload
	 * @param[in] cl - the client socket where the data shall be readed
	 * @param[inout] readPtr - where the function shall store the readed data
	 * @param[in] len - how much bytes shall be readed
	 * @return -
	 */
	bool Read(Client& cl, char* readPtr, const uint32_t& len);
	/*!
	 * @overload
	 * @param[in] ser - the server socket where the data shall be readed
	 * @param[inout] readPtr - where the function shall store the readed data
	 * @param[in] len - how much bytes shall be readed
	 * @return -
	 */
	bool Read(Server& ser, char* readPtr, const uint32_t& len);

	/*!
	 * @brief Function reads nrPackToRead Messages and stores them to the readBuffer. If the buffer has not enough free entries, no data is read and nothing is changed.
	 * 			Then another read is possible when the buffer has enough free entries.
	 * @param[in] readBuffer - This buffer is the memory location for the read data
	 * @param[in] nrPackToRead - default is one packet, otherwise this is the number of packets which will be read
	 * @return the first error that occurred or ALF_NO_ERROR when successful
	 */
	alf_error Read(Alf_Urg_Measurements_Buffer& readBuffer, alf_mess_types& msgType, const uint32_t& nrPackToRead = 1);

	/*!
	 * @brief Function to end the communication
	 * @return - true if everything works, false otherwise
	 */
	bool EndCommunication(void);
};

#include "alf_communication.tpp"

#endif
