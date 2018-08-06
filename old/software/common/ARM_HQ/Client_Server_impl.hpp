/*!
 * @file
 * */
#include <string>

#include <unistd.h>
#include <netinet/in.h>

#include "alf_erno.h"
#include "alf_log.hpp"

using std::string;
/*!
 * @brief
 */
class Client {
public:
	/*!
	 * @brief Sending the string to over the socket via the underlying linux functaion.
	 * @param[in] data - the string with the message which shall be transmitted
	 * @return
	 * 			- ALF_NO_ERROR if the message can be transmitted
	 * 			- ALF_SOCKET_NOT_READY if the socket is not initialised and
	 * 			- ALF_CANNOT_SEND_MESSAGE if there are errors in the linux functionalitys, typical triggered by a too long message etc.
	 */
	alf_error sendOverSocket(const string& data);
	/*!
	 * @brief reads a string object over the socket. three conditions for read ending are given
	 * 		1) if the end delimiter is reached ';'
	 * 		2) no more readable data is available
	 * 		3) more than 20 characters were read and no delimiter '|' or end delimiter ';' was read
	 * @param[in] the string data object were the read data is stored (no appending string gets overwritten)
	 * @return
	 * 			- ALF_NO_ERROR if the read works
	 * 			- ALF_CANNOT_READ_SOCKET if it does not work
	 */
	alf_error readOverSocket(string& s);
	/*!
	 * @brief starts the socket connection
	 * @param[in] the portnumber
	 * @param[in] servername
	 * @return 1 if successful otherwise error < 0
	 */
	uint8_t startConnection(const uint32_t& _portno, const string& _server);
	/*!
	 * @brief closes the connection, communication is no longer possible
	 */
	void closeConnection(void);
	/*!
	 * @brief dummy function to satisfy the compiler (std::fstream, Server/Client all have the good()
	 * 		function so no explicit type handling must be done
	 */
	bool good(){ return true;}
	/*!
	 * @brief returns the state of the socket connection
	 * @return true if connection is good, false otherwise
	 */
	bool is_open(){return __valid;}
	int32_t getSocketNumber(void) { return __sockfd;}

private:
	bool __valid;
	int32_t __sockfd;
	uint32_t __portno;
	struct sockaddr_in __serv_addr;
	struct hostent* __server;
};

/*!
 * @brief Represents the serverside of an communication for the whole application
 *
 * @attention at the moment this server implementation can only handle <b>ONE</b> connection!
 */
class Server{
public:

	/*!
	 * @brief Trys to open the given port and listen to incoming connections
	 *        It is using the underlying linux functions for socket handling.
	 * @param[in] portno - the portnumber on which the socket should be opened
	 * @return
	 * 			- ALF_SOCKET_SERVER_NOT_READY if something goes wrong (the port is blocked, the function gets no socket handler from os etc.) and
	 *   	   	- ALF_NO_ERROR if the port can be catched and the port is working
	 */
	alf_error startConnection(const uint32_t&);
	/*!
	 * @brief Closing the binded socket and close the server connection.
	 */
	void closeConnection(void);
	/*!
	 * @brief Sending the string to over the socket via the underlying linux functaion.
	 * @param[in] data - the string with the message which shall be transmitted
	 * @return
	 * 			- ALF_NO_ERROR if the message can be transmitted
	 * 			- ALF_SOCKET_NOT_READY if the socket is not initialised and
	 * 			- ALF_CANNOT_SEND_MESSAGE if there are errors in the linux functionalitys, typical triggered by a too long message etc.
	 */
	alf_error sendOverSocket(const string&);
	/*!
	 * @brief read from the underlying socket
	 * @param[in] s - a string reference
	 * @return at this moment -> nothing
	 *
	 * @attention this is just the dummy function, the implementation of this function is missing
	 */
	alf_error readOverSocket(string& s);
	/*!
	 * @brief returns the state of the socket connection
	 * @return true if connection is good, false otherwise
	 */
	bool is_open(){return __valid;}
	/*!
	 * @brief dummy function to satisfy the compiler (std::fstream, Server/Client all have the good()
	 * 		function so no explicit type handling must be done
	 */
	bool good(){ return true;}
	/*!
	 * @brief returns the socket handler id given from linux at initalisation of the socket
	 * @return the socket handler number
	 */
	int32_t getSocketNumber(void) { return __sockfd;}

private:
	bool __valid;
	int32_t __sockfd;
	uint32_t __portnumber;
	int32_t __newsocketfd;
	struct sockaddr_in __server_address, __client_address;
	uint32_t __clientlen;

};
