/*!
 * @file
 * */
#include "Client_Server_impl.hpp"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netdb.h>

using std::string;


uint8_t Client::startConnection(const uint32_t& _portno, const string& _server)
{
	uint8_t ret_val = ALF_SOCKET_NOT_READY;
	__portno = _portno;
	__sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(__sockfd > 0)
	{
		__server = gethostbyname(_server.c_str());
		if(__server != nullptr)
		{
			bzero((char *) &__serv_addr, sizeof(__serv_addr));
			__serv_addr.sin_family = AF_INET;
			bcopy((char *)__server->h_addr, (char *)&__serv_addr.sin_addr.s_addr, __server->h_length);
			__serv_addr.sin_port = htons(__portno);

			if (connect(__sockfd,(struct sockaddr *) &__serv_addr,sizeof(__serv_addr)) == 0)
			{
				ret_val = ALF_NO_ERROR;
				__valid = true;
			}
		}
	}
	return ret_val;
}

alf_error Client::sendOverSocket(const string& data)
{
	alf_error ret_val;
	if(__valid){
		if(send(__sockfd, data.c_str(), data.length(), 0) <= 0)
			ret_val = ALF_CANNOT_SEND_MESSAGE;
		else
			ret_val = ALF_NO_ERROR;
	}
	else{
		ret_val = ALF_SOCKET_NOT_READY;
	}
	return ret_val;
}

alf_error Client::readOverSocket(string& s)
{
	alf_error ret_val = ALF_CANNOT_READ_SOCKET;
	s.clear();
	char hlp;
	if(__valid)
	{
		do {
			if( read(__sockfd, &hlp, 1) == 1)
			{
				s.append(&hlp,1);
			}
			else
			{

			}
		} while (hlp != ';');
		ret_val = ALF_NO_ERROR;
	}
	return ret_val;
}


void Client::closeConnection(void)
{
	close(__sockfd);
	__valid = false;
}

alf_error Server::startConnection(const uint32_t& portno){
	alf_error ret_val = ALF_SOCKET_SERVER_NOT_READY;
	ALF_LOG_WRITE("Starting the server...", log_info);
	__sockfd = socket(AF_INET, SOCK_STREAM, 0);
	__portnumber = portno;
	__valid = false;
	if(__sockfd >= 0){
		bzero((char *) &__server_address, sizeof(__server_address));
		__server_address.sin_family = AF_INET;
		__server_address.sin_addr.s_addr = INADDR_ANY;
		__server_address.sin_port = htons(__portnumber);
		if (bind(__sockfd, (struct sockaddr *) &__server_address, sizeof(__server_address)) < 0) {
			ALF_LOG_WRITE("Cannot open the given portnumber", log_error);
			ret_val = ALF_SOCKET_SERVER_NOT_READY;
		}
		else{
			listen(__sockfd,5);
			__clientlen = sizeof(__client_address);
			__newsocketfd = accept(__sockfd, (struct sockaddr *)&__client_address, &__clientlen);
			if(__newsocketfd < 0){
				ret_val = ALF_SOCKET_SERVER_NOT_READY;
				ALF_LOG_WRITE("Cannot accept the client", log_error);
			}
			else{
				__valid = true;
				ret_val = ALF_NO_ERROR;
			}
		}

	}
	else{
		ALF_LOG_WRITE("Cannot open the port and start the server...", log_error);
		ret_val = ALF_SOCKET_SERVER_NOT_READY;
	}


	return ret_val;
}

void Server::closeConnection(void){
	close(__newsocketfd);
	__valid = false;
}

alf_error Server::readOverSocket(string& s)
{
    alf_error ret_val = ALF_CANNOT_READ_SOCKET;
    s.clear();
    char hlp;
    if(__valid)
    {
        do {
            if( read(__newsocketfd, &hlp, 1) == 1)
            {
                s.append(&hlp,1);
            }
            else
            {

            }
        } while (hlp != ';');
        ret_val = ALF_NO_ERROR;
    }
    return ret_val;
}

alf_error Server::sendOverSocket(const string &data){
	alf_error ret_val;
	if(__valid){
		if(send(__newsocketfd, data.c_str(), data.length(), 0) <= 0)
			ret_val = ALF_CANNOT_SEND_MESSAGE;
		else
			ret_val = ALF_NO_ERROR;
	}
	else{
		ret_val = ALF_SOCKET_NOT_READY;
	}
	return ret_val;
}
