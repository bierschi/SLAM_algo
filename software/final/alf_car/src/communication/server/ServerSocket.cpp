//
// Created by christian on 20.11.18
//

#include "ServerSocket.h"
#include "SocketException.h"

/**
 * Default Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket();
 *
 */
ServerSocket::ServerSocket()
{


}

/**
 * Constructor for a ServerSocket instance
 *
 * USAGE:
 *      ServerSocket* sock = new ServerSocket(2500, 2);
 *
 * @param port: int
 */
ServerSocket::ServerSocket(unsigned int port) : port_(port) 
{
    
    if ( !Socket::create() ) {

        throw SocketException("Could not create server socket!");

    }

    if ( !Socket::bind(port_) ) {

        throw SocketException("Could not bind to port!");

    }

    if ( !Socket::listen() ) {

        throw SocketException("Could not listen to socket");

    }
    

}

/**
 * Destructor in ServerSocket
 */
ServerSocket::~ServerSocket() {
    
}

/**
 * get port of Server
 *
 * @return unsigned int port
 */
int ServerSocket::getPort() const {
    return port_;
}


/**
 * sending strings to sockets
 *
 * @param s: const string reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::sending (const std::string &s) const {

    if ( !Socket::send(s)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 * sending predefined commands to sockets
 *
 * @param cmd: Commands& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::sending (Commands& cmd) const {

    if ( !Socket::send(cmd)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 * sending a int vector to sockets
 *
 * @param v: std::vector<int>& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::sending (std::vector<int>& v) {

    if ( !Socket::send(v)) {

        throw SocketException("Could not write to socket!");

    }

    return *this;

}

/**
 * receiving strings from sockets
 *
 * @param s: string& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::receiving(std::string &s) const {

    if ( !Socket::recv(s)) {

        throw SocketException("Could not read from socket!");

    }

    return *this;
}

/**
 * receiving predefined commands from sockets
 *
 * @param cmd: Commands& reference
 * @return const ServerSocket& reference
 */
const ServerSocket& ServerSocket::receiving (Commands& cmd) const {

    if ( !Socket::recv(cmd) ) {

        throw SocketException("Could not read cmd from socket!");

    }

    return *this;
}

/**
 * blocking method, waits for a client connection
 *
 * @param sock: ServerSocket& reference
 */
void ServerSocket::accept(ServerSocket &sock) {

    if ( !Socket::accept(sock)) {

        throw SocketException("Could not accept socket!");

    }
}


