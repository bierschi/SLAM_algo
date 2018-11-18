//
// Created by christian on 18.11.18
//

#ifndef ALF_GUI_SOCKET_EXCEPTION_H
#define ALF_GUI_SOCKET_EXCEPTION_H

/**
 * /CLASS SocketException
 *
 * handles Socket Exception
 *
 */
class SocketException {

private:
    std::string m_s;

public:

    SocketException( std::string s ) : m_s(s) {}
    ~SocketException(){}

    std::string description() {return m_s;}

};

#endif //ALF_GUI_SOCKETEXCEPTION_H
