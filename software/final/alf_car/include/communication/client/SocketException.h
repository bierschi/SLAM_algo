//
// Created by christian on 20.11.18
//

#ifndef ALF_SOCKETEXCEPTION_H
#define ALF_SOCKETEXCEPTION_H

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

#endif //ALF_SOCKETEXCEPTION_H
