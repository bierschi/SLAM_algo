//
// Created by christian on 20.11.18.
//

#ifndef ALF_CLIENTSOCKET_H
#define ALF_CLIENTSOCKET_H

#include "Socket.h"

/**
 * /CLASS ClientSocket
 *
 * creates a ClientSocket object
 */
class ClientSocket : private Socket {

public:

    ClientSocket(std::string host, int port);
    virtual ~ClientSocket(){};

    const ClientSocket& sending (const std::string&) const;
    const ClientSocket& sending (Commands&) const;

    const ClientSocket& receiving (std::string&) const;
    const ClientSocket& receiving (Commands&) const;

    void closeConnection();
};

#endif //ALF_CLIENTSOCKET_H
