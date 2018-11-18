//
// Created by christian on 18.11.18.
//

#ifndef ALF_GUI_CLIENTSOCKET_H
#define ALF_GUI_CLIENTSOCKET_H

#include "Socket.h"

/**
 * /CLASS ClientSocket
 *
 * creates a ClientSocket object
 */
class ClientSocket : private Socket {

public:
    // constructor/destructor
    ClientSocket(std::string host, int port);
    virtual ~ClientSocket(){};

    const ClientSocket& sending (const std::string&) const;
    const ClientSocket& sending (Commands&) const;

    const ClientSocket& receiving (std::string&) const;
    const ClientSocket& receiving (Commands&) const;
    const ClientSocket& receiving (std::vector<int>& v);

    void closeConnection();
};

#endif //ALF_GUI_CLIENTSOCKET_H
