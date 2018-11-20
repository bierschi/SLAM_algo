//
// Created by christian on 20.11.18.
//

#ifndef ALF_SERVERSOCKET_H
#define ALF_SERVERSOCKET_H

#include <thread>
#include <vector>
#include <iomanip>
#include <unistd.h>

#include "Socket.h"

/**
 * /CLASS ServerSocket
 *
 * creates a ServerSocket object
 */
class ServerSocket : private Socket {

private:
    unsigned int port_;

public:

    ServerSocket();
    ServerSocket(unsigned int port);
    virtual ~ServerSocket();

    const ServerSocket& sending(const std::string& s) const;
    const ServerSocket& sending(Commands& cmd) const;
    const ServerSocket& sending(std::vector<int>& v);
    const ServerSocket& receiving (std::string& ) const;
    const ServerSocket& receiving (Commands& ) const;

    int getPort() const;

    void accept(ServerSocket&);


};

#endif //ALF_SERVERSOCKET_H
