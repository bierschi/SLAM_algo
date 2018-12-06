//
// Created by christian on 02.11.18.
//

#ifndef ALF_SERVER_H
#define ALF_SERVER_H

#include "ServerSocket.h"
#include "SocketException.h"
#include "SlamMap.h"

#include "std_msgs/String.h"


/**
 * /CLASS Server
 *
 * creates a Server object
 */
class Server {

private:
    unsigned int port_;
    static bool connected;
    ServerSocket* serverSocket;
    ServerSocket* sock;
    Commands cmd;

    SlamMap& slamMap_;

    //private methods


public:
    Server(unsigned int port, SlamMap& slamMap);
    ~Server();

    static bool isConnected();

    void startThread();
    void waitForClient();
    void run();
    void actions(Commands& cmd);
    void sendDataAtStart();

};

#endif //ALF_SERVER_H
