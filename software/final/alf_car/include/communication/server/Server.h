//
// Created by christian on 02.11.18.
//

#ifndef ROBOCAR_SERVER_H
#define ROBOCAR_SERVER_H

#include "communication/server/ServerSocket.h"
#include "communication/server/SocketException.h"
#include "slam/SlamMap.h"

#include "std_msgs/String.h"


/**
 * /CLASS Server
 *
 * creates a Server object
 */
class Server {

private:
    unsigned int port_;
    ServerSocket* serverSocket;
    ServerSocket* sock;
    Commands cmd;

    SlamMap& slamMap_;

public:
    Server(unsigned int port, SlamMap& slamMap);
    ~Server();

    void startThread();
    void waitForClient();
    void run();
    void actions(Commands& cmd);
    void sendDataAtStart();

};

#endif //ROBOCAR_SERVER_H
