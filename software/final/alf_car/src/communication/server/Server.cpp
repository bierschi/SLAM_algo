//
// Created by christian on 02.11.18.
//

#include "communication/server/Server.h"

bool Server::connected = false;
/**
 * Constructor for a Server instance
 *
 * USAGE:
 *      Server* server = new Server(2501, *car);
 *
 * @param port
 */
Server::Server(unsigned int port, SlamMap& slamMap) : port_(port),
                                                      slamMap_(slamMap)
{

    serverSocket = new ServerSocket(port);
    sock = new ServerSocket();

    std::cout << "Server is being set up on port: " << port_ << std::endl;

    startThread();
}

/**
 * Destructor in Server
 */
Server::~Server() {

    delete serverSocket;

}

/**
 * static method to query if the server is connected
 *
 * @return bool connected
 */
bool Server::isConnected() {
    return connected;
}

/**
 * starts the run thread
 */
void Server::startThread() {

    std::thread server(&Server::run, this);
    server.detach();

}

/**
 * listening for a new client connection
 */
void Server::waitForClient() {

    std::cout << "Listening for new client ..." << std::endl;
    serverSocket->accept(*sock);
    connected = true;

}

/**
 * Server run thread to handle incoming commands
 */
void Server::run() {

    waitForClient();

    try {

            //sendDataAtStart();
            while (true) {
                // receiving command from client
                sock->receiving(cmd);
                // execute this command
                actions(cmd);

            }

        } catch (SocketException& e) {

            std::cout << "SocketException was caught: " << e.description() << std::endl;
            connected = false;
            // stop slammap thread
            run();
        }
}

/**
 *
 */
void Server::sendDataAtStart() {

    //car_.sendSlamMap(*sock);

}

/**
 * selects a appropriate action, depending on the incoming command
 *
 * @param cmd: Commands reference to execute defined commands
 */
void Server::actions(Commands& cmd) {

    switch(cmd) {


            //SlamMap
        case START_STREAM_MAP: {
            std::cout << "Start Stream Map!" << std::endl;
            slamMap_.startSendSlamMap(*sock);
        }
            break;

        case STOP_STREAM_MAP:
            std::cout << "Stop Stream Map!" << std::endl;
            slamMap_.stopSendSlamMap();
            break;

        case SAVE_MAP: {
            std::cout << "Save SLAM Map" << std::endl;
            slamMap_.setSaveMap(true);
        }
            break;

        case RESET_MAP: {
            std::cout << "Reset SLAM Map" << std::endl;
            slamMap_.resetMap();
        }
            break;


        default:
            std::cout << "Default in method Server::actions!" << std::endl;
    }
}
