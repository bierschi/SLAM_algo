//
// Created by christian on 15.11.18.
//

#include <iostream>
#include "ClientSocket.h"

int main(int argc, char** argv) {

    std::cout << "Test Client" << std::endl;

    std::string host = "localhost";
    unsigned int port = 2501;

    ClientSocket* client = new ClientSocket(host, port);
    Commands save_map = SAVE_MAP;
    Commands reset_map = RESET_MAP;

    while (true) {
        client->sending(save_map);
        sleep(2);
        //client->sending(reset_map);
    }


    return 0;
}
