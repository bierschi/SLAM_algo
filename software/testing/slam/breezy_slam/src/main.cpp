#include <iostream>
#include "URG04LX.hpp"

static const char* DEVICE = "/dev/ttyACM0";
static const int NITER = 20;

int main() {
    std::cout << "Start scanning!" << std::endl;

    URG04LX laser;
    laser.connect(DEVICE);

    for(int i = 1; i<NITER; i++){

        unsigned int data[1000];
        int ndata = laser.getScan(data);
        std::cout << "Iteration: " << i << std::endl;

        if (ndata) {
            std::cout << "got " << ndata << " data points" << std::endl;

        }
        else {
            std::cout << "SCAN FAILED " << std::endl;
        }
    }


    return 0;
}