/*
urtest.cpp : C++ lest program for BreezyLidar on Hokuyo URG-04LX

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "URG04LX.hpp"

//static const char * DEVICE = "/dev/tty.usbmodem1421";
static const char * DEVICE = "/dev/ttyACM0";
static const std::string APP_FOLDER = "../LidarOutput/";
//static const std::string APP_FOLDER = "/home/dbroot/Studium/Master/OTH/HSP_1/UnserProjekt/Software/LidarConnectProj/LidarOutput/";


static const int MEASUREMENT_POINTS = 768;  //size of array with lidar points
static const float MEAS_ERROR_VAL = 5000.0f;  //size of array with lidar points
static const float MEAS_CORRECTION_VAL = 0.0f;  //size of array with lidar points
static const bool APPEND = true;
static const bool TRUNCATE = false;

static const float PI = 3.14159265;

static const int FIRST_MEASUREMET_POINT = 0;
static const int FIRST_POINT_DETECT_RANGE = 44;
static const int FRONT_VIEW = 384;
static const int LAST_POINT_DETECT_RANGE = 725;
static const int LAST_MEASUREMENT_POINT = 768;
static const int STEP_RESOLUTION = 1024;

static const float DETECTION_RANGE = (LAST_POINT_DETECT_RANGE - FIRST_POINT_DETECT_RANGE)  / STEP_RESOLUTION;   // 239.4140625 degrees
static const float MEASUREMENT_RANGE = LAST_MEASUREMENT_POINT / STEP_RESOLUTION;                                // 270.0 degrees
static const float STARTING_ANGLE = 60.0F;
static const float ANGLE_STEP = 0.3515625; // 1024 / 360


URG04LX laser;


int main();
void getDataFromLidar(unsigned long int *data);
void getDataFromFile(int lineNr, unsigned long int *data, std::string const&path);
void convertLidarData(unsigned long int data[MEASUREMENT_POINTS], float dataXY[][2]);
void saveRawDataToFile(unsigned long int *data, std::string const&path, bool append);
void saveDataXYAsCoordinates(float dataXY[][2], std::string const&path, bool append);
void print2view(unsigned long int *data, float dataXY[][2]);

int main()
{
    unsigned long int data[MEASUREMENT_POINTS];    // raw data
    float dataXY[MEASUREMENT_POINTS][2];           // converted data

//    for (int i=0; i<200; i++)
//    {
//        getDataFromLidar(data);  // evtl probleme, da .getScan nur unsigned int erwartet statt long int!
//        saveRawDataToFile(data, APP_FOLDER + "/scans_saved.txt", APPEND);
//    }

    getDataFromFile(0, data, APP_FOLDER + "scans.txt");

    convertLidarData(data, dataXY);

    print2view(data, dataXY);

    saveDataXYAsCoordinates(dataXY, APP_FOLDER + "xyCoords.csv", TRUNCATE);

    return 0;
}


void getDataFromLidar(unsigned long int *data)
{
    std::cout << "===============================================================" << endl;
    std::cout << "connecting to lidar..." << endl;
    laser.connect(DEVICE);

    std::cout << "===============================================================" << endl;
    std::cout << "lidar info: " << endl;
    std::cout << laser << endl;

    std::cout << "===============================================================" << endl;
    std::cout << "get data from lidar..." << endl;

    int ndata = laser.getScan( reinterpret_cast<unsigned int *>(data) );

    if (ndata)
    {
        std::cout << "got "<< ndata << " data points" << endl;

        std::cout << "===============================================================" << endl;
        std::cout << "remove error values (d>" << MEAS_ERROR_VAL << ")" << endl;
        for (int i = 0; i<MEASUREMENT_POINTS; i++)
            if (data[i] > MEAS_ERROR_VAL)
                data[i] = MEAS_CORRECTION_VAL;
    }
    else
    {
        std::cout << "=== SCAN FAILED ===" << endl;
    }

    std::cout << "===============================================================" << endl;


}

void getDataFromFile(int lineNr, unsigned long int *data, std::string const&path)
{
    int i = 0;
    ifstream f_lidarOutput(path);

    for (std::string line; std::getline(f_lidarOutput, line); )
    {
        if(i < lineNr) {
            i++;
            continue;
        }
        for(int i = 0; i < MEASUREMENT_POINTS; i++)
        {
            std::string token = line.substr(0, line.find(" "));
            line = line.substr(line.find(" ")+1, line.length());
            data[i] = std::atol(token.c_str());
            if (data[i] > MEAS_ERROR_VAL)
                data[i] = MEAS_CORRECTION_VAL;
        }
        break;
    }
    f_lidarOutput.close();

}

void convertLidarData(unsigned long int data[MEASUREMENT_POINTS], float dataXY[][2])
{
    float angle = STARTING_ANGLE;
    unsigned long int dist = 0;

    for (int i = 0; i<MEASUREMENT_POINTS; i++ )
    {
        dist = data[i];
        dataXY[i][0] = sin(angle *PI/180.0f) * (float) dist;           // x = sin(a) * d
        dataXY[i][1] = cos(angle *PI/180.0f) * (float) dist;           // y = cos(a) * d

        if(angle <= 90)
        {
            dataXY[i][0] *=  1.0f;
            dataXY[i][1] *= -1.0f;
        }else if(angle <= 180)
        {
            dataXY[i][0] *= 1.0f;
            dataXY[i][1] *= -1.0f;
        } else if(angle <= 270)
        {
            dataXY[i][0] *=  1.0f;
            dataXY[i][1] *= -1.0f;
        } else if(angle <= 360)
        {
            dataXY[i][0] *= 1.0f;
            dataXY[i][1] *=  -1.0f;
        }
        angle += ANGLE_STEP;
    }
}

void saveRawDataToFile(unsigned long int *data, std::string const&path, bool append)
{
    ofstream f_lidarOutput;
    if(append)
        f_lidarOutput.open(path, std::ios_base::app);
    else
        f_lidarOutput.open(path, std::ios_base::trunc);

    for(int j = 0; j<MEASUREMENT_POINTS; j++)
        f_lidarOutput << data[j]<< " ";

    f_lidarOutput << "\n";

    f_lidarOutput.close();
}

void saveDataXYAsCoordinates(float dataXY[][2], std::string const&path, bool append)
{
    ofstream f_lidarOutput;
    if(append)
        f_lidarOutput.open(path, std::ios_base::app);
    else
        f_lidarOutput.open(path, std::ios_base::trunc);

    for(int j = 0; j<MEASUREMENT_POINTS; j++)
        f_lidarOutput << std::fixed << dataXY[j][0] << ";" << dataXY[j][1] << endl;

    f_lidarOutput.close();

}


void print2view(unsigned long int *data, float dataXY[][2])
{
    for(int i = 0; i<MEASUREMENT_POINTS; i++)
    {
        if(i == 0)
            std::cout << "\t\t ########### first measurement point #########"<<endl;
        else if (i == FIRST_POINT_DETECT_RANGE)
            std::cout << "\t\t ########### start of detection range #########"<<endl;
        else if (i == FRONT_VIEW)
            std::cout << "\t\t ########### front view #########"<<endl;
        else if (i == LAST_POINT_DETECT_RANGE)
            std::cout << "\t\t ########### end of detection range #########"<<endl;
        else if (i == LAST_MEASUREMENT_POINT)
            std::cout << "\t\t ########### last measurement point #########"<<endl;

        std::cout << "Point " << i << ": " << data[i] << " \t\tangle: " << 60+i*ANGLE_STEP <<"\t\t x: " << dataXY[i][0] << " y: " << dataXY[i][1] << endl;
    }
}
