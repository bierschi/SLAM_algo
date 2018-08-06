#ifndef ALF_DATA_INFO
#define ALF_DATA_INFO
#include "stdint.h"

/*!
 * @brief The Alf_Drive_Info class holds the Infos for steering the Alf
 */
class Alf_Drive_Info {
public:
    /// This is the current speed
    uint8_t speed;
    /// This is the acceleration of the car
    float acceleration;
    /// This is the lateral acceleration of the car
    float lateral_acceleration;
    /// This is the acceleration in Z direction
    float z_acceleration;
    /// This is the Gyroscope value x axis
    float Gyroscope_X;
    /// This is the Gyroscope value y axis
    float Gyroscope_Y;
    /// This is the Gyroscope value z axis
    float Gyroscope_Z;
    /// This is the temperature
    float temperature;
};

class Alf_Drive_Command {
public:
    /// This variable holds the current speed (0 - 100%)
    uint8_t speed;
    /// This is the direction to drive (0: forward, 1: backward)
    uint8_t direction;
    /// This is the currents steering angle (-90 - 90°)
    int8_t angle;
    /// This holds the state of the light
    bool light;
};

/// global variables
extern Alf_Drive_Info global_drive_info;
extern Alf_Drive_Command global_drive_command;

#endif
