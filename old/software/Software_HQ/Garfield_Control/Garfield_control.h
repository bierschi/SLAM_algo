/*!
 * @file
 * */
#ifndef GARFIELD_CONTROL_H
#define GARFIELD_CONTROL_H

#include <QMainWindow>
#include "QKeyEvent"

#include "joystick.h"

#include "Settings.h"

#include "alf_communication.hpp"
#include "alf_log.hpp"

/// ID of the gamepad triangle button
#define GAMEPAD_BUTTON_TRIANGLE 12
/// ID of the gamepad circle button
#define GAMEPAD_BUTTON_CIRCLE 13
/// ID of the gamepad cross button
#define GAMEPAD_BUTTON_CROSS 14
/// ID of the gamepad square button
#define GAMEPAD_BUTTON_SQUARE 15
/// ID of the gamepad L1 button
#define GAMEPAD_BUTTON_L1 10
/// ID of the gamepad R1 button
#define GAMEPAD_BUTTON_R1 11
/// ID of the gamepad direction pad up button
#define GAMEPAD_BUTTON_DPAD_UP 4
/// ID of the gamepad direction pad right button
#define GAMEPAD_BUTTON_DPAD_RIGHT 5
/// ID of the gamepad direction pad down button
#define GAMEPAD_BUTTON_DPAD_DOWN 6
/// ID of the gamepad direction pad left button
#define GAMEPAD_BUTTON_DPAD_LEFT 7
/// ID of the gamepad select button
#define GAMEPAD_BUTTON_SELECT 0
/// ID of the gamepad start button
#define GAMEPAD_BUTTON_START 3
/// ID of the gamepad analog left button
#define GAMEPAD_BUTTON_ANALOG_LEFT 1
/// ID of the gamepad analog right button
#define GAMEPAD_BUTTON_ANALOG_RIGHT 2
/// ID of the gamepad analog left axis from left to right
#define GAMEPAD_AXIS_ANALOG_LEFT_LR 0
/// ID of the gamepad analog left axis from up to down
#define GAMEPAD_AXIS_ANALOG_LEFT_UD 1
/// ID of the gamepad analog right axis from left to right
#define GAMEPAD_AXIS_ANALOG_RIGHT_LR 2
/// ID of the gamepad analog right axis from up to down
#define GAMEPAD_AXIS_ANALOG_RIGHT_UD 3
/// ID of the gamepad analog left axis L2
#define GAMEPAD_AXIS_L2 12
/// ID of the gamepad analog left axis R2
#define GAMEPAD_AXIS_R2 13

/// Gamepad value button down
#define GAMEPAD_BUTTON_DOWN 1
/// Gamepad value button up
#define GAMEPAD_BUTTON_UP 0

/// Gamepad value axis down
#define GAMEPAD_AXIS_DOWN 32767
/// Gamepad value axis up
#define GAMEPAD_AXIS_UP -32768

/*!
 * @brief Function normalizes values from given intervall to given intervall
 * @param[in] in_min - This is the minimal value of the originally intervall
 * @param[in] in_max - This is the maximal value of the originally intervall
 * @param[in] out_min - This is the minimal value of the destination intervall
 * @param[in] value - This is the value to convert to the destination intervall
 * @return the converted value is returned
 */
template<typename T>
T norm_value(T in_min, T in_max, T out_min, T out_max, T value) {
    return ((T)(((out_max-out_min)*(value-in_min))/(in_max-in_min))+out_min);
}

class Settings;

namespace Ui {
class Garfield_control;
}

/*!
 * @brief Garfield_control is the main class that provides all functionalities for the Garfield control program
 */
class Garfield_control : public QMainWindow {

    Q_OBJECT

    public:
        /*!
         * @brief The constructor
         */
        Garfield_control(QMainWindow *parent = 0);
        /*!
         * @brief The destructor
         */
        ~Garfield_control();
        /*!
         * @brief connect_gamepad() function connects the gamepad. It takes the _Dev object which contains the device name
         */
        bool connect_gamepad();
        /*!
         * @brief keyPressEvent handles all pressed keys which are necessary for controling the car. After that the keyPressEvent of the base class is called
         * @param[in] e - The QKeyEvent of the key that is pressed.
         */
        void keyPressEvent(QKeyEvent* e);
        /*!
         * @brief keyReleaseEvent handles all released keys which are necessary for controling the car. After that the keyReleaseEvent of the base class is called
         * @param[in] e - The QKeyEvent of the key that is released.
         */
        void keyReleaseEvent(QKeyEvent* e);
        /*!
         * @brief ladSettings() loads the settings file and stores all settings in its variables
         */
        void loadSettings();
        /*!
         * @brief saveSettings() saves all settings to the Garfield.conf file if the settings window gets closed
         */
        void saveSettings();
        /*!
         * @brief getIP() is the getter function for the IP
         */
        void getIP(QString *IP);
        /*!
         * @brief getPort() is the getter function for the Port
         */
        void getPort(int *Port);
        /*!
         * @brief setSpeed() is the setter function for the Speed
         */
        void setSpeed(int speed);
        /*!
         * @brief setAngle() is the setter function for the Angle
         */
        void setAngle(int angle);
        /*!
         * @brief setLight() is the setter function for the Light
         */
        void setLight(bool light);
        /*!
         * @brief getSpeed() is the getter function for the Speed
         */
        void getSpeed(int &speed);
        /*!
         * @brief getAngle() is the getter function for the Angle
         */
        void getAngle(int &angle);
        /*!
         * @brief getLight() is the getter function for the Light
         */
        void getLight(bool &light);
        /*!
         * @brief sendThread() is executed in an extra thread. It handles all data that are sent over the socket
         */
        void sendThread();
        /*!
         * @brief recThread() is executed in an extra thread. It handles all data that are received over the socket
         */
        void recThread();
    private:
        /// Joystick object for getting all data  of the gamepad
        Joystick *_joystick;
        /// Timer for polling gamepad. (Polling function is called when timer is exceeded)
        QTimer *GamepadTimer;
        /// user interface object for setting data in the gui
        Ui::Garfield_control *ui;
        /// Position of GridPoint
        QPoint GridPointPos;
        /// debug variable. If true, debug slider are visible, hidden else
        bool debug = false;

        /// ClientComm for communicating with garfield over the socket
        Alf_Communication<Client> ClientComm;

        /// Cyclic timer for updating the acceleration map
        QTimer *UpdateAccTimer;

        /// object of class settings for the settings window
        Settings *settings;

        /// Garfield.conf settings file string
        QString m_sSettingsFile;

        /// IP to connect with
        QString _IP;
        /// Port on which the socket is created
        QString _Port;
        /// Device name of the gamepad (e.g.: "js0")
        QString _Dev;

        /// variable holding the speed which is sent over socket
        uint8_t _speed;
        /// direction which is sent over socket (0: forward, 1: backwards)
        int _direction;
        /// steering angle (-90-90°)
        int _angle;
        /// true: light is on, false: else
        bool _light = false;
        /// acceleration of the car which is received on the socket
        double _acceleration;
        /// transverse acceleration of the car which is received on the socket
        double _lateral_acceleration;

        /// true: connected with socket, false: else
        bool _connected;
        /// true: thread should close socket
        bool _disconnect;

    private slots:
        /*!
         * @brief poll_gamepad() slot which is cyclic called for polling the gamepad actions
         */
        void poll_gamepad();

        /*!
         * @brief update_acceleration() slot which is cyclic called for updating the acceleration grid
         */
        void update_acceleration();

        /*!
         * @brief command_forward() slot which is called when forward button or W key is pressed
         */
        void command_forward();
        /*!
        * @brief command_back() slot which is called when backward button or S key is pressed
        */
        void command_back();
        /*!
        * @brief command_forback_rel() slot which is called when forward or backward button or W or S keys are released
        */
        void command_forback_rel();
        /*!
        * @brief command_left() slot which is called when left button or A key is pressed
        */
        void command_left();
        /*!
        * @brief command_right() slot which is called when right button or D key is pressed
        */
        void command_right();
        /*!
        * @brief command_leftright_rel() slot which is called when left or right button or A or D keys are released
        */
        void command_leftright_rel();
        /*!
        * @brief command_setForwardSpeed(double value) slot which is called when gamepad axis for driving forward is moved
        * @param[in] value - The value of the gamepad axis
        */
        void command_setForwardSpeed(double value);
        /*!
        * @brief command_setBackSpeed(double value) slot which is called when gamepad axis for driving backwards is moved
        * @param[in] value - The value of the gamepad axis
        */
        void command_setBackSpeed(double value);
        /*!
        * @brief command_setDirection() slot which is called when gamepad axis for steering is moved
        * @param[in] value - The value of the gamepad axis
        */
        void command_setDirection(double value);
        /*!
        * @brief command_toggleLights(int state) slot which is called when light checkbox or gamepad button is pressed
        * @param[in] state - The state of the button
        */
        void command_toggleLights(int state);
        /*!
        * @brief debug_settings(bool state) slot which is called when Debug Checkbox state changed
        * @param[in] state - The state of the checkbox
        */
        void debug_settings(bool state);
        /*!
        * @brief open_settings() slot which is called when menu item for settings window is clicked
        */
        void open_settings();
        /*!
        * @brief closed_settings() slot which is called when settings window is closed
        */
        void closed_settings();
        /*!
        * @brief open_close_connection() slot which is called when connect/disconnect button is pressed
        */
        void open_close_connection();
};

#endif //GARFIELD_CONTROL_H
