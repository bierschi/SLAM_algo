/*!
 * @file Steering.hpp
 */
#ifndef STEERING
#define STEERING

//maximum possible steering angle in one direction set by the init  function
#define MAX_STEERING_ANGLE 60

//Value at wich servo takes neutral position
#define NEUTRAL_POS_VALUE 51

/*!
 *class Steering for controlling the steering servo. No object is needed because of static functions
 */
class Steering {
public:
    /*!
     * Delete the default constructor
     */
    Steering() = delete;
	/*!
	* Init Function for initializing the Steering with the maximum steering angle
	* @param max_angle: This is the maximum steering angle in one direction (e.g. 50 deg). If the Set(alt_8 angle) is called with a bigger angle, it is set to max_angle_delta
	*/
	static void Init(alt_u8 max_angle);
	/*!
	* Set Function for setting given angle to the servo
	* @param angle: This is the angle to set the servo (between -max_angle_delta and max_angle_delta)
	*/
	static void Set(alt_8 angle);
private:
	static alt_u8 max_angle_delta; /*! Stores the maximum angle of the servo to prevent it from to wide angles */
	static float val_per_deg;
	/*! the highest possible/allowed value that comes from the HQ/ARM system, needed for rescaling the steering values if an lower max steering angle is set */
	static const alt_u8 max_steering_value = 90;
};

#endif
