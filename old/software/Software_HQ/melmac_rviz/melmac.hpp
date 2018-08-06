/*!
 *@file
 *
 *
 *All global variables, defines and the two functions which represents the threads are declared here
 */

#ifndef SRC_MELMAC_HPP_
#define SRC_MELMAC_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "alf_data.hpp"
#include "alf_communication.hpp"

/*!
 * @brief This function represents the sendThread
 *
 * It takes all data from Alf Measurement Buffer and maps the data to the ros data structure
 * @param[in] n is the nodehandler which checks the status
 * @param[in] scan_pub is the Scan Publisher which sends all data to rviz
 * @param[in] broadcaster is the broadcaster to send tf messages to rviz
 * @param[in] r is necessary for creating a ros loop with the frequence of the lidar (here: 10 Hz)
 * @return void
 */
void rvizWrapper(ros::NodeHandle* n, ros::Publisher* scan_pub, tf::TransformBroadcaster* broadcaster, ros::Rate* r);

/*!
 * @brief function for reading the measurement data from the socket connection. If and end message was read the function returns and the user can end or reopen the communication
 * @param[in] -
 * @return -
 */
void readStreamingData(void);

/*!
 * @brief Main function of rviz_wrapper
 *
 * It opens the socket communication, starts the two threads (readThread and sendThread) etc.
 */
int main(int argc, char** argv);

#endif /* SRC_MELMAC_HPP_ */
