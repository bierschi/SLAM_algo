/*!
 * @file
 *
 * @brief contains the main application for wrapping data which are collected with the alf_urg application and sended
 * to this client
 * @attention can only be build within a working ROS environment
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>

#include "melmac.hpp"

#include "alf_erno.h"
#include "alf_data.hpp"
#include "alf_log.hpp"
#include "alf_communication.hpp"

#define BUF_SIZE 1322 /*!< This defines the size of AlfMeasBuffer*/

#define LIDAR_FREQ 10 /*!< The frequence of the Lidar. It is needed for the ros loop and scan_time*/

#define ANGLE_INC 0.006136 /*!< Better working ANGLE_INC which works better than the commented calculation*/
#define TIME_INC 0.000098 /*!< Better working TIME_INC which works better than the commented calculation*/

static Alf_Urg_Measurements_Buffer AlfMeasBuffer(BUF_SIZE); /*!< The Buffer in which all data gets stored from the "readThread" The size if defined in BUF_SIZE*/
static Alf_Urg_Measurement AlfMeas; /*!< The Alf_Urg_Measurement object with one measurement from the lidar */
static Alf_Data AlfData; /*!< Object of Alf_Data with static variables for the lidar */
static Alf_Communication<std::fstream> FileComm; /*!< Alf_Communication file-communication object. Its only necessaray for debug*/
static Alf_Communication<Client> ClientComm; /*!< Alf_Communication Client-communication object. This creates the communication between client and server */
static std::mutex queueAccess; /*!< A Mutex lock for threadsafe queue access */

static const std::string host = "10.0.0.1"; /*!< The address of the host, which sends all lidar data (here: Raspberry Pi with 10.0.0.1) */
static const uint32_t portNumber = 6666; /*!< The port on which all data are sent */

void rvizWrapper(ros::NodeHandle* n, ros::Publisher* scan_pub, tf::TransformBroadcaster* broadcaster, ros::Rate* r)
{
	while (n->ok()) {

		std::unique_lock<std::mutex> lock(queueAccess);
		alf_error res = AlfMeasBuffer.pop(&AlfMeas);
		lock.unlock();

		if( res == ALF_NOTHING_IN_BUFFER )
			continue; //If buffer is empty, do not try to pop an Alf_Measurement Object

		lock.unlock();

		ros::Time scan_time = ros::Time::now(); //Use current time for sending the data

		//Broadcast tf message
		broadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0)), scan_time, "alf", "laser"));


		//Data for the LaserScan message
		sensor_msgs::LaserScan scan;
		scan.header.seq = AlfMeas.sequence_number;
		scan.header.stamp.sec = scan_time.sec;
		scan.header.stamp.nsec = scan_time.nsec;
		scan.header.frame_id = "alf";
		scan.angle_min = AlfData.urg_angle_min;
		scan.angle_max = AlfData.urg_angle_max;
		scan.scan_time = (float)1/LIDAR_FREQ;

		//scan.angle_increment =  3.14/AlfMeas.elements_in_array; // This is the normal calculation for angle_increment. Hardcoded define works better
		scan.angle_increment = ANGLE_INC;


		//scan.time_increment = (float)(1.0/10.0)/(float)myMeas.elements_in_array; //This is the normal calculation for time_increment. Hardcoded define works better
		scan.time_increment = TIME_INC;

		scan.range_min = (float)AlfData.urg_range_min/1000; //Convert urg_range_min from mm to m
		scan.range_max = (float)AlfData.urg_range_max/1000; //Convert urg_range_max from mm to m

		scan.ranges.resize(AlfMeas.elements_in_array); //Resize Alf_Measurement array to ros array, if the size does not fit
		for (unsigned int i = 0; i < AlfMeas.elements_in_array; i++) {
			scan.ranges[i] = ((float)AlfMeas.measurement_points[i])/1000; //Convert measurement_point from m to mm and store it in ros array
		}

		scan_pub->publish(scan); //Publish scan

		r->sleep(); //Sleep till the frequence of 10 Hz is reached
	}
}


void readStreamingData(void)
{
	alf_mess_types currMsgType = ALF_END_ID;
	while(1){
		std::unique_lock<std::mutex> lock(queueAccess);
		uint8_t res = ClientComm.Read(AlfMeasBuffer, currMsgType);
		lock.unlock();
		if( res == ALF_NO_ERROR  )
		{
			//everything fine
			if(currMsgType == ALF_MEASUREMENT_DATA_ID)
			{
				//normal data message was read
			}
			else if (currMsgType == ALF_INIT_ID)
			{
				//init message was read
			}
			else
			{
				//end message was read
				break;
			}
		}
		else if ( res == ALF_BUFFER_IS_FULL || res == ALF_IO_ERROR)
		{
			//buffer is too small or connection was bad
		}
	}

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "rviz_wrapper");

	ros::NodeHandle n;

	//Create Publisher with name "scan" and maximum number of 10 queued messages
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);

	tf::TransformBroadcaster broadcaster;

	ros::Rate r(LIDAR_FREQ); //Start loop with 10Hz


	Alf_Log::alf_log_init("Melmac.log", log_debug, true);


	//FileComm.Init("/home/hsp/catkin_ws/src/rviz_wrapper/src/data_neu.alf");
	//FileComm.Init("/home/hsp/catkin_ws/fipsi_zimmer.alf");

	int retVal ;

	//retVal = FileComm.Read(mybuffer, 1322);

	if( ClientComm.Init(host, portNumber) == true)
	{
		std::thread dataThread(readStreamingData);
		std::thread sendThread(rvizWrapper, &n, &scan_pub, &broadcaster, &r);
		dataThread.join();
		sendThread.join();
		//end message was read so now the communication can be closed
		ClientComm.EndCommunication();
	}
	else
	{
		Alf_Log::alf_log_write("The socket-connection could not be established...", log_error);
	}


	Alf_Log::alf_log_write("Ending the application", log_info);
	Alf_Log::alf_log_end();
}
