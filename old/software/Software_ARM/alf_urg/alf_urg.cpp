/*!
 * @file
 * @brief contains the main application to collect measurements from the URG Lidar and offer the collected data in a
 * properitary format other applications
 */

#include "alf_urg.hpp"

using std::cout;
using std::endl;

#define COMMSERVICE Server
#define COMMFILE 6666
#define msleep(a) usleep(a*1000)

/// the buffer with the Size of 100 for all measurements
Alf_Urg_Measurements_Buffer Alf_Measurements_Buffer(100);
/// mutex to lock the Alf_Measurements_Buffer
std::mutex Alf_Measurements_Buffer_Mutex;
/// struct for the ONE connected sensor
urg_t urg_sensor;

/// control variable for the thread which collects the measurements
bool Run_Measurement_Task;
/// control variable for the thread which handles the communication
bool Run_Server_Task;

/// variable to let sleep the main thread
std::condition_variable Run_Main_Task_cond;
/// mutex to for the main thread
std::mutex Run_Main_Task_mut;

/// the communication which shall be handled
Alf_Communication<COMMSERVICE> server_communication;

/*!
 * @brief function for collecting data from a urg_sensor and pushing them into a the Alf_Measurements_Buffer
 * @attention needs a initialized and running urg_sensor, given by the global variable urg_sensor
 * @note normally executed as a standalone thread/task
 */
inline void GetMeasurements() {
	ALF_LOG_WRITE("Start Task for measurements", log_debug);
	uint32_t sequence_number = 0;
	int urg_error;
	alf_error error_code;
	while (Run_Measurement_Task) {
		Alf_Urg_Measurement m;
		urg_error = urg_get_distance(&urg_sensor, m.measurement_points, &m.time_stamp);
		m.first_valid_index = urg_sensor.first_data_index;
		m.last_valid_index = urg_sensor.last_data_index;
		m.sequence_number = sequence_number++;
		if (urg_error < 0) { //an error happens while collecting the urg laser scan data
			ALF_LOG_WRITE("The urg sensor returns the error code " + std::to_string(urg_error) + " on sequence number " + std::to_string(sequence_number) , log_warning);
			sequence_number = 0;
			if(urg_error == URG_CHECKSUM_ERROR){
				ALF_LOG_WRITE("There was a checkum error. Don't using this measurement!", log_warning);
			}
			else if(urg_error == URG_NO_RESPONSE){
				ALF_LOG_WRITE("The urg sensor doesn't answer. Trying to reconnect...", log_warning);
				msleep(200);
				std::stringstream error_stringstream;
				uint8_t i = 0;
				for(; i < 5; i++){ //trying 5 times to reconnect to the sensor
					if (urg_open(&urg_sensor, Alf_Urg_Sensor::alf_urg_connection_type,((char*) std::string(Alf_Urg_Sensor::alf_urg_device_port).c_str()), Alf_Urg_Sensor::alf_urg_baudrate) < 0) {
						error_stringstream << "Cannot open Sensor on Port " << Alf_Urg_Sensor::alf_urg_device_port << " with baudrate " << Alf_Urg_Sensor::alf_urg_baudrate << ". Abort..." << endl;
						ALF_LOG_WRITE(error_stringstream.str(), log_error);
					}
					else{
						urg_start_measurement(&urg_sensor, URG_DISTANCE, URG_SCAN_INFINITY, 0);
						break;
					}
				}
				if(i >= 4){
					ALF_LOG_WRITE("Cannot reconnect to the sensor. Abort...", log_error);
				}
			}
			else{
				Run_Measurement_Task = false;
				ALF_LOG_WRITE("Must stop the application", log_error);
				Run_Main_Task_cond.notify_all();
			}
		}
		else{ // the data could be readed
			Alf_Measurements_Buffer_Mutex.lock();
			error_code = Alf_Measurements_Buffer.push(m);
			Alf_Measurements_Buffer_Mutex.unlock();
			if(error_code == ALF_BUFFER_IS_FULL){
				ALF_LOG_WRITE("The buffer for the urg measurement data is full! There are lost measurements!", log_warning);
			}
		}
	}
}

/*!
 * @brief function for sending collected measurement data over the socket connection
 * @attention the server connection should be established before calling
 * @note normally executed as an own thread
 */
inline void ServerConnection() {
	ALF_LOG_WRITE("Start task for the connection", log_debug);
	while (Run_Server_Task) {
		Alf_Urg_Measurement m;
		Alf_Measurements_Buffer_Mutex.lock();
		alf_error ret_val = Alf_Measurements_Buffer.pop(&m);
		Alf_Measurements_Buffer_Mutex.unlock();
		if(ret_val == ALF_NOTHING_IN_BUFFER){
			msleep(100);
		}
		else if(ret_val == ALF_NO_ERROR){
			server_communication.Write(m);
		}

	}
}

/*!
 * @brief dummy function which wake up the main thread from "sleep". This is needed for a clean stop of the programm
 * 		  with a SIGINT of the OS (typical CTRL+C)
 * @param[in] sig - SIGINT
 * @return -
 */
void Stop_Program(int sig) {
	Run_Main_Task_cond.notify_all();
}

/*!
 * @brief the main process of this application
 * this does
 * - initializing the urg_sensor
 * - initializing the server connection
 * - starting the two threads
 * - ending the application in a clean way (after CTRL+C)
 */
int main() {
	/*
	 * Variables
	 */
	std::stringstream error_stringstream;
	std::unique_lock<std::mutex> lck(Run_Main_Task_mut);

	/*
	 * init
	 */
	ALF_LOG_INIT("Alf_urg.log", log_debug, true);
	signal(SIGINT, Stop_Program);

	//initialize the urg_sensor
	ALF_LOG_WRITE("Initializing the urg sensor...", log_info);
	if (urg_open(&urg_sensor, Alf_Urg_Sensor::alf_urg_connection_type,((char*) std::string(Alf_Urg_Sensor::alf_urg_device_port).c_str()), Alf_Urg_Sensor::alf_urg_baudrate) < 0) {
		error_stringstream << "Cannot open Sensor on Port " << Alf_Urg_Sensor::alf_urg_device_port << " with baudrate " << Alf_Urg_Sensor::alf_urg_baudrate << ". Abort..." << endl;
		ALF_LOG_WRITE(error_stringstream.str(), log_error);
	}
	else{
		//working code
		ALF_LOG_WRITE("Urg sensor works!Initializing communication on port/file " + std::to_string(COMMFILE) + "...", log_info);
		if(server_communication.Init(COMMFILE)){ //waiting until the communication connection is established
			ALF_LOG_WRITE("All good! Starting with the application...", log_info);
			Alf_Data::Init_Data(urg_index2deg(&urg_sensor, 0), urg_index2deg(&urg_sensor, Alf_Urg_Measurement::elements_in_array), urg_index2deg(&urg_sensor, 1) - urg_index2deg(&urg_sensor, 0), urg_sensor.min_distance, urg_sensor.max_distance, urg_sensor.scan_usec / 1000);
			server_communication.WriteInitMessage();
			urg_start_measurement(&urg_sensor, URG_DISTANCE, URG_SCAN_INFINITY, 0);
			Run_Measurement_Task = true;
			Run_Server_Task = true;
			std::thread measurements (GetMeasurements);
			std::thread server (ServerConnection);

			Run_Main_Task_cond.wait(lck); //waiting until this main thread should continue

			ALF_LOG_WRITE("Stopping the application!", log_info);
			Run_Measurement_Task = false;
			Run_Server_Task = false;
			measurements.join();
			server.join();
			server_communication.EndCommunication();
		}
		else{
			ALF_LOG_WRITE("Cannot open the given file/port. Abort...", log_error);
		}
	}
	/*
	 * the end
	 */
	ALF_LOG_WRITE("Ending the application", log_info);
	urg_close(&urg_sensor);
	ALF_LOG_END();
	return 0;
}
