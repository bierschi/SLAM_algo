/*!
 * @file
 * @brief a library for collect all classes which represents any physical data
 */
#ifndef ALF_DATA
#define ALF_DATA


#include <queue>
#include <string>

#include "alf_erno.h"

/*!
 * @brief the number of elements the measurement buffer can store by default.
 */
#define MAX_SIZE_OF_MEASUREMENT_BUFFER_DEFAULT 10

/*!
 * @brief number of the measurements the urg_sensors made. These number varies from sensor to sensor, so with another sensor this
 * value must be adjusted
 */
#define URG_NUMBER_OF_MEASUREMENT_DATA 768

/*!
 * @brief contains all the data about the vehicle which could be exchanges between the vehicle and other applications
 * so serves as interface between a controller and the hardware
 */
class Alf_Data{
public:
	/// the min angle which the urg laser scanner can provide
	static float urg_angle_min;
	/// the max angle which the urg laser scanner can provide
	static float urg_angle_max;
	/// the increment between two measurments of the laser scanner
	static float urg_angle_increment;
	/// the time between two measurements of the laser scanner in ms, with our laser scanner this is 100ms
	static int urg_time_increment;
	/// the minimal distance the laser scanner can measure
	static uint32_t urg_range_min;
	/// the maximal distance the laser scanner can measure
	static uint32_t urg_range_max;

	/*!
	 * @brief initialise the Alf_Data
	 */
	static bool Init_Data(float, float, float, int32_t, int32_t, int32_t);
};

/*!
 * @brief This class stands for <b>one</b> whole measurement of the laser scanner and provides additional informations
 * 		  It contains all measurement values, also this one, which are invalid in case of the datasheet
 */
class Alf_Urg_Measurement {
public:
	/// how much measurement points do we have for one measurement
	static constexpr uint32_t elements_in_array = URG_NUMBER_OF_MEASUREMENT_DATA + 1;
	/// The storage for the measurement points. Each index represents one urg_angle_increment
	long int measurement_points[elements_in_array];
	/// The first index of the measurement_points which should be used (derived from the data sheet)
	uint32_t first_valid_index;
	/// The last index of the measurement_points which should be used
	uint32_t last_valid_index;
	/// To provide a chronological sequence of the various measurements
	uint32_t sequence_number;
	/// The timestamp of the measurement. Its no absolut time, just the internal counter, so several measurements can be set in an chronologically relation.
	long int time_stamp;
};

/*!
 * @brief This buffer can store a set of Alf_Urg_Measurement . It use the std::queue for storing the data and have a maximum size to determine the maximum
 * 		  RAM size which can be used.
 */
class Alf_Urg_Measurements_Buffer{
private:
	/// the max size of the buffer, when this size is reached no more elements can be stored in the buffer
	uint32_t _max_size;
	/// the underlying queue of the buffer
	std::queue<Alf_Urg_Measurement> _buffer;

public:
	/*!
	 * @brief constructor for the Alf_Urg_Measurement_Buffer
	 *  	  set _max_size to the given value or default to the macro MAX_SIZE_OF_MEASUREMENT_BUFFER_DEFAULT
	 * @param[in] size - the size, default MAX_SIZE_OF_MEASUREMENT_BUFFER_DEFAULT
	 * @return -
	 */
	Alf_Urg_Measurements_Buffer(uint32_t size = MAX_SIZE_OF_MEASUREMENT_BUFFER_DEFAULT);
	/*!
	 * @brief append one Alf_Urg_Measurement to the buffer
	 * @param[in] a - the measurement
	 * @return
	 * 			- ALF_NO_ERROR if the element can be appended to the queue
	 * 			- ALF_BUFFER_IS_FULL if the queue is full and cannot store any additonal elements
	 */
	alf_error push(const Alf_Urg_Measurement&);
	/*!
	 * @brief pops one element of the buffer and stores it in the memory given by a pointer
	 * @param[inout] a - the memory where the Alf_Urg_Measurement shall be stored
	 * @return
	 * 			- ALF_NO_ERROR if everything works
	 * 			- ALF_NOTHING_IN_BUFFER if there is no more element in the queue which could be removed
	 */
	alf_error pop(Alf_Urg_Measurement*);
	/*!
	 * @brief returns the actual size of the queue (so how much elements are stored within)
	 * @return the number of elements
	 */
	uint32_t size() const;

	/*!
	 * @brief returns the maximal number of elements which could be stored
	 * @return the maximal number of elements
	 */
	uint32_t getMaxSize(void) const;
};


#endif 
