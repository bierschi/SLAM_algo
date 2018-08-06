/*!
 * @file
 */

#include "alf_data.hpp" 

/*
 * static variables
 */

float Alf_Data::urg_angle_min = 0.0;
float Alf_Data::urg_angle_max = 0.0;
float Alf_Data::urg_angle_increment = 0;
int Alf_Data::urg_time_increment = 100;
uint32_t Alf_Data::urg_range_min = 0;
uint32_t Alf_Data::urg_range_max = 0;

/*
 * global variables
 */


/*
 * functions
 */

bool Alf_Data::Init_Data(float an_min, float an_max, float an_inc, int32_t ran_min, int32_t ran_max, int32_t time){
	Alf_Data::urg_angle_increment = an_inc;
	Alf_Data::urg_angle_min = an_min;
	Alf_Data::urg_angle_max = an_max;
	Alf_Data::urg_range_min = ran_min;
	Alf_Data::urg_range_max = ran_max;
	Alf_Data::urg_time_increment = time;
	return true;
}

Alf_Urg_Measurements_Buffer::Alf_Urg_Measurements_Buffer(uint32_t size){
	_max_size = size;
}

alf_error Alf_Urg_Measurements_Buffer::push(const Alf_Urg_Measurement& a){
	alf_error err;
	if(_buffer.size() < _max_size){
		_buffer.push(a);
		err = ALF_NO_ERROR;
	}
	else{
		err = ALF_BUFFER_IS_FULL;
	}
	return err;
}

alf_error Alf_Urg_Measurements_Buffer::pop(Alf_Urg_Measurement* a){
	alf_error err;
	if(_buffer.empty()){
		err = ALF_NOTHING_IN_BUFFER;
	}
	else{
		*a = _buffer.front();
		_buffer.pop();
		err = ALF_NO_ERROR;
	}
	return err;
}

uint32_t Alf_Urg_Measurements_Buffer::size() const {
	return _buffer.size();
}

uint32_t Alf_Urg_Measurements_Buffer::getMaxSize(void) const {
	return _max_size;
}
