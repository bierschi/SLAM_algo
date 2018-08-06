/*!
 * @file
 * @brief contains the implementations for template functions to be outside of the hpp
 */
template<class _comType>
uint8_t Alf_Communication<_comType>::readLine(std::fstream& comtype,
		string& s) {
	uint8_t ret_val = 0;
	getline(comtype, s, __end_delim);
	if (!comtype.fail())
		ret_val = 1;
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::__writeLine(std::fstream& comtype,
		string &s) {
	alf_error ret_val = ALF_UNKNOWN_ERROR;
	comtype << s;
	if (comtype.good())
		ret_val = ALF_NO_ERROR;
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Init(const string& filename) {
	bool ret_val = false;
	if (!__comHandler.is_open()) {
		__comHandler.open(filename,
				std::fstream::in | std::fstream::out | std::fstream::app);
	}
	if (__comHandler.good()) {
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Init(const string& server, const uint32_t& portno)
{
	bool ret_val = false;
	if(__comHandler.startConnection(portno, server) == 1)
	{
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Init(const uint32_t& portno)
{
	bool ret_val = false;
	if(__comHandler.startConnection(portno) == ALF_NO_ERROR){
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Write(std::fstream& file, const char* data, const uint32_t& len) {
	bool ret_val = false;
	if (file.good() && file.is_open()) {
		file.write(data, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Write(Client& cl, const char* data, const uint32_t& len) {
	bool ret_val = false;
	if (cl.is_open()) {
		write(cl.getSocketNumber(), data, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Write(Server& ser, const char* data, const uint32_t& len) {
	bool ret_val = false;
	if (ser.is_open()) {
		write(ser.getSocketNumber(), data, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::Write(Alf_Urg_Measurements_Buffer &buffer) {
	alf_error ret_val = ALF_NO_ERROR;
	Alf_Urg_Measurement m;
	while(buffer.size() > 0 and ret_val == ALF_NO_ERROR){
		ret_val = buffer.pop(&m);
		if(ret_val == ALF_NO_ERROR){
			ret_val = Write(m);
		}
	}
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::Write(Alf_Urg_Measurement &meas){
	alf_error ret_val = ALF_CANNOT_SEND_MESSAGE;
	if(__comHandler.good() and __comHandler.is_open()){
		string to_send;
		to_send = std::to_string((uint8_t)ALF_MEASUREMENT_DATA_ID) + __delim;
		to_send += std::to_string(meas.sequence_number) + __delim + std::to_string(meas.time_stamp) + __delim;
		for(uint32_t i = 0; i < meas.elements_in_array; i++){
			to_send += std::to_string(meas.measurement_points[i]) + __delim;
		}
		to_send += std::to_string(meas.first_valid_index) + __delim + std::to_string(meas.last_valid_index);
		to_send += __end_delim;

		ret_val = ALF_NO_ERROR;
        ret_val = __writeLine(__comHandler, to_send);
	}
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::Write(Alf_Drive_Command &command) {
    alf_error ret_val = ALF_CANNOT_SEND_MESSAGE;
    if(__comHandler.good() and __comHandler.is_open()){
        string to_send;
        to_send = std::to_string((uint8_t) ALF_DRIVE_COMMAND_ID) + __delim;
        to_send += std::to_string(command.speed) + __delim;
        to_send += std::to_string(command.direction) + __delim;
        to_send += std::to_string(command.angle) + __delim;
        to_send += std::to_string(command.light) + __end_delim;
        ret_val = ALF_NO_ERROR;
        ret_val = __writeLine(__comHandler, to_send);
    }
    return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::Write(Alf_Drive_Info &info) {
    alf_error ret_val = ALF_CANNOT_SEND_MESSAGE;
    if(__comHandler.good() and __comHandler.is_open()){
        string to_send;
        to_send = std::to_string((uint8_t) ALF_DRIVE_INFO_ID) + __delim;
        to_send += std::to_string(info.speed) + __delim;
        to_send += std::to_string(info.acceleration) + __delim;
        to_send += std::to_string(info.lateral_acceleration) + __delim;
        to_send += std::to_string(info.z_acceleration) + __delim;
        to_send += std::to_string(info.Gyroscope_X) + __delim;
        to_send += std::to_string(info.Gyroscope_Y) + __delim;
        to_send += std::to_string(info.Gyroscope_Z) + __delim;
        to_send += std::to_string(info.temperature) + __end_delim;
        ret_val = ALF_NO_ERROR;
        ret_val = __writeLine(__comHandler, to_send);
    }
    return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Read(std::fstream& file, char* readPtr, const uint32_t& len) {
	bool ret_val = false;
	if (file.good() && file.is_open())
	{
		file.seekg(0);
		file.read(readPtr, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Read(Client& cl, char* readPtr, const uint32_t& len) {
	bool ret_val = false;
	if (cl.is_open())
	{
		read(cl.getSocketNumber(), readPtr, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::Read(Server& ser, char* readPtr, const uint32_t& len) {
	bool ret_val = false;
	if (ser.is_open())
	{
		read(ser.getSocketNumber(), readPtr, len);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::Read(Alf_Urg_Measurements_Buffer& readBuffer, alf_mess_types& msgType_out, const uint32_t& nrPackToRead)
{
	alf_error ret_val = ALF_UNKNOWN_ERROR;
	if( (__comHandler.is_open() == false) || (__comHandler.good() == false) )
	{
		//Server/Client/File not open
		ret_val = ALF_IO_ERROR;
	}
	else if((readBuffer.size() + nrPackToRead) > readBuffer.getMaxSize())
	{
		//the buffer is too small to read all values -> do not read any values
		//todo: @tobi thinking about it. If the buffer is full and there are messages which are not in context with the buffer, why stop reading this message?
		ret_val = ALF_BUFFER_IS_FULL;
	}
	else
	{
		//Everything is fine, go on
		//get the minimum of maxsize-size or nrPacketToRead = actual number of packets to read
		const std::regex expr("\\-?[0-9]*\\.?[0-9]+");
		alf_mess_types msgType = ALF_MEASUREMENT_DATA_ID;
		for(uint32_t i = (nrPackToRead > (readBuffer.getMaxSize() - readBuffer.size()) ? (readBuffer.getMaxSize() - readBuffer.size()) : nrPackToRead); i > 0; i--)
		{
			std::string _line;
			if ( readLine(__comHandler, _line) == 1 )
			{ // readline was successful means valid message was found
				Alf_Urg_Measurement hlp{};
				std::smatch match{};
				msgType = ALF_MEASUREMENT_DATA_ID;
				uint16_t idx = 0;
				uint16_t measPointIdx = 0;

				while ( std::regex_search(_line, match, expr) )
				{
					if(msgType == ALF_INIT_ID)
					{
						switch(idx++)
						{
							case 1:
								Alf_Data::urg_angle_min = std::stof(match.str());
								break;
							case 2:
								Alf_Data::urg_angle_max = std::stof(match.str());
								break;
							case 3:
								Alf_Data::urg_angle_increment = std::stof(match.str());
								break;
							case 4:
								Alf_Data::urg_time_increment = std::stoi(match.str());
								break;
							case 5:
								Alf_Data::urg_range_min = std::stoi(match.str());
								break;
							case 6:
								Alf_Data::urg_range_max = std::stoi(match.str());
								break;
						}
					}
					else if(msgType == ALF_END_ID)
					{
						break;
					}
                    else if(msgType == ALF_DRIVE_INFO_ID)
                    {
                        switch(idx++)
                        {
                            case 1:
                                global_drive_info.speed = std::stoi(match.str());
                                break;
                            case 2:
                                global_drive_info.acceleration = std::stoi(match.str());
                                break;
                            case 3:
                                global_drive_info.lateral_acceleration = std::stoi(match.str());
                                break;
                            case 4:
                                global_drive_info.z_acceleration = std::stoi(match.str());
                                break;
                            case 5:
                                global_drive_info.Gyroscope_X = std::stoi(match.str());
                                break;
                            case 6:
                                global_drive_info.Gyroscope_Y = std::stoi(match.str());
                                break;
                            case 7:
                                global_drive_info.Gyroscope_Z = std::stoi(match.str());
                                break;
                            case 8:
                                global_drive_info.temperature = std::stof(match.str());
                                break;
                        }
                    }
                    else if(msgType == ALF_DRIVE_COMMAND_ID)
                    {
                        switch(idx++)
                        {
                            case 1:
                                global_drive_command.speed = std::stoi(match.str());
                                break;
                            case 2:
                                global_drive_command.direction = std::stoi(match.str());
                                break;
                            case 3:
                                global_drive_command.angle = std::stoi(match.str());
                                break;
                            case 4:
                                global_drive_command.light = std::stoi(match.str());
                            break;
                        }
                    }
					else
					{
						switch(idx++)
						{
							// no boundary check for the msgType is given at the moment
							case 0:
								msgType = static_cast<alf_mess_types>(std::stoi(match.str()));
								break;
							case 1:
								hlp.sequence_number = std::stoi(match.str());
								break;
							case 2:
								hlp.time_stamp = std::stol(match.str());
								break;
							case Alf_Urg_Measurement::elements_in_array + 4:
								hlp.first_valid_index = std::stoi(match.str());
								break;
							case Alf_Urg_Measurement::elements_in_array + 5:
								hlp.last_valid_index = std::stoi(match.str());
								break;
							default:
								hlp.measurement_points[measPointIdx++] = std::stol(match.str());
						}
					}
					_line = match.suffix().str();
				}
				msgType_out = msgType;
				if( msgType == ALF_MEASUREMENT_DATA_ID )
				{
					ret_val = readBuffer.push(hlp);
				}
				else
				{
					ret_val = ALF_NO_ERROR;
				}
			}
		}
	}
	return ret_val;
}

template<class _comType>
bool Alf_Communication<_comType>::EndCommunication(void)
{
	bool ret_val = false;
	string to_send = std::to_string((uint8_t)ALF_END_ID) + __delim + std::to_string((uint8_t)1) + __end_delim;
	if (__comHandler.is_open() and __comHandler.good())
	{
		__writeLine(__comHandler, to_send);
		closeCom(__comHandler);
		ret_val = true;
	}
	return ret_val;
}

template<class _comType>
alf_error Alf_Communication<_comType>::WriteInitMessage(){
	alf_error ret_val = ALF_CANNOT_SEND_MESSAGE;
	if(__comHandler.good() and __comHandler.is_open()){
		string to_send;
		to_send = std::to_string((uint8_t)ALF_INIT_ID) + __delim;
		to_send += std::to_string(Alf_Data::urg_angle_min) + __delim + std::to_string(Alf_Data::urg_angle_max) + __delim;
		to_send += std::to_string(Alf_Data::urg_angle_increment) + __delim + std::to_string(Alf_Data::urg_time_increment) + __delim;
		to_send += std::to_string(Alf_Data::urg_range_min) + __delim + std::to_string(Alf_Data::urg_range_max);
		to_send += __end_delim;

		ret_val = ALF_NO_ERROR;
		ret_val = __writeLine(__comHandler, to_send);
	}
	return ret_val;
}
