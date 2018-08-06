/*!
 * @file
 */
#include "alf_log.hpp"
#include <fstream>
#include <ctime>
#include <iostream>
/*** for C string handling ***/
#include <string.h>
#include <string>

using std::cout;
using std::endl;
using std::string;

std::ofstream Alf_Log::__logfile{};
alf_log_level_e Alf_Log::__loglevel = log_debug;
bool Alf_Log::__con_out = false;
const string Alf_Log::msges[] =
{
	"[ERROR:]",
	"[WARNING:]",
	"[INFO:]",
	"[DEBUG:]"
};


bool Alf_Log::alf_log_init(const string& filename, const alf_log_level_e& log_level, const bool& console_output)
{
	bool ret_val = false;

	if(!__logfile.is_open())
	{
		__logfile.open(filename, std::ofstream::app);
	}
	if(__logfile.good())
	{
		__con_out = console_output;
		time_t result = std::time(nullptr);
		const char* log_timestamp = std::asctime(std::localtime(&result));
		string log_entry(log_timestamp, strlen(log_timestamp));

		log_entry.insert(0,"Start with logging at ");
		__logfile << "\n" << log_entry << "\n\n";

		__loglevel = log_level;
		ret_val = true;
	}
	return ret_val;
}

bool Alf_Log::alf_log_end(void)
{
	bool ret_val = false;
	if(__logfile.is_open() && __logfile.good())
	{
		__logfile.close();
		ret_val = true;
	}
	return ret_val;
}

bool Alf_Log::alf_log_write(const string& log_entry, const alf_log_level_e& log_level)
{
	bool ret_val = false;

	if(__con_out == true)
	{
		string con_out = "\033[1;";		//"\033[1; + color +\033[0m"
		switch(log_level)
		{
			case log_error: con_out += std::to_string(error); break;
			case log_warning: con_out += std::to_string(warning); break;
			case log_info: con_out += std::to_string(info); break;
			default: con_out += std::to_string(debug);
		}

		con_out += ("m" + msges[log_level] + log_entry + "\033[0m");
		cout << con_out << "\n";
	}
	if(__loglevel >= log_level)
	{
		if(__logfile.is_open() && __logfile.good())
		{
			__logfile << msges[log_level] << log_entry << "\n";
		}
		ret_val = true;
	}

	return ret_val;
}

void Alf_Log::alf_set_loglevel(const alf_log_level_e& log_level)
{
	__loglevel = log_level;
}
