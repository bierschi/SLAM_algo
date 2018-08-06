/*!
 * @file
 * @brief a library give access to log variants and functionality for this
 */
#ifndef ALF_LOG
#define ALF_LOG

#include <string>
#include <stdio.h>

/// LOG_ENABLE does enabling the log, with LOG_DISABLE there are no further log informations
#define LOG_ENABLE
//defines
#ifdef LOG_ENABLE
#define ALF_LOG_INIT(args...) Alf_Log::alf_log_init(args)
#define ALF_LOG_WRITE(args...) Alf_Log::alf_log_write(args)
#define ALF_LOG_END() Alf_Log::alf_log_end()
#define ALF_LOG_SET_LEVEL(a) ALF_Log::alf_set_loglevel(a)
#endif
#ifdef LOG_DISABLE
#define ALF_LOG_INIT(args...)
#define ALF_LOG_WRITE(args...)
#define ALF_LOG_END()
#define ALF_LOG_SET_LEVEL(a)
#endif //LOG_ENABLE

/*!
 * @brief all log leves which are available <br>
 * the log levels are based on each other, which means, that every log_error is also a log_warning, log_info, log_debug, but a log_info is no log_warning but a log_debug
 */
enum alf_log_level_e{
	/// strongest error, should be used if the desired function of the application could not be provided
	log_error = 0,
	/// a warning should be used it the execution of the application is in danger, but it is still running
	log_warning,
	/// just for info messages, which could be later used in case of errors or warnings to see the control flow etc.
	log_info,
	/// developer informations
	log_debug,
};

/*!
 * @brief This class handle all the log informations. There will be always a log file, additional the log can be printed to standard output
 */
class Alf_Log{
private:

	enum color{
		error = 31, //red
		info = 32, //green
		warning = 33, //yellow
		debug = 37 //white
	};

	static const std::string msges[];
	static std::ofstream __logfile;
	static alf_log_level_e __loglevel;
	static bool __con_out;
public:
	/*!
	 * @brief Initialize the logging functionality (performed with a file)
	 * @param[in] filename Path to File
	 * @param[in] loglevel All Messages with level above will be logged
	 * @param[in] consoleoutput If true all messages will be printed on console ouptut
	 * @return true if successful otherwise false
	 */
	static bool alf_log_init(const std::string& filename = "dummy.alf_log", const alf_log_level_e& log_level = log_debug, const bool& console_output = false);
	/*!
	 * @brief Writes a log entry
	 * @param[in] log_entry the message to be logged
	 * @param[in] log_level the significance of the message
	 * @return true if successful otherwise false
	 */
    static bool alf_log_write(const std::string& log_entry, const alf_log_level_e& log_level = log_debug);
	/*!
	 * @brief close the logging
	 * @param[in] -
	 * @return true if successful otherwise false
	 */
	static bool alf_log_end(void);
	/*!
	 * @brief Set the log level
	 * @param[in] log_level which messages should be logged from now on
	 * @return -
	 */
	static void alf_set_loglevel(const alf_log_level_e& log_level);
};

#endif
