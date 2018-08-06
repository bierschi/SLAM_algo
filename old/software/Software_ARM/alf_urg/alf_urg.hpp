/*!
 * @file
 */
#ifndef ALF_URG
#define ALF_URG

#include <iostream>
#include <sstream>
#include <string>

#include <signal.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "alf_log.hpp"
#include "alf_data.hpp"
#include "alf_erno.h"
#include "alf_communication.hpp"
#include "alf_message_types.hpp"
#include "alf_sensors.hpp"

#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_errno.h"

int main();



#endif
