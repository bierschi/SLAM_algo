/*!
 * @file
 * */
/*
 * Comm_Gateway.hpp
 *
 *  Created on: 16.03.2017
 *      Author: philipp
 */

#ifndef COMM_GATEWAY_HPP_
#define COMM_GATEWAY_HPP_

#include <string>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>

#include <thread>
#include <chrono>

#include <condition_variable>

//ALF INCLUDES
#include "alf_log.hpp"
#include "alf_data.hpp"
#include "alf_data_info.hpp"
#include "alf_erno.h"
#include "alf_communication.hpp"
#include "alf_message_types.hpp"

#include "alf_sharedmemory.hpp"


/*!
 * @brief Sig Handler for closing the socket
 * @param[in] sig - the signal
 */
void Stop_Program(int sig);

/**
 * @brief This function is for interrupt handling from the hardware mailbox in user mode. It should run in its own thread.
 */
void HardwareReadHandler(void);

/*!
 * @brief writeData() Function runs in a thread an writes cyclic the alf_drive_info data in the socket for Garfield_control to display
 */
void writeData(void);

/*!
 * @brief readData() Function runs in a thread an reads cyclic the alf_drive_command data from the socket to send it over the Mailbox to the NIOS2
 */
void readData(void);

/*!
 * @brief The Main function
 */
int main();



#endif /* COMM_GATEWAY_HPP_ */
