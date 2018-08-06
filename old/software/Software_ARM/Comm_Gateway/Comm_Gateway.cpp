/*!
 * @file
 * */
#include "Comm_Gateway.hpp"
#include "hps_fpga_addresses.h"
#include <stdint.h>
#include <poll.h>
#include <fcntl.h>

///Port on which socket is created
#define COMPORT 6666

///Send/Receive Frequence in Hz
#define COMFREQ 50

///Alf Communication Server object
Alf_Communication<Server> ServerComm;

/// variable to let sleep the main thread
std::condition_variable Run_Main_Task_cond;
std::condition_variable Run_ServerWrite_Task;

/// mutex to for the main thread
std::mutex Run_Main_Task_mut;

///Alf Log
Alf_Log my_log;

///Shared Memory Mailbox object
Alf_SharedMemoryComm shared_mem;

///Run or close threads
bool run_threads = true;
bool notify_ServerWrite_Task = false;

int fd;


void Stop_Program(int sig) {

	Run_Main_Task_cond.notify_all();
}

/**
 * @brief This function is for interrupt handling in user mode. It should run in its own thread
 */
void HardwareReadHandler(void){
	Alf_Log::alf_log_write("Started HardwareReadHandler thread", log_info);
	Alf_Drive_Info drive_info_local_copy;
	uint32_t irq_info;
	int ret;
	ssize_t nb;
    struct pollfd fds = {
        .fd = fd,
        .events = POLLIN,
    };
	while(run_threads){
	    irq_info = 1; /* unmask */

	    nb = write(fd, &irq_info, sizeof(irq_info));
	    if (nb < sizeof(irq_info)){
	    	Alf_Log::alf_log_write("Cannot write to fd", log_error);
	    	run_threads = false;
	    	break;
	    }

		ret = poll(&fds, 1, -1);	//waiting until interrupt was coming
        if (ret >= 1) {
            nb = read(fd, &irq_info, sizeof(irq_info));
            if (nb == sizeof(irq_info)) {
            	shared_mem.ReadInterruptHandler();
            	notify_ServerWrite_Task = true;
            	Run_ServerWrite_Task.notify_one();
            }
        } else {
            Alf_Log::alf_log_write("Error while handling interrupt in user mode!", log_error);
            run_threads = false;
        }
	}
	uint32_t end = ALF_END_ID;
	shared_mem.Write(end);
	Alf_Log::alf_log_write("Ended HardwareReadHandler thread", log_info);
}

void writeData(void) {
	Alf_Log::alf_log_write("Started writeData thread", log_info);

	Alf_Drive_Info drive_info_local_copy;
	std::mutex mux;
	std::unique_lock<std::mutex> lock(mux);
	while(run_threads) {
		while(not notify_ServerWrite_Task){
			Run_ServerWrite_Task.wait(lock);
		}
		if(shared_mem.Read(drive_info_local_copy) == ALF_NO_ERROR){
			ServerComm.Write(drive_info_local_copy);
		}
		notify_ServerWrite_Task = false;
	}
	Alf_Log::alf_log_write("Ended writeData thread", log_info);
}

void readData(void) {
	std::string rec;
	Alf_Log::alf_log_write("Started readData thread", log_info);

	while(run_threads) {
		Alf_Urg_Measurements_Buffer readBuffer(10);
		alf_mess_types msgType;

		ServerComm.Read(readBuffer, msgType);

		rec = "Speed: " + std::to_string(global_drive_command.speed);
		rec += ", Direction: " + std::to_string(global_drive_command.direction);
		rec += ", Angle: " + std::to_string(global_drive_command.angle);
		rec += ", Light: " + std::to_string(global_drive_command.light);

		my_log.alf_log_write(rec, log_info);

		shared_mem.Write(global_drive_command);

		std::this_thread::sleep_for(std::chrono::milliseconds(1/COMFREQ*1000));
	}
	Alf_Log::alf_log_write("Ended readData thread", log_info);
}

int main()
{
	std::unique_lock<std::mutex> lck(Run_Main_Task_mut);
	signal(SIGINT, Stop_Program);

	my_log.alf_log_init("Melmac.log", log_debug, true);

	global_drive_info.speed = 42;
	global_drive_info.acceleration = 43;
	global_drive_info.lateral_acceleration = 44;
	global_drive_info.z_acceleration = 45;
	global_drive_info.Gyroscope_X = 1;
	global_drive_info.Gyroscope_Y = 1;
	global_drive_info.Gyroscope_Z = 1;
	global_drive_info.temperature = 13;

    fd = open("/dev/uio0", O_RDWR);
    if (fd < 0) {
        Alf_Log::alf_log_write("Opening /dev/uio0 does not work -> Abort!", log_error);
        exit(EXIT_FAILURE);
    }
	if(shared_mem.Init(SHARED_MEMORY_MASTER_HPS_0_BASE, SHARED_MEMORY_MUTEX_MASTER_HPS_0_BASE, MAILBOX_ARM2NIOS_0_BASE, SHARED_MEMORY_MASTER_NIOS_0_BASE, SHARED_MEMORY_MUTEX_MASTER_NIOS_0_BASE, MAILBOX_NIOS2ARM_0_BASE, 01, HPS_OFFSET)) {

		shared_mem.EnableMailboxInterrupt();
		Alf_Log::alf_log_write("Initialized Mailbox", log_info);

		if(ServerComm.Init(COMPORT)) {

			Alf_Log::alf_log_write("Created socket", log_info);
			shared_mem.WriteInterfaceStatus = true;
			std::thread hardwareReadThread(HardwareReadHandler);
			std::thread sendThread(writeData);
			std::thread recThread(readData);

			Run_Main_Task_cond.wait(lck);

			run_threads = false;

			sendThread.join();
			recThread.join();
			hardwareReadThread.join();

			close(fd);
			ServerComm.EndCommunication();
		}
		else {
			Alf_Log::alf_log_write("Could not create socket", log_error);
		}
	}
	else {
		Alf_Log::alf_log_write("Could not initialize Mailbox", log_error);
	}


	Alf_Log::alf_log_write("Ending the application", log_info);
	Alf_Log::alf_log_end();

  	return 0;
}
