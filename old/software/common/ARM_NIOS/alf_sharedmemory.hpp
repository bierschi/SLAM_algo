/**
 * @file
 * @brief Header file of abstraction class for hardware communication on the hardware shared memory (with mutex and mailbox) in the garfield project.
 * alf_sharedmemory.hpp
 *
 *  Created on: 02.03.2017
 *      Author: florian
 */

#ifndef ARM_NIOS_ALF_SHAREDMEMORY_HPP_
#define ARM_NIOS_ALF_SHAREDMEMORY_HPP_

#include "alf_erno.h"
#include "alf_data_info.hpp"
#include <cinttypes>
#include <stack>

/**
 * @brief Implementation of a ringbuffer with fixed size. If the queue is full, the oldest element will be overwritten.
 */
template <class obj, uint32_t size>
class Garifield_RingBuffer {
public:
	/**
	 * @brief      returns the top element on the ring buffer (is the actualst)
	 *
	 * @return     the top element, could be of any datatype
	 * @attention  a call to #pop() is necessary if the element should removed from the ring buffer
	 */
	obj top(){
		return _stack[_top_element];
	}

	/**
	 * @brief      Removes the top element of the ringbuffer. This element is the actualst element, next top element ist n-1.
	 */
	void pop(){
		if(not _empty){
			if(_top_element == _first_element) _empty = true;
			else{
				if(_top_element == 0) _top_element = _max_size;
				_top_element--;
			}
		}
	}
	/**
	 * @brief      Is the ring buffer empty?
	 *
	 * @return     true = empty, false = elements in the ring buffer
	 */
	bool empty(){
		return _empty;
	}
	/**
	 * @brief      Pushs a element to the ring buffer. If the ring buffer is full, the oldest element in there will be overwritten.
	 *
	 * @param[in]  a     The element to push into.
	 */
	void push(const obj &a){
		if(not _empty){
			_top_element = (_top_element + 1)%_max_size;
			if(_top_element == _first_element){
				_first_element = (_first_element+1)%_max_size;
			}
			_stack[_top_element] = a;
			_empty = false;
		}
		_stack[_top_element] = a;
		_empty = false;
	}
private:
	const uint32_t _max_size = size;
	obj _stack[size];
	uint32_t _top_element = 0;
	uint32_t _first_element = 0;
	bool _empty = true;
};

/**
 * @brief Implementation for communcatiing via a shared memory section on the fpga. Abstraction for the mailbox, the hardware mutex and the shared memory in both directions.
 */
class Alf_SharedMemoryComm{
private:

	struct mailbox_s{
		uint32_t reg;
	};

	/// the real address of the write shared memory section
	uint32_t _shared_memory_wr_addr;
	/// the virtual address of the shared memory within the context (in linux this is virtuel, within NIOS without MMU not)
	void *_shared_memory_wr;
	/// the real address of the write mutex peripheral
	uint32_t _wr_mutex_addr;
	/// the virtual|real address of the write mutex peripheral
	void *_wr_mutex;
	/// rhe real address of the write mailbox peripheral
	uint32_t _wr_mailbox_addr;
	/// the virtual|real address of the write mailbox periph
	void *_wr_mailbox;

	/// the real address of the read shared memory section
	uint32_t _shared_memory_rd_addr;
	/// the virtual|real address of the shared memory section
	void *_shared_memory_rd;
	/// the real address of the mutex for the read memory section
	uint32_t _rd_mutex_addr;
	/// the virtual|real address of the mutex for the read memory section
	void *_rd_mutex;
	/// the real address of the mailbox for reading of the shared memory
	uint32_t _rd_mailbox_addr;
	/// the virtual|real address of the mailbox
	void *_rd_mailbox;

	/// The CPU-ID which is unique in system context. This is very important for giving access to the shared memory via locking it through the mutex. This should be 0x03 for NIOS2 processor and 0x01 for ARM HPS subsystem.
	uint16_t _cpu_id;
	/// flag that all addresses for read of the shared memory could be mapped
	bool _all_read_addr_mapped;
	/// flag that all addresses for write of the shared memory could be mapped
	bool _all_write_addr_mapped;

	/// used for calculating the position wihtin the shared memory section. The shared memory is used like a queue, #wr_memory_pos goes up until it reachs the end, then starting from beginning
	uint16_t _wr_memory_pos;
	/// the fix address offset we need in the hps (linux) subsystem. This is usually for Cyclone V 0xff200000, but we handle it dynamicly via the #Init function
	uint32_t _wr_memory_addr_offset;
	/// the size of the shared memory, taken from the system overview
	const uint32_t _shared_memory_size = 2048;
	/// the size (span, how much register are there) of the mailbox, taken from the system overview
	const uint32_t _mailbox_size = 16;
	/// the size of the mutex, taken from the sys overview
	const uint32_t _mutex_size = 8;

	/// This ringbuffer stores the addresses readed from the mailbox where the info commands are stored
	Garifield_RingBuffer<mailbox_s, 12> _buffer_drive_info;
	/// This ringbuffer stores the addresses readed from the mailbox where the commands are stored
	Garifield_RingBuffer<mailbox_s, 12> _buffer_drive_command;

	/**
	 * brief Trys to get a lock from the hardware mutex. This does not tries to get the lock until it works, only one time. If you want to stop the application until getted the lock, use #WaitForLock
	 * @param  addr The address of the mutex lock register. Normally one of #_wr_mutex, #_rd_mutex
	 * @return      true if we get the lock, false otherwise
	 */
	bool TryLock(void *addr);
	/**
	 * @brief Releases the lock on the hardware mutex.
	 * @param addr The address of the mutex lock register. Normally one of #_wr_mutex, #_rd_mutex
	 */
	void ReleaseLock(void *addr);
	/**
	 * @brief Writes the given address (from where the receiving processor should read) to the mailbox pointer register (offset 0x04) and the message_id to the mailbox command register (offset 0x00). From there, the receiving
	 *        processors can read this and work with it!
	 * @param  addr The base address of the mailbox, normally one of #_wr_mailbox, #_rd_mailbox
	 * @param  top_address The address the RECEIVING processor could read the data!
	 * @param  message_id  The message id, see #alf_message_types
	 * @return             true if the write has worked, false otherwise
	 * @attention The #top_address must be calculated in a proper way! If the hps want to send a top_address, it must first minus the hps offset
	 */
	bool WriteAndCommitMailbox(const void *addr, const uint32_t top_address, const uint32_t &message_id);
    /**
     * @brief Busy waiting for the lock. Its just an internal call to #TryLock until this function returns true.
     * @param addr The address of the mutex, normally one of #_wr_mailbox, #_rd_mailbox
     */
	void WaitForLock(void *addr);
    /**
     * @brief Resets the reset register of the write mutex, write 1 to the reset register(meaning clearing the register) and releases the mutex register which is locks the mutex during boot up for the #_cpu_id
     */
	void ResetWriteMutex();

	template <typename t>
    /**
     * @brief Locks the write mutex, write the obj via a memcpy to the shared memory and commits the message to the mailbox! If this function cannot get the mutex lock, it returns #ALF_LOCK_MEMORY_FAILED
     * @param  obj        The object to write to the shared memory.
     * @param  message_id The message id which is suitable for the object
     * @param  size       The size in bytes of the object to write (normally with sizeof determined)
     * @return            One of #ALF_ERROR_CODES
     */
	alf_error HardwareWrite(const t &obj, const uint32_t &message_id, const uint32_t &size);

	template <typename t>
    /**
     * @brief Try's to read one object with the type of #obj from the shared memory. If there is a object to read, this function locks the mutex and read the object from the shared memory with the address readed from the mailbox
     *        (anywhere in the past). It always returns the most recent object. If there are older objects, you can call this function more than one times and get from the newest to the oldest value.
     * @param  obj  The object where the data should be saved into
     * @param  size The size of the object in bytes
     * @param  addr The address where the data which should be stored into the object are stored (a location at the shared memory)
     * @return      One of #ALF_ERROR_CODES
     */
	alf_error HardwareRead(t &obj, const uint32_t &size, const uint32_t &addr);

public:

	/**
	 * @brief Flag to disable (=false) or enable (=true) the read interface.
	 * @attention Actual not used, just for completness
	 */
	bool ReadInterfaceStatus;

	/**
	 * @brief Enables (=true) or disables (=false) the write operations to hardware. If set to false, all write operations #Write will return the error #ALF_WRITE_SHARED_MEMORY_DISABLED.
	 */
	bool WriteInterfaceStatus;
    /**
     * @brief Initialize the hardware communication with the shared memory
     * @param  sh_mem_wr_addr The base address of the shared memory where the instance of this class should write its data
     * @param  wr_mutex_addr  The base address of the mutex which should lock all writes from this instance
     * @param  wr_mb_addr     The base address of the mailbox which this instance should write his data
     * @param  sh_mem_rd_addr The base address of the shared memory where the instance of this class should read data (receiver)
     * @param  rd_mutex_addr  The base address of the mutex where the instance is the receiver
     * @param  rd_mb_addr     The base address of the mailbox where the instance is the receiver
     * @param  cp_id          The cpu id which instantiate this class. Normally 0x01 for HSP and 0x03 for NIOS 2
     * @param  addr_offset    The general address offset for all address defined. In NIOS2 this should be 0, within HPS on Cylcone V this is normally 0xff200000
     * @return                true if all addresses could be mapped in a proper way and all read/write operation can be used, false otherwise
     */
	bool Init(uint32_t sh_mem_wr_addr, uint32_t wr_mutex_addr, uint32_t wr_mb_addr, uint32_t sh_mem_rd_addr,
			 uint32_t rd_mutex_addr, uint32_t rd_mb_addr, uint16_t cp_id, uint32_t addr_offset);

	/**
	 * @brief Writes an #Alf_Drive_Info to the shared memory section.
	 * @param  drive The #Alf_Drive_Info object which should be written to the shared memory via memcpj
	 * @return       One of #ALF_ERROR_CODES
	 */
	alf_error Write(const Alf_Drive_Info &drive);

	/**
	 * @brief Write an #Alf_Drive_Command to the shared memory section.
	 * @param  drive The #Alf_Drive_Command which should be written to the shared memory
	 * @return       One of #ALF_ERROR_CODES
	 */
	alf_error Write(const Alf_Drive_Command &drive);

	/**
	 * @brief      Writes a simple number into the mailbox. This number will be used within the mailbox comannd register and in the shared memory!
	 *
	 * @param      num   The number
	 *
	 * @return     One of #ALF_ERROR_CODES
	 */
	alf_error Write(uint32_t &num);

	/**
	 * @brief Reads one #Alf_Drive_Command from the shared memory, if there is one to read. This function changes memory of the given object!
	 * @param  drive The #Alf_Drive_Command where the informations should be stored
	 * @return       One of #ALF_ERROR_CODES
	 */
	alf_error Read(Alf_Drive_Command &drive);

	/**
	 * @brief Reads one #Alf_Drive_Info from the shared memory if there is one to read. Returns an errorcode otherwise
	 * @param  drive The #Alf_Drive_Info where the informations should be stored
	 * @return       One of #Alf_Drive_CODES
	 */
	alf_error Read(Alf_Drive_Info &drive);

    /**
     * @brief The routine which should be called within the interrupt routine for receiving messages. It saves the pointer and command register from the read mailbox and saves that information. Later in a programm context, you can read
     *        a object from the shared memory with #Read
     * @example using_shared_memory_example.cpp This is an example how to proper use the class and in special this function!
     */
	void ReadInterruptHandler(void);
	
	/**
     * @brief Disables the interrupt on receiving messages
     */
	void DisableMailboxInterrupt();

    /**
     * @brief Enables all interrupts of the mailbox (at this moment: only on receiving messages)
     */
	void EnableMailboxInterrupt();
};



#endif /* ARM_NIOS_ALF_SHAREDMEMORY_HPP_ */
