/*
 * hps_fpga_addresses.h
 *
 *  Created on: 23.03.2017
 *      Author: florian
 */

#ifndef COMM_GATEWAY_HPS_FPGA_ADDRESSES_H_
#define COMM_GATEWAY_HPS_FPGA_ADDRESSES_H_

#define HPS_OFFSET 								0xff200000
#define SHARED_MEMORY_MASTER_HPS_0_BASE 		0x60000
#define SHARED_MEMORY_MUTEX_MASTER_HPS_0_BASE 	0x50000
#define SHARED_MEMORY_MUTEX_MASTER_NIOS_0_BASE	0x80000
#define SHARED_MEMORY_MASTER_NIOS_0_BASE		0x90000
#define MAILBOX_ARM2NIOS_0_BASE					0x20000
#define MAILBOX_NIOS2ARM_0_BASE					0x70000
#define MAILBOX_NIOS2ARM_0_SPAN					0x16
#define MAILBOX_NIOS2ARM_0_INT_REG				0xC

#endif /* COMM_GATEWAY_HPS_FPGA_ADDRESSES_H_ */