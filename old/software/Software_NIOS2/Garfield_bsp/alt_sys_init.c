/*
 * alt_sys_init.c - HAL initialization source
 *
 * Machine generated for CPU 'nios2_gen2_0' in SOPC Builder design 'Garfield_system'
 * SOPC Builder design path: ../../Garfield_system.sopcinfo
 *
 * Generated: Wed Jun 07 14:02:01 GMT+01:00 2017
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#include "system.h"
#include "sys/alt_irq.h"
#include "sys/alt_sys_init.h"

#include <stddef.h>

/*
 * Device headers
 */

#include "altera_nios2_gen2_irq.h"
#include "altera_avalon_epcs_flash_controller.h"
#include "altera_avalon_jtag_uart.h"
#include "altera_avalon_mailbox_simple.h"
#include "altera_avalon_mutex.h"
#include "altera_avalon_spi.h"
#include "altera_avalon_sysid_qsys.h"
#include "altera_avalon_timer.h"
#include "i2c_opencores.h"

/*
 * Allocate the device storage
 */

ALTERA_NIOS2_GEN2_IRQ_INSTANCE ( NIOS2_GEN2_0, nios2_gen2_0);
ALTERA_AVALON_EPCS_FLASH_CONTROLLER_INSTANCE ( EPCS_FLASH_CONTROLLER_0, epcs_flash_controller_0);
ALTERA_AVALON_JTAG_UART_INSTANCE ( JTAG_UART_NIOS2, jtag_uart_nios2);
ALTERA_AVALON_MAILBOX_SIMPLE_INSTANCE ( MAILBOX_ARM2NIOS_0, mailbox_arm2nios_0);
ALTERA_AVALON_MAILBOX_SIMPLE_INSTANCE ( MAILBOX_NIOS2ARM_0, mailbox_nios2arm_0);
ALTERA_AVALON_MUTEX_INSTANCE ( SHARED_MEMORY_MUTEX_MASTER_HPS_0, shared_memory_mutex_master_hps_0);
ALTERA_AVALON_MUTEX_INSTANCE ( SHARED_MEMORY_MUTEX_MASTER_NIOS_0, shared_memory_mutex_master_nios_0);
ALTERA_AVALON_SPI_INSTANCE ( SPI_0, spi_0);
ALTERA_AVALON_SYSID_QSYS_INSTANCE ( SYSID_FPGA, sysid_fpga);
ALTERA_AVALON_TIMER_INSTANCE ( TIMER_0_NIOS2, timer_0_nios2);
I2C_OPENCORES_INSTANCE ( I2C_OPENCORES_0, i2c_opencores_0);

/*
 * Initialize the interrupt controller devices
 * and then enable interrupts in the CPU.
 * Called before alt_sys_init().
 * The "base" parameter is ignored and only
 * present for backwards-compatibility.
 */

void alt_irq_init ( const void* base )
{
    ALTERA_NIOS2_GEN2_IRQ_INIT ( NIOS2_GEN2_0, nios2_gen2_0);
    alt_irq_cpu_enable_interrupts();
}

/*
 * Initialize the non-interrupt controller devices.
 * Called after alt_irq_init().
 */

void alt_sys_init( void )
{
    ALTERA_AVALON_TIMER_INIT ( TIMER_0_NIOS2, timer_0_nios2);
    ALTERA_AVALON_EPCS_FLASH_CONTROLLER_INIT ( EPCS_FLASH_CONTROLLER_0, epcs_flash_controller_0);
    ALTERA_AVALON_JTAG_UART_INIT ( JTAG_UART_NIOS2, jtag_uart_nios2);
    ALTERA_AVALON_MAILBOX_SIMPLE_INIT ( MAILBOX_ARM2NIOS_0, mailbox_arm2nios_0);
    ALTERA_AVALON_MAILBOX_SIMPLE_INIT ( MAILBOX_NIOS2ARM_0, mailbox_nios2arm_0);
    ALTERA_AVALON_MUTEX_INIT ( SHARED_MEMORY_MUTEX_MASTER_HPS_0, shared_memory_mutex_master_hps_0);
    ALTERA_AVALON_MUTEX_INIT ( SHARED_MEMORY_MUTEX_MASTER_NIOS_0, shared_memory_mutex_master_nios_0);
    ALTERA_AVALON_SPI_INIT ( SPI_0, spi_0);
    ALTERA_AVALON_SYSID_QSYS_INIT ( SYSID_FPGA, sysid_fpga);
    I2C_OPENCORES_INIT ( I2C_OPENCORES_0, i2c_opencores_0);
}
