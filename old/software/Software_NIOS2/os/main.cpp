/*!
 * @file
 * */
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "alf_sharedmemory.hpp"
#include "Drive.hpp"

#include "tasks_nios.hpp"
#include "i2c_opencores.h"


int main()
{
	//Display lcd(240,320);
	//lcd.init(ILI9341_BLACK);
	//lcd.fillScreen(ILI9341_BLACK);
	//lcd.setRotation(3);

	//lcd.writeLine("Start Application", ILI9341_DARKGREEN, 2);


	IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LED_BASE, 0x0F);
	/* init section */
	alt_irq_register(MAILBOX_ARM2NIOS_0_IRQ, 0, Mailbox_isr);
	I2C_init(I2C_OPENCORES_0_BASE, ALT_CPU_CPU_FREQ, 100000);
	sharedMem.Init(SHARED_MEMORY_MASTER_NIOS_0_BASE, SHARED_MEMORY_MUTEX_MASTER_NIOS_0_BASE, MAILBOX_NIOS2ARM_0_BASE, SHARED_MEMORY_MASTER_HPS_0_BASE, SHARED_MEMORY_MUTEX_MASTER_HPS_0_BASE, MAILBOX_ARM2NIOS_0_BASE, 0x03, 0);
	sharedMem.EnableMailboxInterrupt();
	Drive::SetDriveSpeed(0,0);

	/* set up of all needed NIOS tasks which will run periodically */
	xTaskCreate(readUltraSonic, "1", 512, NULL, 3, NULL);
	xTaskCreate(readMPU, "2", 512, NULL, 2, NULL);
	xTaskCreate(readRotary, "3", 512, NULL, 2, NULL);
	xTaskCreate(setMotor_and_Steering, "4", 512, NULL, 3, NULL);
	xTaskCreate(setDriveInfo, "5", 256, NULL, 1, &writeTask);

	vTaskStartScheduler();
	return 0;
}
