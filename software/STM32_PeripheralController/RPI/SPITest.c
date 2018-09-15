#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ComStructure.h"

// adjust struct packing size
#define PATH_TO_SPI "/dev/spidev0.0"
#define SUCCESS 0

ComStructureType COM_Struct;
int fdToSpiDev = 0;
unsigned char initialized = 0u;

void spiOpen(void)
{
    /*describe the transmitting options here:*/
	uint8_t mode = SPI_MODE_0;/*...specific for MCP23S17*/
	uint8_t bits_pW = 8;
	uint32_t speed = 100000;/*transmit frequency 100 kHz*/

	fdToSpiDev = open(PATH_TO_SPI, O_RDWR);

	if(fdToSpiDev < 0) {
		printf("\nCould not open SPI Bus Interface!\n");
	}
	else {
		if(ioctl(fdToSpiDev,SPI_IOC_WR_MODE,&mode) < 0) {printf("\nFehler bei der Mode-selection RD!\n");}
		if(ioctl(fdToSpiDev,SPI_IOC_WR_BITS_PER_WORD,&bits_pW) < 0) {printf("\nFehler bei der Mode-selection bpW!\n");}
		if(ioctl(fdToSpiDev,SPI_IOC_WR_MAX_SPEED_HZ,&speed) < 0) {printf("\nFehler bei der Mode-selection SPEED_HZ!\n");}
		initialized = 1u;
	}
}

void spiSend(void)
{
    if(fdToSpiDev > 0) {
        struct spi_ioc_transfer spi_tf = {
		        .tx_buf = (unsigned long ) &COM_Struct, /*send buffer*/
		        .rx_buf = (unsigned long ) &COM_Struct, /*receive buffer*/
		        .len = sizeof(ComStructureType),
		        .speed_hz = 100000, /*10 kHz*/
		        .delay_usecs = 0,
		        .bits_per_word = 8,
	    };

	    if((ioctl(fdToSpiDev, SPI_IOC_MESSAGE(1),&spi_tf)) < 0)
	    {printf("\nCould not send SPI-Message!\n");
	    }
	}
}

void spiClose(void)
{
    if (fdToSpiDev != 0)
    {
        close(fdToSpiDev);
    }
}

int main (int argc, char * argv)
{
    int goOn = 1u;
    uint16_t speed = 0u;
    uint8_t direction = 0u;
    float angle = 0.0f;
    int speedInt = 0;
    COM_Struct.CurrentSteeringMode = 2u;
    COM_Struct.CurrentSteeringAngle = 45.0f;
    COM_Struct.CurrentSteeringDirection = 1u;
    COM_Struct.CurrentSteeringSpeed = speed;

    COM_Struct.Target_X = 0.0f;
    COM_Struct.Target_Y = 0.0f;

    spiOpen();

    while(goOn > 0)
    {
        printf("\nEnter Speed of Motor 0...1000\n");
        scanf("%i", &speedInt); speed = (uint16_t) speedInt;
        printf("\nEnter direction of Motor 0...1\n");
        scanf("%i", &direction);
	    printf("\nEnter angle of Motor -90.0f...90.0f\n");
        scanf("%f", &angle);
	
	    COM_Struct.CurrentSteeringMode = 2u; // Manual
	    COM_Struct.CurrentSteeringAngle = angle;
	    COM_Struct.CurrentSteeringDirection = direction;
	    COM_Struct.CurrentSteeringSpeed = speed;
	    
        spiSend();
        printf("\nContinue? 0...1\n");
        scanf("%i", &goOn);
    }

    spiClose();
}



