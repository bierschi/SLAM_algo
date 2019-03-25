#include "SPILibrary.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include "ComStructure.h"

#ifndef OFFLINE_SIMU
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#endif

// adjust struct packing size
#define PATH_TO_SPI "/dev/spidev0.0"
#define SPI_FREQUENCY 100000
#define SUCCESS 0

int fdToSpiDev = 0;
bool initialized = false;

void spiOpen(void)
{
#ifndef OFFLINE_SIMU
    /*describe the transmitting options here:*/
    uint8_t mode = SPI_MODE_0;/*...specific for MCP23S17*/
    uint8_t bits_pW = 8;
    uint32_t speed = SPI_FREQUENCY;/* SPI_FREQUENCY Hz*/

    fdToSpiDev = open(PATH_TO_SPI, O_RDWR);

    if(fdToSpiDev < 0) {
        printf("\nCould not open SPI Bus Interface!\n");
    }
    else {
        if(ioctl(fdToSpiDev,SPI_IOC_WR_MODE,&mode) < 0) {printf("\nFehler bei der Mode-selection RD!\n");}
        if(ioctl(fdToSpiDev,SPI_IOC_WR_BITS_PER_WORD,&bits_pW) < 0) {printf("\nFehler bei der Mode-selection bpW!\n");}
        if(ioctl(fdToSpiDev,SPI_IOC_WR_MAX_SPEED_HZ,&speed) < 0) {printf("\nFehler bei der Mode-selection SPEED_HZ!\n");}
        initialized = true;
    }
#endif
}

void spiSend(ComStructureType &tx, ComStructureType &rx)
{
#ifndef OFFLINE_SIMU
    if(fdToSpiDev > 0) {
        struct spi_ioc_transfer spi_tf = {
                .tx_buf = (unsigned long ) &tx, /*send buffer*/
                .rx_buf = (unsigned long ) &rx, /*receive buffer*/
                .len = sizeof(ComStructureType),
                .speed_hz = SPI_FREQUENCY, /* SPI_FREQUENCY Hz*/
                .delay_usecs = 0,
                .bits_per_word = 8,
        };

        if((ioctl(fdToSpiDev, SPI_IOC_MESSAGE(1),&spi_tf)) < 0)
        {
            printf("\nCould not send SPI-Message!\n");
        }
    }
#endif
}

void spiClose(void)
{
#ifndef OFFLINE_SIMU
    if (fdToSpiDev != 0 && true == initialized)
    {
        close(fdToSpiDev);
    }
#endif
}
