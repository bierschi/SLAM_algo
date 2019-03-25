
#ifndef SPILIBRARY_H
#define SPILIBRARY_H

#include "ComStructure.h"

typedef enum SpiOpenStatusEnum {
    OPEN,
    CLOSED
} SpiOpenStatusType;

extern void spiOpen(void);
extern void spiSend(ComStructureType &tx, ComStructureType &rx);
extern void spiClose(void);
extern SpiOpenStatusType spiIsOpen(void);

#endif
