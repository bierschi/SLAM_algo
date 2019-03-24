
#ifndef SPILIBRARY_H
#define SPILIBRARY_H

#include "ComStructure.h"

extern void spiOpen(void);
extern void spiSend(ComStructureType &tx, ComStructureType &rx);
extern void spiClose(void);

#endif
