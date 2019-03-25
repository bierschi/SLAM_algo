
#ifndef SRFSENSOR_H
#define SRFSENSOR_H
#include <stdint.h>

typedef uint16_t SRF_DistanceType;

extern SRF_DistanceType SRF_GetDistanceFrontLeft(void);

extern SRF_DistanceType SRF_GetDistanceFrontRight(void);

extern SRF_DistanceType SRF_GetDistanceRear(void);

extern void SRF_Init(void);

extern void SRF_MainFunction(void);

#endif
