/*!
 * @file
 * @brief header file for the rotary encoder.
 * Gives comfortable access to all functions of the rotary encoder
 */

#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdint.h>
#include "Rotary_Encoder_regs.h"
#include <math.h>

#define ROT_ENC_Reset(base) Write_ROTARY_ENCODER_CONTROL(base, pow(2, ROTARY_ENCODER_BIT_RESET))

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

uint32_t ROT_ENC_GetRotations(uint32_t base);

void ROT_ENC_SetNewStatus(uint32_t base, uint8_t status);

void ROT_ENC_ClearCounter();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //ROTARY_ENCODER_H
