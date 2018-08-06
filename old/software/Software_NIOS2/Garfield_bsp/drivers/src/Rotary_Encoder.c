/*!
 * @file
 */

#include "Rotary_Encoder.h"

uint32_t ROT_ENC_GetRotations(uint32_t base){
    return Read_ROTARY_ENCODER_RESULT(base);
}

void ROT_ENC_SetNewStatus(uint32_t base, uint8_t status){
	Write_ROTARY_ENCODER_CONTROL(base, status << ROTARY_ENCODER_BIT_ENABLE);
}

void ROT_ENC_ClearCounter(uint32_t base){
    volatile uint32_t reg = Read_ROTARY_ENCODER_CONTROL(base);
    reg |= (1 << ROTARY_ENCODER_BIT_CLEARCOUNTER);
    Write_ROTARY_ENCODER_CONTROL(base,reg);
}
