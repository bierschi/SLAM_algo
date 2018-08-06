/*!
 * @file
 * @brief The register description for the rotary encoder core
 */

#ifndef ROTARY_ENCODER_REGS_H
#define ROTARY_ENCODER_REGS_H

#include <io.h>

#define ROTARY_ENCODER_CONTROL_REG                  0

#define ROTARY_ENCODER_RESULT_REG                   1

#define IOADDR_ROTARY_ENCODER_CONTROL(base)         __IO_CALC_ADDRESS_NATIVE(base, ROTARY_ENCODER_CONTROL_REG)

#define IOADDR_ROTARY_ENCODER_RESULT(base)          __IO_CALC_ADDRESS_NATIVE(base, ROTARY_ENCODER_RESULT_REG)

#define Read_ROTARY_ENCODER_CONTROL(base)           IORD(base, ROTARY_ENCODER_CONTROL_REG)

#define Write_ROTARY_ENCODER_CONTROL(base,data)     IOWR(base, ROTARY_ENCODER_CONTROL_REG, data)

#define Read_ROTARY_ENCODER_RESULT(base)            IORD(base, ROTARY_ENCODER_RESULT_REG)

#define ROTARY_ENCODER_BIT_ENABLE                   0
#define ROTARY_ENCODER_BIT_CLEARCOUNTER             1
#define ROTARY_ENCODER_BIT_RESET                    2

#endif //ROTARY_ENCODER_REGS_H
