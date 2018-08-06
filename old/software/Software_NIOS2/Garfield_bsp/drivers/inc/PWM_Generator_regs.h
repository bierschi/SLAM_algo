/*!
 * @file
 * @brief The register description for the pwm core
 */

#ifndef PWM_GENERATOR_REGS_H
#define PWM_GENERATOR_REGS_H

#include <io.h>

/// offset which is added to the base address; behind the offset + base address there is the contorl register (thers is no more registers)
#define PWM_GENERATOR_CONTROL_REG               0
/// to calc the base address
#define IOADDR_PWMGen_CONTROL(base)             __IO_CALC_ADDRESS_NATIVE(base, PWM_GENERATOR_CONTROL_REG)
/// macro for reading the control register
#define IORD_PWMGen_Control(base)               IORD(base, PWM_GENERATOR_CONTROL_REG)
/// macro for writing to the control register
#define IOWR_PWMGen_Control(base, data)         IOWR(base, PWM_GENERATOR_CONTROL_REG, data)

#endif //PWM_GENERATOR_REGS_H
