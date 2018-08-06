/*!
 * @file
 * @brief
 */

#ifndef PWM_GENERATOR_H_
#define PWM_GENERATOR_H_

#include "alt_types.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/// for disabling the pwm signal, its just puts 0 to the generated signal
#define PWMGen_Disable PWMGen_Set_DutyCycle(0)
/// init function for the pwm, writing 0 to the output
#define PWMGen_Init PWMGen_Set_DutyCycle(0)

typedef alt_u32 PWMGen_DutyCycle_Width;
/*!
   \brief sets the dutycycle in the pwm control register
   \param base - the base address of the pwm control register, can be normally found in the generated system.h header file
   \param dutycylce - the dutycylce, at the every value between 0 (a low pulse) and 255 (a HIGH pulse)
   \pre -
   \post -
   \return -
*/
void PWMGen_Set_DutyCycle(alt_u32 base, PWMGen_DutyCycle_Width dutycycle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //PWM_GENERATOR_H_
