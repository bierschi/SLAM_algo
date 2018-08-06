#include "PWM_Generator.h"
#include "PWM_Generator_regs.h"

void PWMGen_Set_DutyCycle(alt_u32 base, PWMGen_DutyCycle_Width dutycycle){
    IOWR_PWMGen_Control(base, dutycycle);
}
