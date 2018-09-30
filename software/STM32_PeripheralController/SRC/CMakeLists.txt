cmake_minimum_required(VERSION 3.8)

include(ARM_GCCToolchainFile.cmake)

project(STM32_HSPCar C ASM)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# define the source files to compile:
set(SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/Src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/stm32f3xx_hal_msp.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/stm32f3xx_it.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ComModule/ComModule.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/MotorControl/MotorControl.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/PIDControl/PIDControl.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/PlannerModule/PlannerModule.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ServoControl/ServoControl.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/Odometry/Odometry.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/MPUSensor/MPUSensor.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/startup_stm32f334x8.s
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_hrtim.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi_ex.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
)

#define the include directories:
set(INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Src
    ${CMAKE_CURRENT_LIST_DIR}/Src/ComModule
    ${CMAKE_CURRENT_LIST_DIR}/Src/MotorControl
    ${CMAKE_CURRENT_LIST_DIR}/Src/PIDControl
    ${CMAKE_CURRENT_LIST_DIR}/Src/PlannerModule
    ${CMAKE_CURRENT_LIST_DIR}/Src/ServoControl
    ${CMAKE_CURRENT_LIST_DIR}/Src/Odometry
    ${CMAKE_CURRENT_LIST_DIR}/Src/MPUSensor
)

# define compiler options:
set(C_FLAGS "-g -O0 -Wall -u _printf_float -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs --specs=nosys.specs -ffunction-sections -fdata-sections -DSTM32F334x8")
set(ASM_FLAGS "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(LD_FLAGS "-u _printf_float -static -T ${CMAKE_CURRENT_LIST_DIR}/Src/STM32F334R8Tx_FLASH.ld -Wl,--gc-sections -Wl,-Map,STM32_F334R8.map")

# set compiler options to CMake:
set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS ${ASM_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${LD_FLAGS})

include_directories(${INCLUDE_DIRS})

add_executable(STM32_HSPCar ${SRC_FILES})

add_custom_command(
    TARGET STM32_HSPCar
    POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" "-O" "binary" "./STM32_HSPCar" "./STM32_HSPCarBin")
