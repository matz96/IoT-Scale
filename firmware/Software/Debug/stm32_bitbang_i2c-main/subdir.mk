################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32_bitbang_i2c-main/dwt_stm32_delay.c \
../stm32_bitbang_i2c-main/stm32_sw_i2c.c 

OBJS += \
./stm32_bitbang_i2c-main/dwt_stm32_delay.o \
./stm32_bitbang_i2c-main/stm32_sw_i2c.o 

C_DEPS += \
./stm32_bitbang_i2c-main/dwt_stm32_delay.d \
./stm32_bitbang_i2c-main/stm32_sw_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
stm32_bitbang_i2c-main/%.o: ../stm32_bitbang_i2c-main/%.c stm32_bitbang_i2c-main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Core/Inc -I../App/Inc -I../ssd1306 -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I"C:/Users/engel/OneDrive - FHNW/pro3E/Software/pro3e/firmware/Software/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

