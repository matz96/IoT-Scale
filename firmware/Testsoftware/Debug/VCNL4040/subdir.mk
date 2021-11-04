################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VCNL4040/vcnl4040.c 

OBJS += \
./VCNL4040/vcnl4040.o 

C_DEPS += \
./VCNL4040/vcnl4040.d 


# Each subdirectory must supply rules for building sources it contributes
VCNL4040/%.o: ../VCNL4040/%.c VCNL4040/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F070xB -c -I../Core/Inc -I../VCNL4040 -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/engel/OneDrive - FHNW/pro3E/Software/pro3e/firmware/Testsoftware/VCNL4040" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

