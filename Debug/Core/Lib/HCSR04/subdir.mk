################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/HCSR04/HCSR04.c \
../Core/Lib/HCSR04/HCSR04_cfg.c 

OBJS += \
./Core/Lib/HCSR04/HCSR04.o \
./Core/Lib/HCSR04/HCSR04_cfg.o 

C_DEPS += \
./Core/Lib/HCSR04/HCSR04.d \
./Core/Lib/HCSR04/HCSR04_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/HCSR04/%.o: ../Core/Lib/HCSR04/%.c Core/Lib/HCSR04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

