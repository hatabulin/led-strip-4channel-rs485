################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/dwt_delay/dwt_stm32_delay.c 

OBJS += \
./Core/Src/dwt_delay/dwt_stm32_delay.o 

C_DEPS += \
./Core/Src/dwt_delay/dwt_stm32_delay.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/dwt_delay/%.o: ../Core/Src/dwt_delay/%.c Core/Src/dwt_delay/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-dwt_delay

clean-Core-2f-Src-2f-dwt_delay:
	-$(RM) ./Core/Src/dwt_delay/dwt_stm32_delay.d ./Core/Src/dwt_delay/dwt_stm32_delay.o

.PHONY: clean-Core-2f-Src-2f-dwt_delay

