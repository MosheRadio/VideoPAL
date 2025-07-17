################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/font6x10.c 

OBJS += \
./Core/Inc/font6x10.o 

C_DEPS += \
./Core/Inc/font6x10.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I/Core/Inc -I"/home/sason/STM32CubeIDE/workspace_1.19.0/VideoStm" -I/Drivers/STM32G4xx_HAL_Driver/Inc -I/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I/Drivers/CMSIS/Device/ST/STM32G4xx/Include -I"/home/sason/STM32CubeIDE/workspace_1.19.0/VideoStm"/Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/font6x10.cyclo ./Core/Inc/font6x10.d ./Core/Inc/font6x10.o ./Core/Inc/font6x10.su

.PHONY: clean-Core-2f-Inc

