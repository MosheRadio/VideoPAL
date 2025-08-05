################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/gdi.c \
../Core/Src/gdi_vt100.c \
../Core/Src/main.c \
../Core/Src/show.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/video.c \
../Core/Src/vt100.c \
../Core/Src/vt100_moshe.c 

OBJS += \
./Core/Src/gdi.o \
./Core/Src/gdi_vt100.o \
./Core/Src/main.o \
./Core/Src/show.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/video.o \
./Core/Src/vt100.o \
./Core/Src/vt100_moshe.o 

C_DEPS += \
./Core/Src/gdi.d \
./Core/Src/gdi_vt100.d \
./Core/Src/main.d \
./Core/Src/show.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/video.d \
./Core/Src/vt100.d \
./Core/Src/vt100_moshe.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/gdi.cyclo ./Core/Src/gdi.d ./Core/Src/gdi.o ./Core/Src/gdi.su ./Core/Src/gdi_vt100.cyclo ./Core/Src/gdi_vt100.d ./Core/Src/gdi_vt100.o ./Core/Src/gdi_vt100.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/show.cyclo ./Core/Src/show.d ./Core/Src/show.o ./Core/Src/show.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/video.cyclo ./Core/Src/video.d ./Core/Src/video.o ./Core/Src/video.su ./Core/Src/vt100.cyclo ./Core/Src/vt100.d ./Core/Src/vt100.o ./Core/Src/vt100.su ./Core/Src/vt100_moshe.cyclo ./Core/Src/vt100_moshe.d ./Core/Src/vt100_moshe.o ./Core/Src/vt100_moshe.su

.PHONY: clean-Core-2f-Src

