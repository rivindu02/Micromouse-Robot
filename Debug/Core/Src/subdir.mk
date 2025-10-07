################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/audio.c \
../Core/Src/communication.c \
../Core/Src/gyro.c \
../Core/Src/logging_tests.c \
../Core/Src/main.c \
../Core/Src/micromouse.c \
../Core/Src/movement.c \
../Core/Src/sensors.c \
../Core/Src/speed_run.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/utils.c \
../Core/Src/wall_handle.c 

OBJS += \
./Core/Src/audio.o \
./Core/Src/communication.o \
./Core/Src/gyro.o \
./Core/Src/logging_tests.o \
./Core/Src/main.o \
./Core/Src/micromouse.o \
./Core/Src/movement.o \
./Core/Src/sensors.o \
./Core/Src/speed_run.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/utils.o \
./Core/Src/wall_handle.o 

C_DEPS += \
./Core/Src/audio.d \
./Core/Src/communication.d \
./Core/Src/gyro.d \
./Core/Src/logging_tests.d \
./Core/Src/main.d \
./Core/Src/micromouse.d \
./Core/Src/movement.d \
./Core/Src/sensors.d \
./Core/Src/speed_run.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/utils.d \
./Core/Src/wall_handle.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/audio.cyclo ./Core/Src/audio.d ./Core/Src/audio.o ./Core/Src/audio.su ./Core/Src/communication.cyclo ./Core/Src/communication.d ./Core/Src/communication.o ./Core/Src/communication.su ./Core/Src/gyro.cyclo ./Core/Src/gyro.d ./Core/Src/gyro.o ./Core/Src/gyro.su ./Core/Src/logging_tests.cyclo ./Core/Src/logging_tests.d ./Core/Src/logging_tests.o ./Core/Src/logging_tests.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/micromouse.cyclo ./Core/Src/micromouse.d ./Core/Src/micromouse.o ./Core/Src/micromouse.su ./Core/Src/movement.cyclo ./Core/Src/movement.d ./Core/Src/movement.o ./Core/Src/movement.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/speed_run.cyclo ./Core/Src/speed_run.d ./Core/Src/speed_run.o ./Core/Src/speed_run.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su ./Core/Src/wall_handle.cyclo ./Core/Src/wall_handle.d ./Core/Src/wall_handle.o ./Core/Src/wall_handle.su

.PHONY: clean-Core-2f-Src

