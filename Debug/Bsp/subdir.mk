################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/bsp_can.c \
../Bsp/bsp_key.c \
../Bsp/bsp_pwm.c \
../Bsp/pid.c 

OBJS += \
./Bsp/bsp_can.o \
./Bsp/bsp_key.o \
./Bsp/bsp_pwm.o \
./Bsp/pid.o 

C_DEPS += \
./Bsp/bsp_can.d \
./Bsp/bsp_key.d \
./Bsp/bsp_pwm.d \
./Bsp/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/bsp_can.d ./Bsp/bsp_can.o ./Bsp/bsp_can.su ./Bsp/bsp_key.d ./Bsp/bsp_key.o ./Bsp/bsp_key.su ./Bsp/bsp_pwm.d ./Bsp/bsp_pwm.o ./Bsp/bsp_pwm.su ./Bsp/pid.d ./Bsp/pid.o ./Bsp/pid.su

.PHONY: clean-Bsp

