################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/controls/straight_controls/straight_controls.c 

OBJS += \
./Src/controls/straight_controls/straight_controls.o 

C_DEPS += \
./Src/controls/straight_controls/straight_controls.d 


# Each subdirectory must supply rules for building sources it contributes
Src/controls/straight_controls/%.o: ../Src/controls/straight_controls/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -D__weak=__attribute__((weak)) -DARM_MATH_CM4 -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F405xx -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


