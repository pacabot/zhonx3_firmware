################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/peripherals/display/menu.c \
../Src/peripherals/display/smallfonts.c \
../Src/peripherals/display/ssd1306.c 

OBJS += \
./Src/peripherals/display/menu.o \
./Src/peripherals/display/smallfonts.o \
./Src/peripherals/display/ssd1306.o 

C_DEPS += \
./Src/peripherals/display/menu.d \
./Src/peripherals/display/smallfonts.d \
./Src/peripherals/display/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Src/peripherals/display/%.o: ../Src/peripherals/display/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -D__weak=__attribute__((weak)) -DARM_MATH_CM4 -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F405xx -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


