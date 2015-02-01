################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dsp_lib/arm_pid_init_f32.c \
../Src/dsp_lib/arm_pid_init_q15.c \
../Src/dsp_lib/arm_pid_init_q31.c \
../Src/dsp_lib/arm_pid_reset_f32.c \
../Src/dsp_lib/arm_pid_reset_q15.c \
../Src/dsp_lib/arm_pid_reset_q31.c \
../Src/dsp_lib/arm_sin_cos_f32.c \
../Src/dsp_lib/arm_sin_cos_q31.c 

OBJS += \
./Src/dsp_lib/arm_pid_init_f32.o \
./Src/dsp_lib/arm_pid_init_q15.o \
./Src/dsp_lib/arm_pid_init_q31.o \
./Src/dsp_lib/arm_pid_reset_f32.o \
./Src/dsp_lib/arm_pid_reset_q15.o \
./Src/dsp_lib/arm_pid_reset_q31.o \
./Src/dsp_lib/arm_sin_cos_f32.o \
./Src/dsp_lib/arm_sin_cos_q31.o 

C_DEPS += \
./Src/dsp_lib/arm_pid_init_f32.d \
./Src/dsp_lib/arm_pid_init_q15.d \
./Src/dsp_lib/arm_pid_init_q31.d \
./Src/dsp_lib/arm_pid_reset_f32.d \
./Src/dsp_lib/arm_pid_reset_q15.d \
./Src/dsp_lib/arm_pid_reset_q31.d \
./Src/dsp_lib/arm_sin_cos_f32.d \
./Src/dsp_lib/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Src/dsp_lib/%.o: ../Src/dsp_lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -D__weak=__attribute__((weak)) -DARM_MATH_CM4 -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F405xx -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


