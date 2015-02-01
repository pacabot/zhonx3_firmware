################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/util/subdir.mk
-include Src/times_base/subdir.mk
-include Src/peripherals/tests/subdir.mk
-include Src/peripherals/telemeters/subdir.mk
-include Src/peripherals/multimeter/subdir.mk
-include Src/peripherals/lineSensors/subdir.mk
-include Src/peripherals/gyroscope/subdir.mk
-include Src/peripherals/expander/subdir.mk
-include Src/peripherals/encoders/subdir.mk
-include Src/peripherals/display/subdir.mk
-include Src/peripherals/bluetooth/subdir.mk
-include Src/dsp_lib/subdir.mk
-include Src/controls/straight_controls/subdir.mk
-include Src/controls/pid/subdir.mk
-include Src/callback/subdir.mk
-include Src/adcManagment/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/subdir.mk
-include Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
ZHONX_III.bin \

SECONDARY_SIZE += \
ZHONX_III.siz \


# All Target
all: ZHONX_III.elf secondary-outputs

# Tool invocations
ZHONX_III.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -T "" -Xlinker --gc-sections -Wl,-Map,"ZHONX_III.map" -Wl,-cref,-u,Reset_Handler -o "ZHONX_III.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

ZHONX_III.bin: ZHONX_III.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary "ZHONX_III.elf"  "ZHONX_III.bin"
	@echo 'Finished building: $@'
	@echo ' '

ZHONX_III.siz: ZHONX_III.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "ZHONX_III.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(S_DEPS)$(S_UPPER_DEPS) ZHONX_III.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
