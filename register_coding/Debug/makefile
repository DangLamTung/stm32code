################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
register_coding.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
register_coding.list \

OBJCOPY_BIN += \
register_coding.bin \


# All Target
all: register_coding.elf secondary-outputs

# Tool invocations
register_coding.elf: $(OBJS) $(USER_OBJS) /home/tung/STM32CubeIDE/workspace_1.2.0/register_coding/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "register_coding.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/tung/STM32CubeIDE/workspace_1.2.0/register_coding/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="register_coding.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

register_coding.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "register_coding.list"
	@echo 'Finished building: $@'
	@echo ' '

register_coding.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "register_coding.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
