################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
CH32V103C8T6.hex \

SECONDARY_LIST += \
CH32V103C8T6.lst \

SECONDARY_SIZE += \
CH32V103C8T6.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: CH32V103C8T6.elf secondary-outputs

# Tool invocations
CH32V103C8T6.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "c:\Users\46451\Desktop\fpga_car\mcu\CH32V103C8T6\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"CH32V103C8T6.map" --specs=nano.specs --specs=nosys.specs -o "CH32V103C8T6.elf" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@
CH32V103C8T6.hex: CH32V103C8T6.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	@	riscv-none-embed-objcopy -O ihex "CH32V103C8T6.elf" "CH32V103C8T6.hex"
	@echo 'Finished building $@'
	@
CH32V103C8T6.lst: CH32V103C8T6.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "CH32V103C8T6.elf" > "CH32V103C8T6.lst"
	@echo 'Finished building $@'
	@
CH32V103C8T6.siz: CH32V103C8T6.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "CH32V103C8T6.elf"
	@echo 'Finished building $@'
	@

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) CH32V103C8T6.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets