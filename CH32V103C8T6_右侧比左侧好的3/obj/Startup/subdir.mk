################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Startup/startup_ch32v10x.S 

S_UPPER_DEPS += \
./Startup/startup_ch32v10x.d 

OBJS += \
./Startup/startup_ch32v10x.o 

DIR_OBJS += \
./Startup/*.o \

DIR_DEPS += \
./Startup/*.d \

DIR_EXPANDS += \
./Startup/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.S
	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -x assembler-with-cpp -I"c:/Users/13413/Desktop/小车避障成功的/右侧比左侧好的/CH32V103C8T6(3)(1)(1)(1)/CH32V103C8T6(3)(1)/CH32V103C8T6(3)/CH32V103C8T6/Startup" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

