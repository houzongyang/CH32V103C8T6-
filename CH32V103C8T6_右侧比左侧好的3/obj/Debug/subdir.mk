################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Debug/debug.c 

C_DEPS += \
./Debug/debug.d 

OBJS += \
./Debug/debug.o 

DIR_OBJS += \
./Debug/*.o \

DIR_DEPS += \
./Debug/*.d \

DIR_EXPANDS += \
./Debug/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
Debug/%.o: ../Debug/%.c
	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/13413/Desktop/小车避障成功的/右侧比左侧好的/CH32V103C8T6(3)(1)(1)(1)/CH32V103C8T6(3)(1)/CH32V103C8T6(3)/CH32V103C8T6/Debug" -I"c:/Users/13413/Desktop/小车避障成功的/右侧比左侧好的/CH32V103C8T6(3)(1)(1)(1)/CH32V103C8T6(3)(1)/CH32V103C8T6(3)/CH32V103C8T6/Core" -I"c:/Users/13413/Desktop/小车避障成功的/右侧比左侧好的/CH32V103C8T6(3)(1)(1)(1)/CH32V103C8T6(3)(1)/CH32V103C8T6(3)/CH32V103C8T6/User" -I"c:/Users/13413/Desktop/小车避障成功的/右侧比左侧好的/CH32V103C8T6(3)(1)(1)(1)/CH32V103C8T6(3)(1)/CH32V103C8T6(3)/CH32V103C8T6/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

