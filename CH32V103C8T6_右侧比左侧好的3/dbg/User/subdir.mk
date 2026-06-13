################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/GPS.c \
../User/JY901.c \
../User/Laser.c \
../User/Motor.c \
../User/Oled.c \
../User/cone_detect.c \
../User/SR04.c \
../User/Servo.c \
../User/ch32v10x_it.c \
../User/main.c \
../User/rplidar_360.c \
../User/system_ch32v10x.c 

C_DEPS += \
./User/GPS.d \
./User/JY901.d \
./User/Laser.d \
./User/Motor.d \
./User/Oled.d \
./User/cone_detect.d \
./User/SR04.d \
./User/Servo.d \
./User/ch32v10x_it.d \
./User/main.d \
./User/rplidar_360.d \
./User/system_ch32v10x.d 

OBJS += \
./User/GPS.o \
./User/JY901.o \
./User/Laser.o \
./User/Motor.o \
./User/Oled.o \
./User/cone_detect.o \
./User/SR04.o \
./User/Servo.o \
./User/ch32v10x_it.o \
./User/main.o \
./User/rplidar_360.o \
./User/system_ch32v10x.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/46451/Desktop/fpga_car/mcu/CH32V103C8T6/Debug" -I"c:/Users/46451/Desktop/fpga_car/mcu/CH32V103C8T6/Core" -I"c:/Users/46451/Desktop/fpga_car/mcu/CH32V103C8T6/User" -I"c:/Users/46451/Desktop/fpga_car/mcu/CH32V103C8T6/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@

