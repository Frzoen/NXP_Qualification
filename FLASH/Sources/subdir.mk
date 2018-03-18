################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/adc.c" \
"../Sources/dma.c" \
"../Sources/gpio.c" \
"../Sources/main.c" \
"../Sources/mcg.c" \
"../Sources/sa_mtb.c" \
"../Sources/tmp.c" \
"../Sources/uart.c" \

C_SRCS += \
../Sources/adc.c \
../Sources/dma.c \
../Sources/gpio.c \
../Sources/main.c \
../Sources/mcg.c \
../Sources/sa_mtb.c \
../Sources/tmp.c \
../Sources/uart.c \

OBJS += \
./Sources/adc.o \
./Sources/dma.o \
./Sources/gpio.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \
./Sources/tmp.o \
./Sources/uart.o \

C_DEPS += \
./Sources/adc.d \
./Sources/dma.d \
./Sources/gpio.d \
./Sources/main.d \
./Sources/mcg.d \
./Sources/sa_mtb.d \
./Sources/tmp.d \
./Sources/uart.d \

OBJS_QUOTED += \
"./Sources/adc.o" \
"./Sources/dma.o" \
"./Sources/gpio.o" \
"./Sources/main.o" \
"./Sources/mcg.o" \
"./Sources/sa_mtb.o" \
"./Sources/tmp.o" \
"./Sources/uart.o" \

C_DEPS_QUOTED += \
"./Sources/adc.d" \
"./Sources/dma.d" \
"./Sources/gpio.d" \
"./Sources/main.d" \
"./Sources/mcg.d" \
"./Sources/sa_mtb.d" \
"./Sources/tmp.d" \
"./Sources/uart.d" \

OBJS_OS_FORMAT += \
./Sources/adc.o \
./Sources/dma.o \
./Sources/gpio.o \
./Sources/main.o \
./Sources/mcg.o \
./Sources/sa_mtb.o \
./Sources/tmp.o \
./Sources/uart.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/adc.o: ../Sources/adc.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/adc.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/adc.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/dma.o: ../Sources/dma.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/dma.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/dma.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/gpio.o: ../Sources/gpio.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/gpio.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/gpio.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/main.o: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/main.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/main.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/mcg.o: ../Sources/mcg.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/mcg.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/mcg.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/sa_mtb.o: ../Sources/sa_mtb.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/sa_mtb.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/sa_mtb.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/tmp.o: ../Sources/tmp.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/tmp.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/tmp.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/uart.o: ../Sources/uart.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/uart.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/uart.o"
	@echo 'Finished building: $<'
	@echo ' '


