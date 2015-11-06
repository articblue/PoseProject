################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../quatadd.c \
../quatconj.c \
../quatdot.c \
../quatnorm.c \
../quatnormsquared.c \
../quatproduct.c \
../quatsubtraction.c 

OBJS += \
./quatadd.o \
./quatconj.o \
./quatdot.o \
./quatnorm.o \
./quatnormsquared.o \
./quatproduct.o \
./quatsubtraction.o 

C_DEPS += \
./quatadd.d \
./quatconj.d \
./quatdot.d \
./quatnorm.d \
./quatnormsquared.d \
./quatproduct.d \
./quatsubtraction.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


