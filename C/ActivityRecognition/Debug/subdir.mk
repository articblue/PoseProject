################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MatchPatternToObserved.c \
../ObservedSequence.c 

OBJS += \
./MatchPatternToObserved.o \
./ObservedSequence.o 

C_DEPS += \
./MatchPatternToObserved.d \
./ObservedSequence.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/madisob/PoseProject/C/Quaternion" -O0 -g3 -Wall -c -fmessage-length=0 -lm -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


