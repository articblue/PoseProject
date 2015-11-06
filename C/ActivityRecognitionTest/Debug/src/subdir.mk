################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ActivityRecognitionTest.c \
../src/SimpleTest.c 

OBJS += \
./src/ActivityRecognitionTest.o \
./src/SimpleTest.o 

C_DEPS += \
./src/ActivityRecognitionTest.d \
./src/SimpleTest.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/madisob/PoseProject/C/Quaternion" -I"/home/madisob/PoseProject/C/ActivityRecognition" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


