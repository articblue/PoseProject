################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../XMLHumanActivityRead.c \
../XMLHumanPoseRead.c 

OBJS += \
./XMLHumanActivityRead.o \
./XMLHumanPoseRead.o 

C_DEPS += \
./XMLHumanActivityRead.d \
./XMLHumanPoseRead.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/madisob/PoseProject/C/Human" -I"/home/madisob/PoseProject/C/Quaternion" -I"/home/madisob/PoseProject/C/ActivityRecognition" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


