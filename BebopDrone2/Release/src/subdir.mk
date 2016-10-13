################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BebopDroneDecodeStream.cpp \
../src/DecoderManager.cpp \
../src/ImageProcessing.cpp \
../src/ihm.cpp 

OBJS += \
./src/BebopDroneDecodeStream.o \
./src/DecoderManager.o \
./src/ImageProcessing.o \
./src/ihm.o 

CPP_DEPS += \
./src/BebopDroneDecodeStream.d \
./src/DecoderManager.d \
./src/ImageProcessing.d \
./src/ihm.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/takuya/arsdk3/out/arsdk-native/staging/usr/include -I/usr/local/include/opencv2 -I/usr/local/include/opencv -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


