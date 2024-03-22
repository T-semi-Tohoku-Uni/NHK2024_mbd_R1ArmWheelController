################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/NHK2024_Library/src/clip_number.c \
../Core/NHK2024_Library/src/filter.c \
../Core/NHK2024_Library/src/i_pd.c \
../Core/NHK2024_Library/src/pid.c 

OBJS += \
./Core/NHK2024_Library/src/clip_number.o \
./Core/NHK2024_Library/src/filter.o \
./Core/NHK2024_Library/src/i_pd.o \
./Core/NHK2024_Library/src/pid.o 

C_DEPS += \
./Core/NHK2024_Library/src/clip_number.d \
./Core/NHK2024_Library/src/filter.d \
./Core/NHK2024_Library/src/i_pd.d \
./Core/NHK2024_Library/src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/NHK2024_Library/src/%.o Core/NHK2024_Library/src/%.su Core/NHK2024_Library/src/%.cyclo: ../Core/NHK2024_Library/src/%.c Core/NHK2024_Library/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/Users/shibatakeigo/Development/t-semi/NHK2024_mbd_R1ArmWheelController/Core/NHK2024_Library/src" -I"/Users/shibatakeigo/Development/t-semi/NHK2024_mbd_R1ArmWheelController/Core/NHK2024_R1CANIDList" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-NHK2024_Library-2f-src

clean-Core-2f-NHK2024_Library-2f-src:
	-$(RM) ./Core/NHK2024_Library/src/clip_number.cyclo ./Core/NHK2024_Library/src/clip_number.d ./Core/NHK2024_Library/src/clip_number.o ./Core/NHK2024_Library/src/clip_number.su ./Core/NHK2024_Library/src/filter.cyclo ./Core/NHK2024_Library/src/filter.d ./Core/NHK2024_Library/src/filter.o ./Core/NHK2024_Library/src/filter.su ./Core/NHK2024_Library/src/i_pd.cyclo ./Core/NHK2024_Library/src/i_pd.d ./Core/NHK2024_Library/src/i_pd.o ./Core/NHK2024_Library/src/i_pd.su ./Core/NHK2024_Library/src/pid.cyclo ./Core/NHK2024_Library/src/pid.d ./Core/NHK2024_Library/src/pid.o ./Core/NHK2024_Library/src/pid.su

.PHONY: clean-Core-2f-NHK2024_Library-2f-src

