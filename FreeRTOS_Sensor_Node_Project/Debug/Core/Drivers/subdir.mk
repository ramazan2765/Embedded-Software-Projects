################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/filter.c \
../Core/Drivers/mpu9250.c \
../Core/Drivers/packet.c \
../Core/Drivers/rtc.c \
../Core/Drivers/test_step.c \
../Core/Drivers/tmp102.c 

OBJS += \
./Core/Drivers/filter.o \
./Core/Drivers/mpu9250.o \
./Core/Drivers/packet.o \
./Core/Drivers/rtc.o \
./Core/Drivers/test_step.o \
./Core/Drivers/tmp102.o 

C_DEPS += \
./Core/Drivers/filter.d \
./Core/Drivers/mpu9250.d \
./Core/Drivers/packet.d \
./Core/Drivers/rtc.d \
./Core/Drivers/test_step.d \
./Core/Drivers/tmp102.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/%.o Core/Drivers/%.su Core/Drivers/%.cyclo: ../Core/Drivers/%.c Core/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Drivers -I../Core/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Drivers

clean-Core-2f-Drivers:
	-$(RM) ./Core/Drivers/filter.cyclo ./Core/Drivers/filter.d ./Core/Drivers/filter.o ./Core/Drivers/filter.su ./Core/Drivers/mpu9250.cyclo ./Core/Drivers/mpu9250.d ./Core/Drivers/mpu9250.o ./Core/Drivers/mpu9250.su ./Core/Drivers/packet.cyclo ./Core/Drivers/packet.d ./Core/Drivers/packet.o ./Core/Drivers/packet.su ./Core/Drivers/rtc.cyclo ./Core/Drivers/rtc.d ./Core/Drivers/rtc.o ./Core/Drivers/rtc.su ./Core/Drivers/test_step.cyclo ./Core/Drivers/test_step.d ./Core/Drivers/test_step.o ./Core/Drivers/test_step.su ./Core/Drivers/tmp102.cyclo ./Core/Drivers/tmp102.d ./Core/Drivers/tmp102.o ./Core/Drivers/tmp102.su

.PHONY: clean-Core-2f-Drivers

