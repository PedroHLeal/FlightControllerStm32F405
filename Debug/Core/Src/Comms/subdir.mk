################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Comms/ble.cpp \
../Core/Src/Comms/hc-06.cpp 

OBJS += \
./Core/Src/Comms/ble.o \
./Core/Src/Comms/hc-06.o 

CPP_DEPS += \
./Core/Src/Comms/ble.d \
./Core/Src/Comms/hc-06.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Comms/%.o Core/Src/Comms/%.su Core/Src/Comms/%.cyclo: ../Core/Src/Comms/%.cpp Core/Src/Comms/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Comms

clean-Core-2f-Src-2f-Comms:
	-$(RM) ./Core/Src/Comms/ble.cyclo ./Core/Src/Comms/ble.d ./Core/Src/Comms/ble.o ./Core/Src/Comms/ble.su ./Core/Src/Comms/hc-06.cyclo ./Core/Src/Comms/hc-06.d ./Core/Src/Comms/hc-06.o ./Core/Src/Comms/hc-06.su

.PHONY: clean-Core-2f-Src-2f-Comms

