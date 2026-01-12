################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/Comms/ble.cpp \
../Core/Inc/Comms/hc-06.cpp 

OBJS += \
./Core/Inc/Comms/ble.o \
./Core/Inc/Comms/hc-06.o 

CPP_DEPS += \
./Core/Inc/Comms/ble.d \
./Core/Inc/Comms/hc-06.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Comms/%.o Core/Inc/Comms/%.su Core/Inc/Comms/%.cyclo: ../Core/Inc/Comms/%.cpp Core/Inc/Comms/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Comms

clean-Core-2f-Inc-2f-Comms:
	-$(RM) ./Core/Inc/Comms/ble.cyclo ./Core/Inc/Comms/ble.d ./Core/Inc/Comms/ble.o ./Core/Inc/Comms/ble.su ./Core/Inc/Comms/hc-06.cyclo ./Core/Inc/Comms/hc-06.d ./Core/Inc/Comms/hc-06.o ./Core/Inc/Comms/hc-06.su

.PHONY: clean-Core-2f-Inc-2f-Comms

