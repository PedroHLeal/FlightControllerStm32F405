################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Calcs/estimations.cpp \
../Core/Src/Calcs/filters.cpp 

OBJS += \
./Core/Src/Calcs/estimations.o \
./Core/Src/Calcs/filters.o 

CPP_DEPS += \
./Core/Src/Calcs/estimations.d \
./Core/Src/Calcs/filters.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Calcs/%.o Core/Src/Calcs/%.su Core/Src/Calcs/%.cyclo: ../Core/Src/Calcs/%.cpp Core/Src/Calcs/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Calcs

clean-Core-2f-Src-2f-Calcs:
	-$(RM) ./Core/Src/Calcs/estimations.cyclo ./Core/Src/Calcs/estimations.d ./Core/Src/Calcs/estimations.o ./Core/Src/Calcs/estimations.su ./Core/Src/Calcs/filters.cyclo ./Core/Src/Calcs/filters.d ./Core/Src/Calcs/filters.o ./Core/Src/Calcs/filters.su

.PHONY: clean-Core-2f-Src-2f-Calcs

