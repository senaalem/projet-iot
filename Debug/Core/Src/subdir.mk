################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/aes.c \
../Core/Src/bme68x.c \
../Core/Src/bme68x_necessary_functions.c \
../Core/Src/cayenne_lpp.c \
../Core/Src/debug.c \
../Core/Src/gpio.c \
../Core/Src/hal.c \
../Core/Src/i2c.c \
../Core/Src/lmic.c \
../Core/Src/main.c \
../Core/Src/oslmic.c \
../Core/Src/radio.c \
../Core/Src/spi.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/aes.o \
./Core/Src/bme68x.o \
./Core/Src/bme68x_necessary_functions.o \
./Core/Src/cayenne_lpp.o \
./Core/Src/debug.o \
./Core/Src/gpio.o \
./Core/Src/hal.o \
./Core/Src/i2c.o \
./Core/Src/lmic.o \
./Core/Src/main.o \
./Core/Src/oslmic.o \
./Core/Src/radio.o \
./Core/Src/spi.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/aes.d \
./Core/Src/bme68x.d \
./Core/Src/bme68x_necessary_functions.d \
./Core/Src/cayenne_lpp.d \
./Core/Src/debug.d \
./Core/Src/gpio.d \
./Core/Src/hal.d \
./Core/Src/i2c.d \
./Core/Src/lmic.d \
./Core/Src/main.d \
./Core/Src/oslmic.d \
./Core/Src/radio.d \
./Core/Src/spi.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -DCFG_eu868 -DCFG_sx1276_radio -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/aes.cyclo ./Core/Src/aes.d ./Core/Src/aes.o ./Core/Src/aes.su ./Core/Src/bme68x.cyclo ./Core/Src/bme68x.d ./Core/Src/bme68x.o ./Core/Src/bme68x.su ./Core/Src/bme68x_necessary_functions.cyclo ./Core/Src/bme68x_necessary_functions.d ./Core/Src/bme68x_necessary_functions.o ./Core/Src/bme68x_necessary_functions.su ./Core/Src/cayenne_lpp.cyclo ./Core/Src/cayenne_lpp.d ./Core/Src/cayenne_lpp.o ./Core/Src/cayenne_lpp.su ./Core/Src/debug.cyclo ./Core/Src/debug.d ./Core/Src/debug.o ./Core/Src/debug.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hal.cyclo ./Core/Src/hal.d ./Core/Src/hal.o ./Core/Src/hal.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/lmic.cyclo ./Core/Src/lmic.d ./Core/Src/lmic.o ./Core/Src/lmic.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/oslmic.cyclo ./Core/Src/oslmic.d ./Core/Src/oslmic.o ./Core/Src/oslmic.su ./Core/Src/radio.cyclo ./Core/Src/radio.d ./Core/Src/radio.o ./Core/Src/radio.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

