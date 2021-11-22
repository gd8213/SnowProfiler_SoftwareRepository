################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.c 

OBJS += \
./STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.o 

C_DEPS += \
./STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.c STM32L4xx_HAL_Driver/Src/Legacy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

