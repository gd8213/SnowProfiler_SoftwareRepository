################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_audio.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_bus.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_env_sensors.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_env_sensors_ex.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_gg.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_motion_sensors.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_motion_sensors_ex.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_sd.c \
C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/hci_tl_interface_template.c 

OBJS += \
./SensorTile/SensorTile.o \
./SensorTile/SensorTile_audio.o \
./SensorTile/SensorTile_bus.o \
./SensorTile/SensorTile_env_sensors.o \
./SensorTile/SensorTile_env_sensors_ex.o \
./SensorTile/SensorTile_gg.o \
./SensorTile/SensorTile_motion_sensors.o \
./SensorTile/SensorTile_motion_sensors_ex.o \
./SensorTile/SensorTile_sd.o \
./SensorTile/hci_tl_interface_template.o 

C_DEPS += \
./SensorTile/SensorTile.d \
./SensorTile/SensorTile_audio.d \
./SensorTile/SensorTile_bus.d \
./SensorTile/SensorTile_env_sensors.d \
./SensorTile/SensorTile_env_sensors_ex.d \
./SensorTile/SensorTile_gg.d \
./SensorTile/SensorTile_motion_sensors.d \
./SensorTile/SensorTile_motion_sensors_ex.d \
./SensorTile/SensorTile_sd.d \
./SensorTile/hci_tl_interface_template.d 


# Each subdirectory must supply rules for building sources it contributes
SensorTile/SensorTile.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_audio.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_audio.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_bus.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_bus.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_env_sensors.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_env_sensors.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_env_sensors_ex.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_env_sensors_ex.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_gg.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_gg.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_motion_sensors.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_motion_sensors.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_motion_sensors_ex.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_motion_sensors_ex.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/SensorTile_sd.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/SensorTile_sd.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SensorTile/hci_tl_interface_template.o: C:/Users/Felbermayr\ Simon/OneDrive\ -\ mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile/hci_tl_interface_template.c SensorTile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/SensorTile" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/lsm6dsm" -I"C:/Users/Felbermayr Simon/OneDrive - mci4me.at/01_Master/3.Semester/Project_2/SensorTile/STSW-STLKT01_V2.5.0_1/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

