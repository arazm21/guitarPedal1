################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Atan_Overdrive.cpp \
../Core/Src/CF.cpp \
../Core/Src/Delay.cpp \
../Core/Src/Delay2.cpp \
../Core/Src/EQ_High_Shelving.cpp \
../Core/Src/EQ_Low_Butter.cpp \
../Core/Src/EQ_Low_Shelving.cpp \
../Core/Src/EQ_Peaking.cpp \
../Core/Src/Exp_Distortion.cpp \
../Core/Src/First_Order_High_Pass.cpp \
../Core/Src/MOD_Flanger_1.cpp \
../Core/Src/MOD_Tremolo.cpp \
../Core/Src/Tremolo.cpp \
../Core/Src/cubic_overdrive.cpp \
../Core/Src/frequencyChanger.cpp \
../Core/Src/main.cpp \
../Core/Src/my_distortion.cpp \
../Core/Src/reverb.cpp 

C_SRCS += \
../Core/Src/ee24.c \
../Core/Src/freertos.c \
../Core/Src/ssd1306.c \
../Core/Src/ssd1306_fonts.c \
../Core/Src/ssd1306_tests.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/ee24.d \
./Core/Src/freertos.d \
./Core/Src/ssd1306.d \
./Core/Src/ssd1306_fonts.d \
./Core/Src/ssd1306_tests.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/Atan_Overdrive.o \
./Core/Src/CF.o \
./Core/Src/Delay.o \
./Core/Src/Delay2.o \
./Core/Src/EQ_High_Shelving.o \
./Core/Src/EQ_Low_Butter.o \
./Core/Src/EQ_Low_Shelving.o \
./Core/Src/EQ_Peaking.o \
./Core/Src/Exp_Distortion.o \
./Core/Src/First_Order_High_Pass.o \
./Core/Src/MOD_Flanger_1.o \
./Core/Src/MOD_Tremolo.o \
./Core/Src/Tremolo.o \
./Core/Src/cubic_overdrive.o \
./Core/Src/ee24.o \
./Core/Src/freertos.o \
./Core/Src/frequencyChanger.o \
./Core/Src/main.o \
./Core/Src/my_distortion.o \
./Core/Src/reverb.o \
./Core/Src/ssd1306.o \
./Core/Src/ssd1306_fonts.o \
./Core/Src/ssd1306_tests.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

CPP_DEPS += \
./Core/Src/Atan_Overdrive.d \
./Core/Src/CF.d \
./Core/Src/Delay.d \
./Core/Src/Delay2.d \
./Core/Src/EQ_High_Shelving.d \
./Core/Src/EQ_Low_Butter.d \
./Core/Src/EQ_Low_Shelving.d \
./Core/Src/EQ_Peaking.d \
./Core/Src/Exp_Distortion.d \
./Core/Src/First_Order_High_Pass.d \
./Core/Src/MOD_Flanger_1.d \
./Core/Src/MOD_Tremolo.d \
./Core/Src/Tremolo.d \
./Core/Src/cubic_overdrive.d \
./Core/Src/frequencyChanger.d \
./Core/Src/main.d \
./Core/Src/my_distortion.d \
./Core/Src/reverb.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Atan_Overdrive.cyclo ./Core/Src/Atan_Overdrive.d ./Core/Src/Atan_Overdrive.o ./Core/Src/Atan_Overdrive.su ./Core/Src/CF.cyclo ./Core/Src/CF.d ./Core/Src/CF.o ./Core/Src/CF.su ./Core/Src/Delay.cyclo ./Core/Src/Delay.d ./Core/Src/Delay.o ./Core/Src/Delay.su ./Core/Src/Delay2.cyclo ./Core/Src/Delay2.d ./Core/Src/Delay2.o ./Core/Src/Delay2.su ./Core/Src/EQ_High_Shelving.cyclo ./Core/Src/EQ_High_Shelving.d ./Core/Src/EQ_High_Shelving.o ./Core/Src/EQ_High_Shelving.su ./Core/Src/EQ_Low_Butter.cyclo ./Core/Src/EQ_Low_Butter.d ./Core/Src/EQ_Low_Butter.o ./Core/Src/EQ_Low_Butter.su ./Core/Src/EQ_Low_Shelving.cyclo ./Core/Src/EQ_Low_Shelving.d ./Core/Src/EQ_Low_Shelving.o ./Core/Src/EQ_Low_Shelving.su ./Core/Src/EQ_Peaking.cyclo ./Core/Src/EQ_Peaking.d ./Core/Src/EQ_Peaking.o ./Core/Src/EQ_Peaking.su ./Core/Src/Exp_Distortion.cyclo ./Core/Src/Exp_Distortion.d ./Core/Src/Exp_Distortion.o ./Core/Src/Exp_Distortion.su ./Core/Src/First_Order_High_Pass.cyclo ./Core/Src/First_Order_High_Pass.d ./Core/Src/First_Order_High_Pass.o ./Core/Src/First_Order_High_Pass.su ./Core/Src/MOD_Flanger_1.cyclo ./Core/Src/MOD_Flanger_1.d ./Core/Src/MOD_Flanger_1.o ./Core/Src/MOD_Flanger_1.su ./Core/Src/MOD_Tremolo.cyclo ./Core/Src/MOD_Tremolo.d ./Core/Src/MOD_Tremolo.o ./Core/Src/MOD_Tremolo.su ./Core/Src/Tremolo.cyclo ./Core/Src/Tremolo.d ./Core/Src/Tremolo.o ./Core/Src/Tremolo.su ./Core/Src/cubic_overdrive.cyclo ./Core/Src/cubic_overdrive.d ./Core/Src/cubic_overdrive.o ./Core/Src/cubic_overdrive.su ./Core/Src/ee24.cyclo ./Core/Src/ee24.d ./Core/Src/ee24.o ./Core/Src/ee24.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/frequencyChanger.cyclo ./Core/Src/frequencyChanger.d ./Core/Src/frequencyChanger.o ./Core/Src/frequencyChanger.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/my_distortion.cyclo ./Core/Src/my_distortion.d ./Core/Src/my_distortion.o ./Core/Src/my_distortion.su ./Core/Src/reverb.cyclo ./Core/Src/reverb.d ./Core/Src/reverb.o ./Core/Src/reverb.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/ssd1306_fonts.cyclo ./Core/Src/ssd1306_fonts.d ./Core/Src/ssd1306_fonts.o ./Core/Src/ssd1306_fonts.su ./Core/Src/ssd1306_tests.cyclo ./Core/Src/ssd1306_tests.d ./Core/Src/ssd1306_tests.o ./Core/Src/ssd1306_tests.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

