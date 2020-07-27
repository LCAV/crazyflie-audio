################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cmsis_fft/arm_bitreversal.c \
../Core/Src/cmsis_fft/arm_cfft_f32.c \
../Core/Src/cmsis_fft/arm_cfft_init_f32.c \
../Core/Src/cmsis_fft/arm_cfft_radix2_f32.c \
../Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.c \
../Core/Src/cmsis_fft/arm_cfft_radix4_f32.c \
../Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.c \
../Core/Src/cmsis_fft/arm_cfft_radix8_f32.c \
../Core/Src/cmsis_fft/arm_cmplx_conj_f32.c \
../Core/Src/cmsis_fft/arm_cmplx_mag_f32.c \
../Core/Src/cmsis_fft/arm_common_tables.c \
../Core/Src/cmsis_fft/arm_const_structs.c \
../Core/Src/cmsis_fft/arm_fft_bin_data.c \
../Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.c \
../Core/Src/cmsis_fft/arm_mat_init_f32.c \
../Core/Src/cmsis_fft/arm_mat_inverse_f32.c \
../Core/Src/cmsis_fft/arm_max_f32.c \
../Core/Src/cmsis_fft/arm_rfft_f32.c \
../Core/Src/cmsis_fft/arm_rfft_fast_f32.c \
../Core/Src/cmsis_fft/arm_rfft_fast_init_f32.c \
../Core/Src/cmsis_fft/arm_rfft_init_f32.c 

S_UPPER_SRCS += \
../Core/Src/cmsis_fft/arm_bitreversal2.S 

OBJS += \
./Core/Src/cmsis_fft/arm_bitreversal.o \
./Core/Src/cmsis_fft/arm_bitreversal2.o \
./Core/Src/cmsis_fft/arm_cfft_f32.o \
./Core/Src/cmsis_fft/arm_cfft_init_f32.o \
./Core/Src/cmsis_fft/arm_cfft_radix2_f32.o \
./Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.o \
./Core/Src/cmsis_fft/arm_cfft_radix4_f32.o \
./Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.o \
./Core/Src/cmsis_fft/arm_cfft_radix8_f32.o \
./Core/Src/cmsis_fft/arm_cmplx_conj_f32.o \
./Core/Src/cmsis_fft/arm_cmplx_mag_f32.o \
./Core/Src/cmsis_fft/arm_common_tables.o \
./Core/Src/cmsis_fft/arm_const_structs.o \
./Core/Src/cmsis_fft/arm_fft_bin_data.o \
./Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.o \
./Core/Src/cmsis_fft/arm_mat_init_f32.o \
./Core/Src/cmsis_fft/arm_mat_inverse_f32.o \
./Core/Src/cmsis_fft/arm_max_f32.o \
./Core/Src/cmsis_fft/arm_rfft_f32.o \
./Core/Src/cmsis_fft/arm_rfft_fast_f32.o \
./Core/Src/cmsis_fft/arm_rfft_fast_init_f32.o \
./Core/Src/cmsis_fft/arm_rfft_init_f32.o 

C_DEPS += \
./Core/Src/cmsis_fft/arm_bitreversal.d \
./Core/Src/cmsis_fft/arm_cfft_f32.d \
./Core/Src/cmsis_fft/arm_cfft_init_f32.d \
./Core/Src/cmsis_fft/arm_cfft_radix2_f32.d \
./Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.d \
./Core/Src/cmsis_fft/arm_cfft_radix4_f32.d \
./Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.d \
./Core/Src/cmsis_fft/arm_cfft_radix8_f32.d \
./Core/Src/cmsis_fft/arm_cmplx_conj_f32.d \
./Core/Src/cmsis_fft/arm_cmplx_mag_f32.d \
./Core/Src/cmsis_fft/arm_common_tables.d \
./Core/Src/cmsis_fft/arm_const_structs.d \
./Core/Src/cmsis_fft/arm_fft_bin_data.d \
./Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.d \
./Core/Src/cmsis_fft/arm_mat_init_f32.d \
./Core/Src/cmsis_fft/arm_mat_inverse_f32.d \
./Core/Src/cmsis_fft/arm_max_f32.d \
./Core/Src/cmsis_fft/arm_rfft_f32.d \
./Core/Src/cmsis_fft/arm_rfft_fast_f32.d \
./Core/Src/cmsis_fft/arm_rfft_fast_init_f32.d \
./Core/Src/cmsis_fft/arm_rfft_init_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/cmsis_fft/arm_bitreversal.o: ../Core/Src/cmsis_fft/arm_bitreversal.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_bitreversal.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/%.o: ../Core/Src/cmsis_fft/%.S
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
Core/Src/cmsis_fft/arm_cfft_f32.o: ../Core/Src/cmsis_fft/arm_cfft_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_init_f32.o: ../Core/Src/cmsis_fft/arm_cfft_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_radix2_f32.o: ../Core/Src/cmsis_fft/arm_cfft_radix2_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_radix2_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.o: ../Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_radix2_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_radix4_f32.o: ../Core/Src/cmsis_fft/arm_cfft_radix4_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_radix4_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.o: ../Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_radix4_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cfft_radix8_f32.o: ../Core/Src/cmsis_fft/arm_cfft_radix8_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cfft_radix8_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cmplx_conj_f32.o: ../Core/Src/cmsis_fft/arm_cmplx_conj_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cmplx_conj_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_cmplx_mag_f32.o: ../Core/Src/cmsis_fft/arm_cmplx_mag_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_cmplx_mag_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_common_tables.o: ../Core/Src/cmsis_fft/arm_common_tables.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_common_tables.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_const_structs.o: ../Core/Src/cmsis_fft/arm_const_structs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_const_structs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_fft_bin_data.o: ../Core/Src/cmsis_fft/arm_fft_bin_data.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_fft_bin_data.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.o: ../Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_mat_cmplx_mult_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_mat_init_f32.o: ../Core/Src/cmsis_fft/arm_mat_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_mat_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_mat_inverse_f32.o: ../Core/Src/cmsis_fft/arm_mat_inverse_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_mat_inverse_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_max_f32.o: ../Core/Src/cmsis_fft/arm_max_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_max_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_rfft_f32.o: ../Core/Src/cmsis_fft/arm_rfft_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_rfft_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_rfft_fast_f32.o: ../Core/Src/cmsis_fft/arm_rfft_fast_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_rfft_fast_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_rfft_fast_init_f32.o: ../Core/Src/cmsis_fft/arm_rfft_fast_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_rfft_fast_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/cmsis_fft/arm_rfft_init_f32.o: ../Core/Src/cmsis_fft/arm_rfft_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -D__FPU_PRESENT -D__FPU_USED -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cmsis_fft/arm_rfft_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

