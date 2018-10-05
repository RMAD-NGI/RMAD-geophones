################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.c \
P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./CMSIS/ControllerFunctions/arm_pid_init_f32.o \
./CMSIS/ControllerFunctions/arm_pid_init_q15.o \
./CMSIS/ControllerFunctions/arm_pid_init_q31.o \
./CMSIS/ControllerFunctions/arm_pid_reset_f32.o \
./CMSIS/ControllerFunctions/arm_pid_reset_q15.o \
./CMSIS/ControllerFunctions/arm_pid_reset_q31.o \
./CMSIS/ControllerFunctions/arm_sin_cos_f32.o \
./CMSIS/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./CMSIS/ControllerFunctions/arm_pid_init_f32.d \
./CMSIS/ControllerFunctions/arm_pid_init_q15.d \
./CMSIS/ControllerFunctions/arm_pid_init_q31.d \
./CMSIS/ControllerFunctions/arm_pid_reset_f32.d \
./CMSIS/ControllerFunctions/arm_pid_reset_q15.d \
./CMSIS/ControllerFunctions/arm_pid_reset_q31.d \
./CMSIS/ControllerFunctions/arm_sin_cos_f32.d \
./CMSIS/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/ControllerFunctions/arm_pid_init_f32.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_f32.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_init_f32.d" -MT"CMSIS/ControllerFunctions/arm_pid_init_f32.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_pid_init_q15.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_q15.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_init_q15.d" -MT"CMSIS/ControllerFunctions/arm_pid_init_q15.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_pid_init_q31.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_init_q31.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_init_q31.d" -MT"CMSIS/ControllerFunctions/arm_pid_init_q31.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_pid_reset_f32.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_f32.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_reset_f32.d" -MT"CMSIS/ControllerFunctions/arm_pid_reset_f32.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_pid_reset_q15.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q15.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_reset_q15.d" -MT"CMSIS/ControllerFunctions/arm_pid_reset_q15.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_pid_reset_q31.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_pid_reset_q31.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_pid_reset_q31.d" -MT"CMSIS/ControllerFunctions/arm_pid_reset_q31.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_sin_cos_f32.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_sin_cos_f32.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_sin_cos_f32.d" -MT"CMSIS/ControllerFunctions/arm_sin_cos_f32.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/ControllerFunctions/arm_sin_cos_q31.o: P:/2014/00/20140081/Design/Software/Inge/CMSIS-SP-00300-r4p1-00rel0/CMSIS/DSP_Lib/Source/ControllerFunctions/arm_sin_cos_q31.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb '-DEFM32GG990F1024=1' '-DDEBUG=1' '-DARM_MATH_CM3=' -I"P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/examples/inttemp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/CMSIS/Include" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/emlib/inc" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/bsp" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/common/drivers" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/kits/EFM32GG_STK3700/config" -I"C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2/Device/SiliconLabs/EFM32GG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"CMSIS/ControllerFunctions/arm_sin_cos_q31.d" -MT"CMSIS/ControllerFunctions/arm_sin_cos_q31.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


