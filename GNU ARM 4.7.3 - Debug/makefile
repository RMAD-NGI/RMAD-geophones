################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/efm32gg/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RMAD-geophone.axf

# Tool invocations
RMAD-geophone.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -T "RMAD-geophone.ld" -Xlinker --gc-sections -Xlinker -Map="RMAD-geophone.map" --specs=nano.specs -o RMAD-geophone.axf "./src/geophone.o" "./emlib/em_acmp.o" "./emlib/em_adc.o" "./emlib/em_assert.o" "./emlib/em_cmu.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_lcd.o" "./emlib/em_rtc.o" "./emlib/em_system.o" "./Drivers/segmentlcd.o" "./CMSIS/efm32gg/startup_gcc_efm32gg.o" "./CMSIS/efm32gg/system_efm32gg.o" "./BSP/bsp_trace.o" "./src/adjustable_params.o" "./src/config.o" "./src/debug_printf.o" "./src/dust_com.o" "./src/fcs.o" "./src/flash.o" "./src/ngi_rtcdrv.o" "./src/packet.o" "./src/process.o" "./src/timestamp.o" "./src/wait_burtc.o" "./emlib/em_aes.o" "./emlib/em_burtc.o" "./emlib/em_cryotimer.o" "./emlib/em_crypto.o" "./emlib/em_dac.o" "./emlib/em_dbg.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_gpcrc.o" "./emlib/em_i2c.o" "./emlib/em_idac.o" "./emlib/em_int.o" "./emlib/em_ldma.o" "./emlib/em_lesense.o" "./emlib/em_letimer.o" "./emlib/em_leuart.o" "./emlib/em_mpu.o" "./emlib/em_msc.o" "./emlib/em_opamp.o" "./emlib/em_pcnt.o" "./emlib/em_prs.o" "./emlib/em_rmu.o" "./emlib/em_rtcc.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./emlib/em_vcmp.o" "./emlib/em_wdog.o" "./Drivers/dmactrl.o" "./Drivers/nandflash.o" "./Drivers/norflash.o" "./Drivers/retargetio.o" "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: RMAD-geophone.hex'
	arm-none-eabi-objcopy -O ihex "RMAD-geophone.axf" "RMAD-geophone.hex"
	@echo ' '

	@echo 'Building bin file: RMAD-geophone.bin'
	arm-none-eabi-objcopy -O binary "RMAD-geophone.axf" "RMAD-geophone.bin"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "RMAD-geophone.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) RMAD-geophone.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
