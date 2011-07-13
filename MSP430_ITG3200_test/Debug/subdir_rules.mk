################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
IMU_main.obj: ../IMU_main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/ccs4/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=msp -g --define=__MSP430F2274__ --include_path="C:/ccs4/ccsv4/msp430/include" --include_path="C:/ccs4/ccsv4/tools/compiler/msp430/include" --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="IMU_main.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

TI_USCI_I2C_master.obj: ../TI_USCI_I2C_master.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/ccs4/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=msp -g --define=__MSP430F2274__ --include_path="C:/ccs4/ccsv4/msp430/include" --include_path="C:/ccs4/ccsv4/tools/compiler/msp430/include" --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="TI_USCI_I2C_master.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

basic_utils.obj: ../basic_utils.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/ccs4/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=msp -g --define=__MSP430F2274__ --include_path="C:/ccs4/ccsv4/msp430/include" --include_path="C:/ccs4/ccsv4/tools/compiler/msp430/include" --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="basic_utils.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '

printf.obj: ../printf.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/ccs4/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=msp -g --define=__MSP430F2274__ --include_path="C:/ccs4/ccsv4/msp430/include" --include_path="C:/ccs4/ccsv4/tools/compiler/msp430/include" --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="printf.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '


