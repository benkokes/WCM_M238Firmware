################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../bh1750.c \
../bmp180.c \
../eeprom.c \
../i2c_core.c \
../KXCJ9.c \
../sharplcd.c \
../sht21.c \
../spi.c \
../time.c \
../twimaster.c \
../uart.c \
../WineCellarMonitor.c \
../_StackInstrument.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
bh1750.o \
bmp180.o \
eeprom.o \
i2c_core.o \
KXCJ9.o \
sharplcd.o \
sht21.o \
spi.o \
time.o \
twimaster.o \
uart.o \
WineCellarMonitor.o \
_StackInstrument.o

OBJS_AS_ARGS +=  \
bh1750.o \
bmp180.o \
eeprom.o \
i2c_core.o \
KXCJ9.o \
sharplcd.o \
sht21.o \
spi.o \
time.o \
twimaster.o \
uart.o \
WineCellarMonitor.o \
_StackInstrument.o

C_DEPS +=  \
bh1750.d \
bmp180.d \
eeprom.d \
i2c_core.d \
KXCJ9.d \
sharplcd.d \
sht21.d \
spi.d \
time.d \
twimaster.d \
uart.d \
WineCellarMonitor.d \
_StackInstrument.d

C_DEPS_AS_ARGS +=  \
bh1750.d \
bmp180.d \
eeprom.d \
i2c_core.d \
KXCJ9.d \
sharplcd.d \
sht21.d \
spi.d \
time.d \
twimaster.d \
uart.d \
WineCellarMonitor.d \
_StackInstrument.d

OUTPUT_FILE_PATH +=WineCellarMonitor.elf

OUTPUT_FILE_PATH_AS_ARGS +=WineCellarMonitor.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler



























./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG -DF_CPU=8000000UL  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -Wextra -Wundef -mmcu=atmega328 -c -std=gnu99 -v -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="WineCellarMonitor.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega328  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "WineCellarMonitor.elf" "WineCellarMonitor.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "WineCellarMonitor.elf" "WineCellarMonitor.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "WineCellarMonitor.elf" > "WineCellarMonitor.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "WineCellarMonitor.elf" "WineCellarMonitor.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1061\avr8-gnu-toolchain\bin\avr-size.exe" "WineCellarMonitor.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "WineCellarMonitor.elf" "WineCellarMonitor.a" "WineCellarMonitor.hex" "WineCellarMonitor.lss" "WineCellarMonitor.eep" "WineCellarMonitor.map" "WineCellarMonitor.srec" "WineCellarMonitor.usersignatures"
	