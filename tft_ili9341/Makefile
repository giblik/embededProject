PROJECT_NAME := radio_test
export C_INCLUDE_PATH = inc

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

DEVICE     := atmega328p
CLOCK      := 16000000
PROGRAMMER := -carduino -P/dev/ttyUSB0
SRC_DIR	   := src/
INC_DIR    := C_INCLUDE_PATH
OBJ_DIR    := build/
SIZE       := avr-size --format=avr --mcu=$(DEVICE)
OBJCOPY    := avr-objcopy -j .text -j .data
#Команда запуска avrdude. Ее нужно скопировать из Arduino IDE.
#AVRDUDE = avrdude -C/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf -carduino -P/dev/tty.usbserial-A600dAAQ -b19200 -D -p atmega168
AVRDUDE = avrdude  -F -v -pm328 $(PROGRAMMER)

MK := mkdir
RM := rm -rf
NO_ECHO := @

#source common to all targets
C_SOURCE_FILES += \
$(abspath tft_display/ili9341_ll.c) \
$(abspath main.c) \
$(abspath ../avr_driver/serial/serial.c) \

#includes common to all targets
INC_PATHS  = -I$(C_INCLUDE_PATH)
INC_PATHS += -I$(abspath ./tft_display)
INC_PATHS += -I$(abspath ../avr_driver)
INC_PATHS += -I$(abspath ../avr_driver/serial)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

CFLAGS  = -DARDUINO_BOARD
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -DF_CPU=$(CLOCK)

LDFLAGS += -Wl,-Map,$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).map

COMPILE = avr-gcc -mmcu=$(DEVICE)

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(dir $(C_SOURCE_FILES) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

vpath %.c $(C_PATHS)

OBJECTS = $(C_OBJECTS)

all: clean
	$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e main

OUTPUT_FILENAME := $(PROJECT_NAME)

main: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(COMPILE) $(LDFLAGS) $(INC_PATHS) $(OBJECTS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	@echo $(MAKEFILE_NAME)
	$(MK) $@

#Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(COMPILE) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex:
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

clean:
	$(RM) $(BUILD_DIRECTORIES)

flash:
	$(AVRDUDE) -U flash:w:$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex:i
