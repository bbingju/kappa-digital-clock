PROJECT_DIR = $(PWD)

# on the Linux
# ARDUINO_DIR   = /usr/share/arduino
# ARDMK_DIR     = /usr/share/arduino
# AVR_TOOLS_DIR = /usr
# ARDUINO_PORT = /dev/ttyUSB0

# on the Mac
#ARDUINO_DIR = $(HOME)/Library/Arduino15/packages/arduino/hardware/avr/1.8.1
ARDUINO_DIR = $(HOME)/work/MegaCore/avr
#ARDUINO_CORE_PATH = $(HOME)/Library/Arduino15/packages/arduino/hardware/avr/1.8.1
#ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/arduino/hardware/avr/1.8.1
ALTERNATE_CORE_PATH = $(HOME)/work/MegaCore/avr
ARDMK_DIR   = /usr/local/opt/arduino-mk
#AVR_TOOLS_DIR = $(HOME)/Library/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino5
AVR_TOOLS_DIR = /usr/local
AVRDUDE_CONF  = $(ARDUINO_DIR)/avrdude.conf
ARDUINO_PORT = /dev/tty.SLAB_USBtoUART
AVRDUDE_ARD_BAUDRATE = 38400

BOARD_TAG = 64
F_CPU = 8000000L

USER_LIB_PATH += $(PROJECT_DIR)/lib
ARDUINO_LIBS  += Wire \
	SoftwareSerial \
	Adafruit_GPS_Library \
	Rtc \
	lunar_cal \
	HT16K33 \
	KappaSegment \
	Button

CURRENT_DIR       = $(shell basename $(CURDIR))
OBJDIR  = $(PROJECT_DIR)/bin/$(BOARD_TAG)/$(CURRENT_DIR)

include $(ARDMK_DIR)/Arduino.mk
