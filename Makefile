PROJECT_DIR = $(PWD)

# on the Linux
# ARDUINO_DIR   = $(PWD)/../MegaCore/avr
# ARDMK_DIR     = /usr/share/arduino
# AVR_TOOLS_DIR = /usr
# ARDUINO_PORT = /dev/ttyUSB0
# ALTERNATE_CORE_PATH = $(PWD)/../MegaCore/avr
# AVRDUDE_CONF  = $(ALTERNATE_CORE_PATH)/avrdude.conf

# on the Mac
ARDUINO_DIR = $(HOME)/work/MegaCore/avr
ALTERNATE_CORE_PATH = $(HOME)/work/MegaCore/avr
ARDMK_DIR   = /usr/local/opt/arduino-mk
AVR_TOOLS_DIR = /usr/local
AVRDUDE_CONF  = $(ALTERNATE_CORE_PATH)/avrdude.conf
ARDUINO_PORT = /dev/tty.SLAB_USBtoUART

AVRDUDE_ARD_BAUDRATE = 38400
BOARD_TAG = 64
F_CPU = 8000000L

USER_LIB_PATH += $(PROJECT_DIR)/lib
ARDUINO_LIBS  += Wire \
	SoftwareSerial \
	VariableTimedAction \
	Adafruit_GPS \
	Rtc \
	HT16K33 \
	KappaGPS \
	KappaDateTime \
	KappaSegment \
	CDS \
	DHT \
	Button \
	NoInteraction

CURRENT_DIR       = $(shell basename $(CURDIR))
OBJDIR  = $(PROJECT_DIR)/bin/$(BOARD_TAG)/$(CURRENT_DIR)

include $(ARDMK_DIR)/Arduino.mk
