.PHONY: clean all distclean
# CC		= mipsel-openwrt-linux-gcc
CC		= g++

CURR_DIR := $(shell pwd)

MY_CFLAGS := -shared -fPIC -DVARIANT_generic_f407v
MY_CFLAGS +=-I$(CURR_DIR)/mcu \
			-I$(CURR_DIR)/Servo/src/ \
			-I$(CURR_DIR)/Servo/src/stm32f4 \
			-I$(CURR_DIR)/ArduinoCore/cores/maple \
			-I$(CURR_DIR)/ArduinoCore/cores/maple/avr \
			-I$(CURR_DIR)/ArduinoCore/cores/maple/libmaple \
			-I$(CURR_DIR)/ArduinoCore/cores/maple/libmaple/usbF4 \
			-I$(CURR_DIR)/ArduinoCore/system/libmaple \
			-I$(CURR_DIR)/ArduinoCore/variants/generic_f407v

PEST_BIN := pest_bin

MCU_DIR := $(CURR_DIR)/mcu
SERVO_DIR := $(CURR_DIR)/Servo/src/stm32f4
ARD_CORE_DIR := $(CURR_DIR)/ArduinoCore/cores/maple
ARD_SYSTEM_DIR := $(CURR_DIR)/ArduinoCore/system/libmaple
GEN_LIB_DIR := $(CURR_DIR)/ArduinoCore/variants/generic_f407v

OBJ_SRCS := $(shell ls $(MCU_DIR)/*.cpp) \
			$(shell ls $(SERVO_DIR)/*.cpp) \
			$(shell ls $(ARD_CORE_DIR)/*.cpp) \
			$(shell ls $(ARD_CORE_DIR)/avr/*.cpp) \
			$(shell ls $(ARD_CORE_DIR)/libmaple/*.cpp) \
			$(shell ls $(ARD_SYSTEM_DIR)/*.cpp) \
			$(shell ls $(GEN_LIB_DIR)/*.cpp) \
			$(shell ls $(MCU_DIR)/*.c) \
			$(shell ls $(SERVO_DIR)/*.c) \
			$(shell ls $(ARD_CORE_DIR)/*.c) \
			$(shell ls $(ARD_CORE_DIR)/avr/*.c) \
			$(shell ls $(ARD_CORE_DIR)/libmaple/*.c) \
			$(shell ls $(ARD_CORE_DIR)/libmaple/usbF4/*.c) \
			$(shell ls $(ARD_SYSTEM_DIR)/*.c)

all: $(PEST_BIN)

$(PEST_BIN):$(OBJ_SRCS)
	$(CC) $(CFLAGS) $(MY_CFLAGS) $^ $(LDFLAGS) -o $@

clean:
	rm -rf $(MCU_DIR)/*.o
	rm -rf $(SERVO_DIR)/*.o
	rm -rf $(ARD_CORE_DIR)/*.o
	rm -f  $(CURR_DIR)/$(PEST_BIN)

distclean: clean
	rm -f  $(CURR_DIR)/$(PEST_BIN)