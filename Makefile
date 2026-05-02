SHELL := /bin/bash

BOARD        ?= rpi_pico2/rp2350a/m33
ZEPHYR_DIR   ?= $(HOME)/zephyrproject-v4.3
BUILD_DIR    ?= $(ZEPHYR_DIR)/build
SRC_DIR      ?= $(CURDIR)
MOUNT_POINT  ?= /media/$(USER)/RP2350
SERIAL_PORT  ?= /dev/ttyACM0
BAUD         ?= 115200

.PHONY: setup build flash monitor clean

setup:
	./setup_zephyr.sh $(ZEPHYR_DIR)

build:
	source $(ZEPHYR_DIR)/.venv/bin/activate && \
	cd $(ZEPHYR_DIR) && \
	west build -b $(BOARD) $(SRC_DIR) --pristine always

flash:
	cp $(BUILD_DIR)/zephyr/zephyr.uf2 $(MOUNT_POINT)/

monitor:
	picocom $(SERIAL_PORT) -b $(BAUD)

clean:
	rm -rf $(BUILD_DIR)
