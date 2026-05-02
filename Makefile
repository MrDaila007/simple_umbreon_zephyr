SHELL := /bin/bash

BOARD        ?= rpi_pico2/rp2350a/m33
ZEPHYR_DIR   ?= $(HOME)/zephyrproject-v4.4
BUILD_DIR    ?= $(ZEPHYR_DIR)/build
SRC_DIR      ?= $(CURDIR)
MOUNT_POINT  ?= /media/$(USER)/RP2350
SERIAL_PORT  ?= /dev/ttyACM0
BAUD         ?= 115200

.DEFAULT_GOAL := help

.PHONY: setup build flash monitor clean help

help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Variables (override with make VAR=value):"
	@echo "  BOARD=$(BOARD)"
	@echo "  ZEPHYR_DIR=$(ZEPHYR_DIR)"
	@echo "  BUILD_DIR=$(BUILD_DIR)"
	@echo "  MOUNT_POINT=$(MOUNT_POINT)"
	@echo "  SERIAL_PORT=$(SERIAL_PORT)"

setup: ## Install Zephyr SDK and workspace
	./setup_zephyr.sh $(ZEPHYR_DIR)

build: ## Build firmware (always pristine)
	source $(ZEPHYR_DIR)/.venv/bin/activate && \
	cd $(ZEPHYR_DIR) && \
	west build -b $(BOARD) $(SRC_DIR) --build-dir $(BUILD_DIR) --pristine always

flash: ## Copy zephyr.uf2 to mounted RP2350 (hold BOOTSEL first)
	cp $(BUILD_DIR)/zephyr/zephyr.uf2 $(MOUNT_POINT)/

monitor: ## Open serial console via picocom
	picocom $(SERIAL_PORT) -b $(BAUD)

clean: ## Remove build directory
	rm -rf $(BUILD_DIR)
