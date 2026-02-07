ROOT_DIR := $(abspath .)
SRC_DIR ?= $(ROOT_DIR)/src
BUILD_DIR ?= $(ROOT_DIR)/build
BIN_DIR ?= $(BUILD_DIR)/bin
TOOLS_DIR ?= $(HOME)/Tools

CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra
INCLUDES := -I$(ROOT_DIR)/mavlink_lib/common -I$(SRC_DIR)/common

SCAN_SRC := $(SRC_DIR)/apps/scan/scan_main.c
SERVO_SRC := $(SRC_DIR)/apps/servo_test/servo_test.c
MOTOR_SRC := $(SRC_DIR)/apps/motor_init/motor_init.c
BATT_SRC := $(SRC_DIR)/apps/mav_batt/mav_batt.c
BATTD_SRC := $(SRC_DIR)/apps/mav_batt/mav_battd.c

SCAN_BIN := $(BIN_DIR)/scan
SERVO_BIN := $(BIN_DIR)/servo_test
MOTOR_BIN := $(BIN_DIR)/motor_init
BATT_BIN := $(BIN_DIR)/mav_batt
BATTD_BIN := $(BIN_DIR)/mav_battd

.PHONY: all build clean install

all: build

build: $(SCAN_BIN) $(SERVO_BIN) $(MOTOR_BIN) $(BATT_BIN) $(BATTD_BIN)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(SCAN_BIN): $(SCAN_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -lm -o $@

$(SERVO_BIN): $(SERVO_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -lm -o $@

$(MOTOR_BIN): $(MOTOR_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -lm -o $@

$(BATT_BIN): $(BATT_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

$(BATTD_BIN): $(BATTD_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

install: build
	mkdir -p $(TOOLS_DIR)
	install -m 0755 $(SCAN_BIN) $(TOOLS_DIR)/
	install -m 0755 $(SERVO_BIN) $(TOOLS_DIR)/
	install -m 0755 $(MOTOR_BIN) $(TOOLS_DIR)/
	install -m 0755 $(BATT_BIN) $(TOOLS_DIR)/
	install -m 0755 $(BATTD_BIN) $(TOOLS_DIR)/

clean:
	rm -rf $(BUILD_DIR)
