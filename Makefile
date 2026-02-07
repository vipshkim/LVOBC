ROOT_DIR := $(abspath .)
SRC_DIR ?= $(ROOT_DIR)/src
BUILD_DIR ?= $(ROOT_DIR)/build
BIN_DIR ?= $(BUILD_DIR)/bin
TOOLS_DIR ?= $(HOME)/Tools

CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra
INCLUDES := -I$(ROOT_DIR)/mavlink_lib/common -I$(SRC_DIR)/common
LDLIBS ?= -lm

APP_DIRS := $(wildcard $(SRC_DIR)/apps/*)
APP_NAMES := $(notdir $(APP_DIRS))
APP_BINS := $(addprefix $(BIN_DIR)/,$(APP_NAMES))

define app_src
$(firstword \
  $(wildcard $(SRC_DIR)/apps/$(1)/main.c) \
  $(wildcard $(SRC_DIR)/apps/$(1)/$(1).c) \
  $(wildcard $(SRC_DIR)/apps/$(1)/*.c))
endef

.PHONY: all build clean install

all: build

build: $(APP_BINS)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

define build_app
$(BIN_DIR)/$(1): $(call app_src,$(1)) | $(BIN_DIR)
	$$(CC) $$(CFLAGS) $$(INCLUDES) $$^ $$(LDLIBS) -o $$@
endef

$(foreach app,$(APP_NAMES),$(eval $(call build_app,$(app))))

install: build
	mkdir -p $(TOOLS_DIR)
	install -m 0755 $(APP_BINS) $(TOOLS_DIR)/

clean:
	rm -rf $(BUILD_DIR)
