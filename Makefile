ROOT_DIR := $(abspath .)
SRC_DIR ?= $(ROOT_DIR)/src
BUILD_DIR ?= $(ROOT_DIR)/build
BIN_DIR ?= $(BUILD_DIR)/bin
TOOLS_DIR ?= $(HOME)/Tools
BASH_COMPLETION_DIR ?= $(HOME)/.local/share/bash-completion/completions

CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra
INCLUDES := -I$(ROOT_DIR)/mavlink_lib/common -I$(SRC_DIR)/common
LDLIBS ?= -lm

APP_DIRS := $(wildcard $(SRC_DIR)/apps/*)
APP_NAMES := $(notdir $(APP_DIRS))
APP_BINS := $(addprefix $(BIN_DIR)/,$(APP_NAMES))
DAEMON_APP_NAMES ?= monitoringd stream_odometry stream_commander
INSTALL_APP_NAMES := $(filter-out $(DAEMON_APP_NAMES),$(APP_NAMES))
INSTALL_APP_BINS := $(addprefix $(BIN_DIR)/,$(INSTALL_APP_NAMES))
INSTALL_EXCLUDED_TOOLS := $(addprefix $(TOOLS_DIR)/,$(DAEMON_APP_NAMES))
MODE_COMPLETION_SRC := $(ROOT_DIR)/config/bash_completion/mode
MODE_COMPLETION_DST := $(BASH_COMPLETION_DIR)/mode
COMMON_SRCS := $(wildcard $(SRC_DIR)/common/*.c)
COMMON_OBJS := $(patsubst $(SRC_DIR)/common/%.c,$(BUILD_DIR)/common/%.o,$(COMMON_SRCS))
ALL_TEST_C_SRCS := $(wildcard $(SRC_DIR)/apps/*/*_test.c)
TEST_SRCS := $(foreach s,$(ALL_TEST_C_SRCS),\
  $(if $(filter $(notdir $(basename $(s))),$(notdir $(patsubst %/,%,$(dir $(s))))),,$(s)))
TEST_BINS := $(foreach s,$(TEST_SRCS),$(BIN_DIR)/$(notdir $(basename $(s))))
ALL_BINS := $(APP_BINS) $(TEST_BINS)
INSTALL_BINS := $(INSTALL_APP_BINS) $(TEST_BINS)

define app_src
$(firstword \
  $(wildcard $(SRC_DIR)/apps/$(1)/main.c) \
  $(wildcard $(SRC_DIR)/apps/$(1)/$(1).c) \
  $(wildcard $(SRC_DIR)/apps/$(1)/*.c))
endef

.PHONY: all build clean install

all: build

build: $(APP_BINS)
build: $(TEST_BINS)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BUILD_DIR)/common:
	mkdir -p $(BUILD_DIR)/common

$(BUILD_DIR)/common/%.o: $(SRC_DIR)/common/%.c | $(BUILD_DIR)/common
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

define build_app
$(BIN_DIR)/$(1): $(call app_src,$(1)) $(COMMON_OBJS) | $(BIN_DIR)
	$$(CC) $$(CFLAGS) $$(INCLUDES) $$^ $$(LDLIBS) -o $$@
endef

$(foreach app,$(APP_NAMES),$(eval $(call build_app,$(app))))

define build_test
$(BIN_DIR)/$(notdir $(basename $(1))): $(1) $(COMMON_OBJS) | $(BIN_DIR)
	$$(CC) $$(CFLAGS) $$(INCLUDES) $$^ $$(LDLIBS) -o $$@
endef

$(foreach src,$(TEST_SRCS),$(eval $(call build_test,$(src))))

install: build
	mkdir -p $(TOOLS_DIR)
	mkdir -p $(BASH_COMPLETION_DIR)
	rm -f $(INSTALL_EXCLUDED_TOOLS)
	install -m 0755 $(INSTALL_BINS) $(TOOLS_DIR)/
	install -m 0644 $(MODE_COMPLETION_SRC) $(MODE_COMPLETION_DST)

clean:
	rm -rf $(BUILD_DIR)
