CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Iinclude
LDLIBS  = -lm

SRC_DIR  = src
BUILD_DIR = build

CORE_SRCS = $(SRC_DIR)/fsm.c $(SRC_DIR)/sim.c $(SRC_DIR)/runner.c
CORE_OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(CORE_SRCS))

.PHONY: all test clean

all: $(BUILD_DIR)/aegis_failsafe

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/aegis_failsafe: $(CORE_OBJS) $(BUILD_DIR)/main.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test: $(CORE_OBJS)
	$(CC) $(CFLAGS) tests/test_fsm.c $(CORE_OBJS) -o $(BUILD_DIR)/test_fsm $(LDLIBS)
	./$(BUILD_DIR)/test_fsm

clean:
	rm -rf $(BUILD_DIR)
