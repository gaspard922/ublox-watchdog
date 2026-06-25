CC = cc
CFLAGS = -Wall -Wextra -Wpedantic -std=c11 -Isrc
LDFLAGS =

SRC_DIR = src
BUILD_DIR = build

SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/serial_port.c $(SRC_DIR)/ubx_parser.c \
       $(SRC_DIR)/ubx_protocol.c $(SRC_DIR)/ubx_checksum.c $(SRC_DIR)/ubx_cfg.c \
       $(SRC_DIR)/ubx_poll.c $(SRC_DIR)/setup_messages.c
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

TARGET = $(BUILD_DIR)/ublox-watchdog

TEST_DIR = tests

TEST_PARSER_LIB_SRCS = $(SRC_DIR)/ubx_parser.c $(SRC_DIR)/ubx_checksum.c $(SRC_DIR)/ubx_protocol.c $(SRC_DIR)/ubx_cfg.c
TEST_PARSER_TARGET = $(BUILD_DIR)/test_ubx_parser

TEST_FUSION_LIB_SRCS = $(SRC_DIR)/ubx_fusion_tracker.c $(SRC_DIR)/ubx_protocol.c
TEST_FUSION_TARGET = $(BUILD_DIR)/test_fusion_tracker

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TEST_PARSER_TARGET) $(TEST_FUSION_TARGET)
	./$(TEST_PARSER_TARGET)
	./$(TEST_FUSION_TARGET)

$(TEST_PARSER_TARGET): $(TEST_DIR)/test_ubx_parser.c $(TEST_PARSER_LIB_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(TEST_FUSION_TARGET): $(TEST_DIR)/test_fusion_tracker.c $(TEST_FUSION_LIB_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD_DIR)
