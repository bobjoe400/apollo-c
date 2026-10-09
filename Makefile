# apollo-c -- Power of Ten rule 10: pedantic warnings, zero tolerated.
CC      ?= gcc
CSTD    := -std=c11
WARN    := -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow \
           -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wcast-qual \
           -Wswitch-enum -Werror
CFLAGS  ?= -O2 -g
CPPFLAGS := -Iinclude
BUILD   := build

LIB_SRCS := $(shell find src -name '*.c' -not -path 'src/sim/*' | sort)
LIB_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(LIB_SRCS))
SIM_SRCS := $(wildcard src/sim/*.c)
SIM_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SIM_SRCS))
TEST_SRCS := $(wildcard tests/*.c)
TEST_BINS := $(patsubst tests/%.c,$(BUILD)/tests/%,$(TEST_SRCS))

.PHONY: all lib sim test check lint clean
all: lib sim

lib: $(BUILD)/libagc.a
$(BUILD)/libagc.a: $(LIB_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

sim: $(BUILD)/agc_sim
$(BUILD)/agc_sim: $(SIM_OBJS) $(BUILD)/libagc.a
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/tests/%: tests/%.c $(BUILD)/libagc.a
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) $(CPPFLAGS) -DAGC_HOST_DEBUG -o $@ $< $(BUILD)/libagc.a -lm

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "== $$t"; $$t || exit 1; done

check:
	python3 -I tools/check_rules.py src include

lint:
	@command -v cppcheck >/dev/null && cppcheck --std=c11 --enable=warning,style,performance,portability --error-exitcode=1 -Iinclude src || echo "cppcheck not installed"

clean:
	rm -rf $(BUILD)

-include $(LIB_OBJS:.o=.d) $(SIM_OBJS:.o=.d)
