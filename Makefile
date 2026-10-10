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
TEST_SRCS := $(shell find tests -name '*.c' | sort)
TEST_BINS := $(patsubst tests/%.c,$(BUILD)/tests/%,$(TEST_SRCS))

# NASA SEL-94-003 style (docs/STYLE.md): Uncrustify 0.72 + tools/check_style.py
UNCRUSTIFY     ?= uncrustify
UNCRUSTIFY_CFG := uncrustify.cfg
FORMAT_SRCS = $(shell find src include tests -name '*.[ch]' | sort)
STYLE_FLAGS ?=

.PHONY: all lib sim test check check-rules format format-check style lint clean
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

# Power of Ten rules, Uncrustify layout, then the SEL-94-003 text checks.
# Use `make -k check` to see all three reports when one fails.
check: check-rules format-check style

check-rules:
	python3 -I tools/check_rules.py src include

# Rewrites files in place: run on a clean tree and rebuild afterwards.
# Two passes: Uncrustify 0.72 aligns before it wraps long lines, so a line
# it wraps (or a trailing comment whose gap changed) can regroup on the next
# run.  The second pass reaches the fixed point; a third changes nothing.
format:
	@command -v $(UNCRUSTIFY) >/dev/null || { echo "$(UNCRUSTIFY) not installed"; exit 1; }
	$(UNCRUSTIFY) -q -c $(UNCRUSTIFY_CFG) --replace --no-backup $(FORMAT_SRCS)
	$(UNCRUSTIFY) -q -c $(UNCRUSTIFY_CFG) --replace --no-backup $(FORMAT_SRCS)

# Lists each file that would change (FAIL: ...) and fails if there is one.
format-check:
	@command -v $(UNCRUSTIFY) >/dev/null || { echo "$(UNCRUSTIFY) not installed"; exit 1; }
	@out=$$($(UNCRUSTIFY) -q -c $(UNCRUSTIFY_CFG) --check $(FORMAT_SRCS)); rc=$$?; \
	  printf '%s\n' "$$out" | grep -v '^PASS' || true; exit $$rc

# Report only (never edits). STYLE_FLAGS=--summary or --exit-zero, or
# STYLE_FLAGS='--rule return-parens' to narrow the report.
style:
	python3 -I tools/check_style.py $(STYLE_FLAGS) src include tests

lint:
	@command -v cppcheck >/dev/null && cppcheck --std=c11 --enable=warning,style,performance,portability --error-exitcode=1 -Iinclude src || echo "cppcheck not installed"

clean:
	rm -rf $(BUILD)

-include $(LIB_OBJS:.o=.d) $(SIM_OBJS:.o=.d)
