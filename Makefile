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
# The same sources with -DAGC_HOST_DEBUG: a failed AGC_ASSERT inside the
# library is reported (agc_assert_report), so unit tests see it.
DBG_OBJS := $(patsubst %.c,$(BUILD)/debug/%.o,$(LIB_SRCS))
SIM_SRCS := $(wildcard src/sim/*.c)
SIM_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SIM_SRCS))
TEST_SRCS := $(shell find tests -name '*.c' | sort)
TEST_BINS := $(patsubst tests/%.c,$(BUILD)/tests/%,$(TEST_SRCS))

# NASA SEL-94-003 style (docs/STYLE.md): Uncrustify 0.72 + tools/check_style.py
UNCRUSTIFY     ?= uncrustify
FORMAT_SRCS = $(shell find src include tests -name '*.[ch]' | sort)
STYLE_FLAGS ?=

.PHONY: all lib lib-debug sim sim-debug scenarios test check check-rules format format-check style style-report lint callgraph prove prove-classify clean
all: lib sim

# The archive is rebuilt from scratch: several objects share a base name
# (clock.o, state.o, ...), and `ar r` replaces members by base name only.
lib: $(BUILD)/libagc.a
$(BUILD)/libagc.a: $(LIB_OBJS)
	@mkdir -p $(dir $@)
	@rm -f $@
	$(AR) rcs $@ $^

lib-debug: $(BUILD)/libagc_debug.a
$(BUILD)/libagc_debug.a: $(DBG_OBJS)
	@mkdir -p $(dir $@)
	@rm -f $@
	$(AR) rcs $@ $^

sim: $(BUILD)/agc_sim
$(BUILD)/agc_sim: $(SIM_OBJS) $(BUILD)/libagc.a
	$(CC) $(CFLAGS) -o $@ $^

# The harness against the debug library: failed AGC_ASSERTs are reported.
sim-debug: $(BUILD)/agc_sim_debug
$(BUILD)/agc_sim_debug: $(SIM_OBJS) $(BUILD)/libagc_debug.a
	$(CC) $(CFLAGS) -o $@ $^

# End-to-end scenarios (docs/modules/sim.md): tools/sim_client.py drives
# build/agc_sim in lockstep over the yaAGC peripheral protocol.
SCENARIOS := $(sort $(wildcard sim/scenarios/*.txt))
scenarios: $(BUILD)/agc_sim
	@for s in $(SCENARIOS); do echo "== $$s"; \
	  python3 -I tools/sim_client.py --sim $(BUILD)/agc_sim --scenario $$s || exit 1; done

$(BUILD)/debug/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) $(CPPFLAGS) -DAGC_HOST_DEBUG -MMD -MP -c -o $@ $<

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/tests/%: tests/%.c $(BUILD)/libagc_debug.a $(wildcard tests/oracle/*.h)
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) $(CPPFLAGS) -DAGC_HOST_DEBUG -o $@ $< $(BUILD)/libagc_debug.a -lm

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "== $$t"; $$t || exit 1; done

# Power of Ten rules, Uncrustify layout, then the SEL-94-003 text checks.
# The style findings are reported but do not fail `make check` until the
# review wave has worked them down (`make style` fails on any finding).
check: check-rules format-check style-report

check-rules:
	python3 -I tools/check_rules.py src include

# Rewrites files in place: run on a clean tree and rebuild afterwards.
# tools/format.sh runs tools/fix_comments.py (comments out of expressions,
# wraps joined, paragraphs), Uncrustify twice, then tools/fix_comments.py
# again (paragraph column alignment); the result is a fixed point.
format:
	@echo "tools/format.sh: $(words $(FORMAT_SRCS)) files"
	@UNCRUSTIFY=$(UNCRUSTIFY) sh tools/format.sh $(FORMAT_SRCS)

# Formats copies; lists each file that would change (FAIL: ...) and fails
# if there is one.
format-check:
	@UNCRUSTIFY=$(UNCRUSTIFY) sh tools/format.sh --check $(FORMAT_SRCS)

# Report only (never edits). STYLE_FLAGS=--summary or --exit-zero, or
# STYLE_FLAGS='--rule return-parens' to narrow the report.
style:
	python3 -I tools/check_style.py $(STYLE_FLAGS) src include tests

style-report:
	python3 -I tools/check_style.py --summary --exit-zero src include tests

lint:
	@command -v cppcheck >/dev/null && cppcheck --std=c11 --enable=warning,style,performance,portability --error-exitcode=1 -Iinclude src || echo "cppcheck not installed"

# Call graph from the compiler (docs/agents/collapse.md): every src/ and
# tests/ TU compiled at -O0 -fno-inline with -fcallgraph-info=su,da (GCC
# writes one VCG .ci file next to each object), cflow for the cross-TU
# cross-check, libclang (inside the script) for extents and guard ASTs.
# Writes docs/overview/callgraph.{tsv,dot} and collapse_candidates.tsv.
CG_DIR   := $(BUILD)/callgraph
CG_SRCS  := $(shell find src -name '*.c' | sort)
CG_TESTS := $(shell find tests -name '*.c' | sort)
CG_CIS   := $(patsubst %.c,$(CG_DIR)/%.ci,$(CG_SRCS) $(CG_TESTS))

callgraph: $(CG_CIS)
	cflow -b --omit-arguments --no-main -i s $(CG_SRCS) > $(CG_DIR)/cflow.txt 2>/dev/null
	python3 -I tools/callgraph.py --ci-dir $(CG_DIR) --cflow $(CG_DIR)/cflow.txt \
	    --out docs/overview

$(CG_DIR)/src/%.ci: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) -O0 -fno-inline $(CPPFLAGS) -fcallgraph-info=su,da -c -o $(@:.ci=.o) $<

$(CG_DIR)/tests/%.ci: tests/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) -O0 -fno-inline $(CPPFLAGS) -DAGC_HOST_DEBUG -fcallgraph-info=su,da -c -o $(@:.ci=.o) $<

# Boundary-check discipline (docs/modules/status-handling-2.md): Frama-C EVA
# proves or refutes every AGC_ASSERT, one analysis per entry point (the sim
# harness and each test main, with the whole library), two at a time, 60 min
# each; it runs detached.  Watch tools/prove/progress.log (one line per run)
# and tools/prove/summary.txt (done/total, verdicts, ETA).  When it has
# finished, `make prove-classify` merges the verdicts with the libclang
# call-site classification into docs/overview/guards.tsv: a guard left in
# src/ must be FALLIBLE (classified GUARANTEED means it is dead code).
prove:
	@mkdir -p $(BUILD)/prove
	nohup setsid python3 -I tools/prove/eva_run.py > $(BUILD)/prove/run.txt 2>&1 < /dev/null &
	@echo "EVA started in the background: tail -f tools/prove/progress.log"

prove-classify:
	python3 -I tools/prove/eva_run.py --merge-only
	python3 -I tools/prove/guards.py index
	python3 -I tools/prove/guards.py classify

clean:
	rm -rf $(BUILD)

-include $(LIB_OBJS:.o=.d) $(DBG_OBJS:.o=.d) $(SIM_OBJS:.o=.d)
