# tools/oracle/Makefile -- builds agc_probe against the vendored yaAGC engine
# and regenerates the oracle cases.  Independent of the project Makefile.
#
#   make            build bin/agc_probe
#   make cases      run every cases/*.spec -> cases/*.txt
#   make tools      fetch + build yaYUL/yaAGC and write the rope symbol tables
#                   (the project Makefile's oracle-tools; untracked outputs)
#   make clean
#
# YAAGC_SRC may point at a virtualagc checkout's yaAGC/ directory instead of
# the vendored copy (the vendored copy is virtualagc commit c239e9c, 2026-10-09).

CC       ?= gcc
YAAGC_SRC ?= yaAGC-src
BUILD    := build
ENGINE_SRCS := agc_engine.c agc_engine_init.c agc_utilities.c agc_symtab.c \
               rfopen.c NullAPI.c
ENGINE_OBJS := $(ENGINE_SRCS:%.c=$(BUILD)/%.o)
ENGINE_CFLAGS := -O2 -w -DNVER='""' -DINSTALLDIR='""'

SPECS := $(wildcard cases/*.spec)
OUTS  := $(SPECS:%.spec=%.txt)

.PHONY: all cases tools clean
all: bin/agc_probe

bin/agc_probe: agc_probe.c $(ENGINE_OBJS) | bin
	$(CC) -O2 -Wall -Wextra -I$(YAAGC_SRC) -o $@ agc_probe.c $(ENGINE_OBJS) -lm

$(BUILD)/%.o: $(YAAGC_SRC)/%.c | $(BUILD)
	$(CC) $(ENGINE_CFLAGS) -I$(YAAGC_SRC) -c -o $@ $<

$(BUILD) bin:
	mkdir -p $@

tools:
	$(MAKE) -C ../.. oracle-tools

ropes/%.symtab:
	$(MAKE) -C ../.. tools/oracle/ropes/$*.symtab

cases: $(OUTS)

cases/%.txt: cases/%.spec bin/agc_probe ropes/Comanche055.symtab ropes/Luminary099.symtab
	AGC_ORACLE_ROPES=ropes ./bin/agc_probe < $< > $@ || (cat $@; false)

clean:
	rm -rf $(BUILD) bin/agc_probe
