/*
 * agc_probe.c -- differential-test driver for the apollo-c oracle.
 *
 * Links Ron Burkey's yaAGC engine (GPLv2, see yaAGC-src/COPYING), loads an
 * assembled rope (.bin) and its yaYUL symbol table (.symtab), applies a
 * key=value spec read from stdin, runs the emulated AGC from an entry label
 * until a stop label (or a cycle limit) and prints requested erasables as
 * octal.  See README.md in this directory for the spec and output formats.
 *
 * This file is part of the apollo-c tooling; it is NOT flight-software
 * translation and the project coding standard does not apply to it.
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_engine.h"
#include "agc_symtab.h"

extern Symbol_t *SymbolTable;
extern int SymbolTableSize;
extern int InhibitAlarms;

/* The engine calls this debugger hook; the probe keeps no backtrace. */
void BacktraceAdd(agc_t *State, int Cause) { (void)State; (void)Cause; }
/* Source-listing hooks used by agc_symtab.c; the probe never lists source. */
void nbclear_files(void) {}
void nbadd_source_file(const char *file) { (void)file; }
char *NormalizeSourceName(char *Directory, char *Filename) { (void)Directory; return Filename; }

#define MAX_LINES   4096
#define MAX_STOPS   16
#define MAX_SETS    512
#define MAX_GETS    256
#define DEFAULT_MAX_CYCLES 2000000ULL

/* A resolved memory location.  fixed=0: Erasable[bank][sreg & 0377].
 * fixed=1: Fixed[bank][sreg & 01777] where bank is the Fixed[] index
 * (2,3 = fixed-fixed; 0..037 = switchable; 040..043 = superbank). */
typedef struct {
    int sreg;      /* 12-bit S-register address */
    int bank;
    int fixed;
    char name[64]; /* display name as written in the spec */
} loc_t;

typedef struct { loc_t loc; int nwords; int words[8]; } set_t;
typedef struct { loc_t loc; int width; } get_t;   /* width 1=sp 2=dp 3=tp */

static char *lines[MAX_LINES];
static int nlines;

static agc_t pristine;
static agc_t cpu;

static char rope_name[256] = "Luminary099";
static loc_t entry;      static int have_entry;
static loc_t stops[MAX_STOPS]; static int nstops;
static loc_t qloc;       static int have_q;
static int ebank_override = -1;
static int fbank_override = -1;
static unsigned long long max_cycles = DEFAULT_MAX_CYCLES;
static int allow_rupts = 0;
static int trace = 0;
static set_t sets[MAX_SETS]; static int nsets;
static get_t gets[MAX_GETS]; static int ngets;
static int nruns;
static int exit_status;

/* ---------------------------------------------------------------- utils */

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "agc_probe: ");
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(2);
}

static char *trim(char *s)
{
    char *e;
    while (isspace((unsigned char)*s)) s++;
    e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

static int sp_signed(int v)
{
    v &= 077777;
    return (v & 040000) ? -((~v) & 037777) : v;
}

static Symbol_t *find_symbol(const char *name)
{
    int i;
    for (i = 0; i < SymbolTableSize; i++)
        if (strcmp(SymbolTable[i].Name, name) == 0)
            return &SymbolTable[i];
    return NULL;
}

/* Compute (bank, sreg, fixed) of a symbol plus a word offset. */
static int symbol_loc(Symbol_t *s, int offset, loc_t *out)
{
    Address_t *a = &s->Value;
    int v;
    if (a->Invalid) return -1;
    if (a->Address || s->Type == SYMBOL_VARIABLE || s->Type == SYMBOL_LABEL
        || s->Type == SYMBOL_REGISTER) {
        v = (int)a->SReg + offset;
        if (a->Erasable || v < 02000) {
            out->fixed = 0;
            out->sreg = v;
            out->bank = (v < 01400) ? (v >> 8) : (int)a->EB;
        } else {
            out->fixed = 1;
            out->sreg = v;
            if (v >= 04000) out->bank = 2 + ((v - 04000) >> 10);
            else out->bank = (int)a->FB + (a->Super ? 010 : 0);
        }
        return 0;
    }
    if (a->Constant) {
        /* EQUALS n: treat as a pseudo-address if it looks like one. */
        v = a->Value + offset;
        if (v >= 0 && v < 04000) {
            out->fixed = 0;
            out->sreg = (v < 01400) ? v : (01400 | (v & 0377));
            out->bank = v >> 8;
            return 0;
        }
        return -1;
    }
    return -1;
}

/* Parse "NAME", "NAME+3", "NAME-1", "oooo", "Eb,oooo", "bb,oooo". */
static void parse_loc(const char *text, loc_t *out)
{
    char buf[128], *p, *plus;
    int offset = 0;
    strncpy(buf, text, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    p = trim(buf);
    strncpy(out->name, p, sizeof out->name - 1); out->name[sizeof out->name - 1] = 0;

    plus = strpbrk(p + 1, "+-");
    if (plus && !isdigit((unsigned char)p[0]) ) {
        offset = (int)strtol(plus, NULL, 10);     /* AGC "+1" offsets are decimal */
        *plus = 0;
        p = trim(p);
    }
    if (p[0] == 'E' && isdigit((unsigned char)p[1]) && p[2] == ',') {
        int bank = p[1] - '0', s = (int)strtol(p + 3, NULL, 8);
        out->fixed = 0; out->bank = bank;
        out->sreg = (s < 0400) ? (01400 | s) : s;
        return;
    }
    if (isdigit((unsigned char)p[0])) {
        char *comma = strchr(p, ',');
        if (comma) {
            int bank = (int)strtol(p, NULL, 8), s = (int)strtol(comma + 1, NULL, 8);
            if (s < 02000 || s >= 04000) die("fixed address %s must be bb,2000-3777", text);
            out->fixed = 1; out->bank = bank; out->sreg = s;
            return;
        } else {
            int s = (int)strtol(p, NULL, 8);
            if (s < 01400) { out->fixed = 0; out->bank = s >> 8; out->sreg = s; return; }
            if (s >= 04000 && s < 010000) {
                out->fixed = 1; out->bank = 2 + ((s - 04000) >> 10); out->sreg = s; return;
            }
            die("octal address %s is banked; write Eb,oooo or bb,oooo", text);
        }
    }
    {
        Symbol_t *s = find_symbol(p);
        if (!s) die("unknown symbol '%s'", p);
        if (symbol_loc(s, offset, out) != 0) die("symbol '%s' is not an address", p);
    }
}

/* Value: octal (optionally -), "NNNd" decimal, or "@LABEL" (12-bit S-reg). */
static int parse_value(const char *text)
{
    char buf[128], *p, *end;
    long v;
    int neg = 0;
    strncpy(buf, text, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    p = trim(buf);
    if (p[0] == '@') {
        loc_t l;
        parse_loc(p + 1, &l);
        return l.sreg & 07777;
    }
    if (p[0] == '-') { neg = 1; p++; }
    end = p + strlen(p);
    if (end > p && (end[-1] == 'd' || end[-1] == 'D')) {
        end[-1] = 0;
        v = strtol(p, &end, 10);
    } else {
        v = strtol(p, &end, 8);
    }
    if (*end) die("bad value '%s'", text);
    if (v < 0 || v > 077777) die("value '%s' out of range", text);
    if (neg) v = (~v) & 077777;
    return (int)v;
}

/* Signed decimal fraction in (-1,1) -> ones'-complement words, canonical
 * form (both words carry the sign).  width 1 = SP (2^14), 2 = DP (2^28). */
static void fraction_words(const char *text, int width, int *words)
{
    double x = strtod(text, NULL);
    int neg = x < 0;
    unsigned long long mag, limit = (width == 1) ? 16383ULL : 268435455ULL;
    if (neg) x = -x;
    mag = (unsigned long long)(x * (width == 1 ? 16384.0 : 268435456.0) + 0.5);
    if (mag > limit) die("fraction %s out of range", text);
    if (width == 1) words[0] = (int)mag;
    else { words[0] = (int)(mag >> 14); words[1] = (int)(mag & 037777); }
    if (neg) { words[0] = (~words[0]) & 077777; if (width == 2) words[1] = (~words[1]) & 077777; }
}

/* ---------------------------------------------------------------- memory */

static int16_t *eras_ptr(agc_t *s, const loc_t *l)
{
    if (l->fixed) die("'%s' is in fixed memory; only erasable may be read/written", l->name);
    if (l->bank < 0 || l->bank > 7) die("'%s' bad erasable bank %d", l->name, l->bank);
    return &s->Erasable[l->bank][l->sreg & 0377];
}

static void write_word(agc_t *s, const loc_t *l, int v)
{
    int16_t *p = eras_ptr(s, l);
    if (l->bank == 0 && (l->sreg & 0377) <= 2) *p = (int16_t)SignExtend((int16_t)(v & 077777));
    else *p = (int16_t)(v & 077777);
}

static int read_word(agc_t *s, const loc_t *l)
{
    int16_t *p = eras_ptr(s, l);
    int v = *p;
    if (l->bank == 0 && (l->sreg & 0377) <= 2) return OverflowCorrected(v) & 077777;
    return v & 077777;
}

static int cur_fixed_bank(const agc_t *s)
{
    int fb = (s->Erasable[0][RegFB] >> 10) & 037;
    if ((fb & 030) == 030 && (s->OutputChannel7 & 0100)) fb += 010;
    return fb;
}

static int at_loc(const agc_t *s, const loc_t *l)
{
    int z = s->Erasable[0][RegZ] & 07777;
    if (z != (l->sreg & 07777)) return 0;
    if (z < 01400 || z >= 04000) return 1;
    if (z < 02000) return ((s->Erasable[0][RegEB] >> 8) & 7) == l->bank;
    return cur_fixed_bank(s) == l->bank;
}

static void set_banks(agc_t *s, int fbank, int ebank)
{
    if (fbank >= 0) {
        int fbfield = fbank, super = 0;
        if (fbank >= 040) { fbfield = fbank - 010; super = 1; }
        s->Erasable[0][RegFB] = (int16_t)((fbfield & 037) << 10);
        s->OutputChannel7 = (int16_t)(super ? 0100 : 0);
        s->InputChannel[7] = s->OutputChannel7;
    }
    if (ebank >= 0) s->Erasable[0][RegEB] = (int16_t)((ebank & 7) << 8);
    s->Erasable[0][RegBB] = (int16_t)((s->Erasable[0][RegFB] & 076000)
                                      | ((s->Erasable[0][RegEB] & 03400) >> 8));
}

/* Nearest preceding label in the same bank, for --trace output. */
static const char *nearest_label(int fixed, int bank, int sreg, int *off_out)
{
    static char none[] = "?";
    int i, best = -1, bestv = -1;
    for (i = 0; i < SymbolTableSize; i++) {
        loc_t l;
        if (SymbolTable[i].Type != SYMBOL_LABEL) continue;
        if (symbol_loc(&SymbolTable[i], 0, &l) != 0) continue;
        if (l.fixed != fixed || l.bank != bank || l.sreg > sreg) continue;
        if (l.sreg > bestv) { bestv = l.sreg; best = i; }
    }
    if (best < 0) { *off_out = 0; return none; }
    *off_out = sreg - bestv;
    return SymbolTable[best].Name;
}

static void trace_step(const agc_t *s, unsigned long long cyc)
{
    int z = s->Erasable[0][RegZ] & 07777, bank, fixed, off;
    const char *lab;
    if (z < 01400) { fixed = 0; bank = z >> 8; }
    else if (z < 02000) { fixed = 0; bank = (s->Erasable[0][RegEB] >> 8) & 7; }
    else if (z < 04000) { fixed = 1; bank = cur_fixed_bank(s); }
    else { fixed = 1; bank = 2 + ((z - 04000) >> 10); }
    lab = nearest_label(fixed, bank, z, &off);
    fprintf(stderr, "%8llu %s%02o,%04o %s+%d A=%05o L=%05o Q=%05o\n", cyc,
            fixed ? "" : "E", bank, z, lab, off,
            OverflowCorrected(s->Erasable[0][RegA]) & 077777,
            s->Erasable[0][RegL] & 077777,
            OverflowCorrected(s->Erasable[0][RegQ]) & 077777);
}

/* ---------------------------------------------------------------- run */

static void print_get(agc_t *s, const get_t *g)
{
    loc_t l = g->loc;
    int w, v[3];
    char base[64], *plus;
    int base_off = 0;
    strcpy(base, l.name);
    plus = strpbrk(base + 1, "+-");
    if (plus && !isdigit((unsigned char)base[0])) { base_off = (int)strtol(plus, NULL, 10); *plus = 0; }
    for (w = 0; w < g->width; w++) {
        loc_t lw = l;
        lw.sreg = l.sreg + w;
        if (!l.fixed && l.sreg >= 01400 && lw.sreg >= 02000) die("'%s' runs off the E-bank", l.name);
        v[w] = read_word(s, &lw);
        if (w == 0 && base_off == 0 && !(plus && !isdigit((unsigned char)base[0])))
            printf("%s %05o\n", l.name, v[w]);
        else if (isdigit((unsigned char)base[0]))
            printf("%s+%d %05o\n", base, w, v[w]);
        else
            printf("%s%+d %05o\n", base, base_off + w, v[w]);
    }
    if (g->width == 1)
        printf("# %s = %d (SP %.9f)\n", l.name, sp_signed(v[0]), sp_signed(v[0]) / 16384.0);
    else if (g->width == 2) {
        long long d = (long long)sp_signed(v[0]) * 16384 + sp_signed(v[1]);
        printf("# %s = %lld (DP %.12f)\n", l.name, d, (double)d / 268435456.0);
    } else {
        long long t = ((long long)sp_signed(v[0]) * 16384 + sp_signed(v[1])) * 16384 + sp_signed(v[2]);
        printf("# %s = %lld (TP %.15f)\n", l.name, t, (double)t / 4398046511104.0);
    }
}

static void do_run(const char *tag)
{
    agc_t *s = &cpu;
    unsigned long long cyc = 0;
    int i, hit = -1, status_limit = 0;

    if (!have_entry) die("no 'entry' before 'run'");
    memcpy(s, &pristine, sizeof *s);

    /* Bank registers: from the entry symbol unless overridden. */
    {
        int fb = -1, eb = (ebank_override >= 0) ? ebank_override : 0;
        if (entry.fixed && entry.sreg < 04000) fb = entry.bank;
        if (fbank_override >= 0) fb = fbank_override;
        set_banks(s, fb < 0 ? 0 : fb, eb);
    }
    s->Erasable[0][RegZ] = (int16_t)(entry.sreg & 07777);
    s->AllowInterrupt = allow_rupts ? 1 : 0;
    s->DownruptTimeValid = 0;
    for (i = 1; i <= NUM_INTERRUPT_TYPES; i++) s->InterruptRequests[i] = 0;

    for (i = 0; i < nsets; i++) {
        int w;
        for (w = 0; w < sets[i].nwords; w++) {
            loc_t l = sets[i].loc;
            l.sreg += w;
            write_word(s, &l, sets[i].words[w]);
        }
    }
    if (have_q) s->Erasable[0][RegQ] = (int16_t)(qloc.sreg & 07777);
    /* Keep BB consistent even if the spec poked EB/FB directly. */
    s->Erasable[0][RegBB] = (int16_t)((s->Erasable[0][RegFB] & 076000)
                                      | ((s->Erasable[0][RegEB] & 03400) >> 8));

    for (;;) {
        if (!s->PendFlag) {
            for (i = 0; i < nstops; i++) if (at_loc(s, &stops[i])) { hit = i; break; }
            if (hit >= 0 && cyc > 0) break;
            if (hit >= 0 && cyc == 0 && !(at_loc(s, &entry))) break;
            hit = -1;
            if (trace) trace_step(s, cyc);
        }
        if (cyc >= max_cycles) { status_limit = 1; break; }
        agc_engine(s);
        cyc++;
        if (!allow_rupts)
            for (i = 1; i <= NUM_INTERRUPT_TYPES; i++) s->InterruptRequests[i] = 0;
    }

    nruns++;
    printf("CASE %s\n", tag);
    printf("ROPE %s\n", rope_name);
    printf("ENTRY %s\n", entry.name);
    printf("STATUS %s\n", status_limit ? "LIMIT" : "OK");
    printf("STOP %s\n", status_limit ? "-" : stops[hit].name);
    printf("CYCLES %llu\n", cyc);
    printf("Z %05o\n", s->Erasable[0][RegZ] & 07777);
    for (i = 0; i < ngets; i++) print_get(s, &gets[i]);
    printf("END\n");
    if (status_limit) exit_status = 1;
    nsets = 0;
}

/* ---------------------------------------------------------------- spec */

static void load_rope(void)
{
    char bin[512], sym[512];
    const char *dir = getenv("AGC_ORACLE_ROPES");
    if (strchr(rope_name, '/') || strstr(rope_name, ".bin")) {
        strncpy(bin, rope_name, sizeof bin - 1); bin[sizeof bin - 1] = 0;
    } else {
        snprintf(bin, sizeof bin, "%s/%s.bin", dir ? dir : "ropes", rope_name);
    }
    strncpy(sym, bin, sizeof sym - 1); sym[sizeof sym - 1] = 0;
    {
        char *dot = strrchr(sym, '.');
        if (dot && strcmp(dot, ".bin") == 0) strcpy(dot, ".symtab");
        else strncat(sym, ".symtab", sizeof sym - strlen(sym) - 1);
    }
    if (ReadSymbolTable(sym) != 0) die("cannot read symbol table %s", sym);
    if (agc_engine_init(&pristine, bin, NULL, 0) != 0) die("cannot load rope %s", bin);
    /* agc_engine_init() needs the rope loaded; nothing else is touched. */
    InhibitAlarms = 1;        /* no GOJAM from night-watchman / TC-trap / rupt-lock */
}

static void whatis(const char *name)
{
    Symbol_t *s = find_symbol(name);
    char buf[32];
    loc_t l;
    if (!s) { printf("# %s: undefined\n", name); return; }
    AddressPrintAGC(&s->Value, buf);
    if (symbol_loc(s, 0, &l) == 0)
        printf("# %s: %s type=%d %s bank=%02o sreg=%04o (%s:%u)\n", name, trim(buf), s->Type,
               l.fixed ? "fixed" : "erasable", l.bank, l.sreg, s->FileName, s->LineNumber);
    else
        printf("# %s: %s type=%d value=%o (%s:%u)\n", name, trim(buf), s->Type,
               s->Value.Value, s->FileName, s->LineNumber);
}

static void process_line(char *line)
{
    char *kw, *arg, *hash;
    hash = strchr(line, '#');
    if (hash) *hash = 0;
    line = trim(line);
    if (!*line) return;
    kw = line;
    arg = line;
    while (*arg && !isspace((unsigned char)*arg)) arg++;
    if (*arg) *arg++ = 0;
    arg = trim(arg);
    for (char *c = kw; *c; c++) *c = (char)tolower((unsigned char)*c);

    if (!strcmp(kw, "rope")) {
        if (SymbolTable) die("'rope' must come first");
        strncpy(rope_name, arg, sizeof rope_name - 1);
        return;
    }
    if (!SymbolTable) load_rope();

    if (!strcmp(kw, "entry")) { parse_loc(arg, &entry); have_entry = 1; }
    else if (!strcmp(kw, "stop")) {
        if (nstops >= MAX_STOPS) die("too many stops");
        parse_loc(arg, &stops[nstops++]);
    }
    else if (!strcmp(kw, "clearstops")) nstops = 0;
    else if (!strcmp(kw, "q")) { parse_loc(arg, &qloc); have_q = 1; }
    else if (!strcmp(kw, "ebank")) ebank_override = (int)strtol(arg, NULL, 8);
    else if (!strcmp(kw, "fbank")) fbank_override = (int)strtol(arg, NULL, 8);
    else if (!strcmp(kw, "maxcycles")) max_cycles = strtoull(arg, NULL, 10);
    else if (!strcmp(kw, "rupts")) allow_rupts = atoi(arg);
    else if (!strcmp(kw, "trace")) trace = atoi(arg);
    else if (!strcmp(kw, "set")) {
        char *eq = strchr(arg, '='), *tok, *save;
        set_t *st;
        if (!eq) { eq = arg; while (*eq && !isspace((unsigned char)*eq)) eq++; if (!*eq) die("set needs NAME=VALUE"); }
        *eq++ = 0;
        if (nsets >= MAX_SETS) die("too many sets");
        st = &sets[nsets];
        parse_loc(arg, &st->loc);
        st->nwords = 0;
        for (tok = strtok_r(eq, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
            if (st->nwords >= 8) die("set: at most 8 words");
            st->words[st->nwords++] = parse_value(tok);
        }
        if (!st->nwords) die("set: no value");
        nsets++;
    }
    else if (!strcmp(kw, "setsp") || !strcmp(kw, "setdp")) {
        char *eq = strchr(arg, '=');
        set_t *st;
        if (!eq) die("%s needs NAME=fraction", kw);
        *eq++ = 0;
        if (nsets >= MAX_SETS) die("too many sets");
        st = &sets[nsets];
        parse_loc(arg, &st->loc);
        st->nwords = (kw[3] == 'd') ? 2 : 1;
        fraction_words(trim(eq), st->nwords, st->words);
        nsets++;
    }
    else if (!strcmp(kw, "get")) {
        char *colon = strchr(arg, ':');
        get_t *g;
        if (ngets >= MAX_GETS) die("too many gets");
        g = &gets[ngets];
        g->width = 1;
        if (colon) {
            *colon++ = 0;
            if (!strcmp(colon, "sp")) g->width = 1;
            else if (!strcmp(colon, "dp")) g->width = 2;
            else if (!strcmp(colon, "tp")) g->width = 3;
            else die("get: width must be sp, dp or tp");
        }
        parse_loc(arg, &g->loc);
        ngets++;
    }
    else if (!strcmp(kw, "cleargets")) ngets = 0;
    else if (!strcmp(kw, "whatis")) whatis(arg);
    else if (!strcmp(kw, "run")) do_run(*arg ? arg : "default");
    else die("unknown directive '%s'", kw);
}

int main(int argc, char **argv)
{
    char buf[1024];
    int i;
    (void)argc; (void)argv;

    /* Read the whole spec first: agc_engine_init() makes stdin non-blocking. */
    while (nlines < MAX_LINES && fgets(buf, sizeof buf, stdin))
        lines[nlines++] = strdup(buf);

    printf("# agc_probe (yaAGC engine) spec lines=%d\n", nlines);
    for (i = 0; i < nlines; i++) process_line(lines[i]);
    if (nruns == 0 && have_entry) do_run("default");
    if (nruns == 0) die("nothing to run (no 'entry')");
    return exit_status;
}
