# tools/oracle -- differential-test oracle (real AGC code in yaAGC)

`bin/agc_probe` loads an assembled Apollo 11 rope, pokes erasable memory,
runs the cycle-accurate yaAGC engine from an entry label to a stop label and
prints erasables as octal.  `tests/oracle_*.c` compare the C translation
against these dumps.

    tools/oracle/
      bin/agc_probe, bin/yaYUL, bin/yaAGC     binaries (BUILD.md)
      ropes/{Comanche055,Luminary099}.{bin,symtab}
      cases/NAME.spec  -> cases/NAME.txt     (make cases)
      agc_probe.c, Makefile, yaAGC-src/       the driver and the engine it links

## 1. Requesting a case (module agents)

Write `cases/<module>_<routine>.spec` (one file may hold many runs) and run
`make cases` (or `AGC_ORACLE_ROPES=ropes ./bin/agc_probe < spec > out`).
Commit both the .spec and the generated .txt.  A spec is a list of
directives, `#` starts a comment:

    rope Luminary099              # or Comanche055, or a path to a .bin (first line)
    entry SPSIN                   # label (or octal address) to start at
    stop ENDOFJOB                 # run until Z == this label; may be repeated
    q ENDOFJOB                    # set Q (for routines that end with TC Q)
    ebank 5                       # EBANK for the run (octal bank number)
    fbank 30                      # override FBANK (default: bank of the entry label)
    maxcycles 100000              # MCT limit (default 2,000,000 ~ 23 s of AGC time)
    set A=04000                   # write one erasable, octal
    set MPAC=20000,00000,00000    # consecutive words
    set LOC=@LABEL                # 12-bit address of a label as the value
    set X=-5                      # ones'-complement negative (= 077772)
    set X=1000d                   # decimal
    setsp X=0.5                   # SP fraction scaled at 1 -> canonical words
    setdp MPAC=-0.125             # DP fraction -> two canonical words
    get A                         # print after the run (SP)
    get MPAC:dp                   # print MPAC and MPAC+1 ("get X:tp" for three words)
    whatis SPSIN                  # print a symbol's address as a comment
    run sin_1/8rev                # execute; the tag names the CASE block
    rupts 1                       # allow interrupts (default 0: all rupts suppressed)
    trace 1                       # one line per instruction on stderr

Names: any symbol from the symtab, `NAME+n` / `NAME-n` (decimal word offset),
unswitched erasable `0154`, banked erasable `E5,1532`, fixed `30,2000`,
fixed-fixed `05033`.  `set` lines are consumed by the next `run` (the CPU is
reset to power-up state before each run); `entry/stop/q/get/ebank/...`
persist across runs.  Every run starts from zeroed erasable, Z = entry,
FBANK/EBANK set, `A/L/Q` = 0 unless set, interrupts disabled, hardware
alarms (night watchman, TC trap, rupt lock) inhibited so a long routine
cannot GOJAM.  There is no DSKY/IMU/radar: input channels read as
`agc_engine_init` leaves them.

What a module note should list for each oracle case: rope, entry label,
stop label (and Q if the routine ends with `TC Q`), every input erasable
with its octal value and scale, every output erasable and its width.

### Basic (non-interpretive) subroutines

Routines called with `TC X` and returning with `TC Q`: `entry X`,
`q ENDOFJOB`, `stop ENDOFJOB` (any fixed-fixed label works as the return
trap; the probe stops *before* executing it).  See `cases/spsin.spec`.
Routines entered through BANKCALL/IBNKCALL can be entered directly at the
target label; the probe sets FBANK from the label's bank.

### Interpretive routines (DLOAD ... EXIT code)

Unused fixed memory cannot be patched on a real AGC, but the interpreter
fetches its opcode words through `LOC` with `INDEX LOC / CA 0`, so
interpretive code can live in erasable (that is exactly how the flown
Erasable Memory Programs worked).  Recipe, proven in
`cases/interp_sqrt_sin.spec`:

    entry INTPRET                 # real entry; does QXCH LOC, BANKSET := BBANK
    q VAC2USE                     # Q = address of the snippet = first opcode word
    stop ENDOFJOB
    set FIXLOC=@VAC1              # work area for the job (VAC1 = 0401, 43 words)
    set PUSHLOC=@VAC1             # push-down pointer
    set VAC2USE=77766,77776,@ENDOFJOB   # SQRT, EXIT, "TC ENDOFJOB" (basic word after EXIT)
    set MPAC=20000,00000,00000
    set MODE=0                    # DP mode (+0 DP, +1 TP, -1 vector)
    run sqrt_0.5

`EXIT` does `INDEX LOC / TC 1`, i.e. executes the basic word following
EXIT from erasable, which we make a `TC` to the stop label (a `TC` word is
just the 12-bit target address).  Opcode words are stored complemented
(yaYUL prints them in the listing).  Known words:

    SQRT 77766   SIN 77756   COS 77746   EXIT 77776
    "SQRT EXIT" (one pair word) 77566      "DLOAD SQRT" 75545 (+ address word)
    operand address word for erasable X: 12-bit pseudo address (ebank*0400+offset),
    addresses < 45 decimal are relative to FIXLOC (the VAC area).

To get more, assemble a snippet with yaYUL in a scratch directory:

    printf '\t\tBANK\t30\n\t\tCOUNT*\t$$/S\nINTPRET\t\tEQUALS\t4001\nX\t\tTC\tINTPRET\n\t\tDLOAD\tSQRT\n\t\t\t0\n\t\tEXIT\n' > s.agc
    tools/oracle/bin/yaYUL s.agc | grep -E '^0000'

(BANK 30 is only so that yaYUL has a fixed location counter; the octal
column is what you poke.)  Interpretive SIN/COS compute (1/2)sin(2*pi*MPAC);
SQRT takes DP MPAC in [0,1).  Scalings are documented at the top of each
AGC section; copy them into the module note.

Alternative that also works: `entry` at a real caller and `stop` at a
label inside it, with all the inputs the caller reads poked first.  The
snippet approach is preferred because it isolates exactly one routine.

## 2. Output format (what tests/oracle_*.c parse)

One record per `run`, every line is `TOKEN value`:

    CASE sqrt_0.5
    ROPE Luminary099
    ENTRY INTPRET
    STATUS OK                 # OK = stop label reached; LIMIT = maxcycles hit
    STOP ENDOFJOB             # which stop label fired ("-" on LIMIT)
    CYCLES 170                # MCTs executed (11.72 us each)
    Z 05155
    MPAC 26501                # one "NAME octal" line per requested word
    MPAC+1 07463
    # MPAC = 189812531 (DP 0.707106780261)     <- comments, ignore
    MPAC+2 00000
    END

Values are 5 octal digits, 15 bits, ones' complement, exactly as stored
(`A`, `L`, `Q` are overflow-corrected).  A test should locate its `CASE`
block, require `STATUS OK`, and compare the `NAME octal` lines it needs.
The probe exits 1 if any run hit the cycle limit, 2 on a spec error.

## 3. Caveats

* Interrupts are suppressed (all requests cleared each MCT) unless
  `rupts 1`; routines that depend on WAITLIST/T4RUPT are out of scope.
* Only erasable can be written/read from the spec; fixed memory is the rope.
* A DP value is two independently-signed words: `-0.125` is `73777,77777`;
  `74000,00000` is -2047/2^14.  Use `setdp` to avoid this.
* The symbol table is yaYUL's, so names are exact and case-sensitive;
  `whatis NAME` prints bank/address and the defining source line.
* `tools/oracle` is tooling, not flight code: the project coding standard
  and `tools/check_rules.py` do not apply to it, and it links GPLv2 code.
