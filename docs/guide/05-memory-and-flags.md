# 5. Memory and flags

Everything the AGC knows that is not in the rope is in 2048 words of
erasable memory. The programmers allocated those words by hand, shared
many of them between activities that never run together, and gave some
words two names. This chapter explains how the listing describes
erasable, how the C stores it, what the flagwords are, what is loaded
on the launch pad, and what survives a restart.

## 5.1 How the listing describes erasable

The file ERASABLE_ASSIGNMENTS.agc allocates every word. Its first page
of conventions (Comanche C pp.37-38) is the key to reading any erasable
comment. A comment of the form M(SIZE)N says:

```
#     M     REFERS TO THE MOBILITY OF THE ASSIGNMENT.
#           B   MEANS THAT THE SYMBOL IS REFERENCED BY BASIC
#               INSTRUCTIONS AND THUS IS E-BANK SENSITIVE.
#           I   MEANS THAT THE SYMBOL IS REFERENCED ONLY BY
#               INTERPRETIVE INSTRUCTIONS, AND IS THUS E-BANK
#               INSENSITIVE AND MAY APPEAR IN ANY E-BANK.
#     SIZE  IS THE NUMBER OF REGISTERS INCLUDED BY THE SYMBOL.
#     N     INDICATES THE NATURE OF PERMANENCE OF THE CONTENTS.
#           PL  MEANS THAT THE CONTENTS ARE PAD LOADED.
#           DSP MEANS THAT THE REGISTER IS USED FOR A DISPLAY.
#           PRM MEANS THAT THE REGISTER IS PERMANENT.  IE., IT
...
#           TMP MEANS THAT THE REGISTER IS USED TEMPORARILY OR
#               IS A SCRATCH REGISTER FOR THE ROUTINE TO WHICH
#               IT IS ASSIGNED.  THAT IS, IT NEED NOT BE SET
...
```

So `RN ERASE +5 # B(6)PRM` (C p.79) is a six-word permanent quantity used by basic
code (the state vector's position), and `TLAND` "I(2)" is a DP used by
interpretive code. The same page defines the two ways to give a word a
second name:

```
#   EQUALS  OFTEN IMPLIES THE SHARING OF REGISTERS (DIFFERENT NAMES
#           AND DIFFERENT DATA).  EXAMPLE:
#                   X   EQUALS  Y
#   =       MEANS THAT MULTIPLE NAMES HAVE BEEN GIVEN TO THE SAME DATA.
#           (THIS IS LOGICAL EQUIVALENCE, NOT SHARING.)  EXAMPLE:
```

EQUALS is how 2048 words were made to serve a whole mission: a TMP
register "MAY BE SHARED WITH ANY OTHER ROUTINE WHICH IS NOT ACTIVE IN
PARALLEL" (same page), so routines that never run together overlay the
same words under different names. `=` is pure synonymy: two names, one
datum.

## 5.2 Erasable as the C sees it: typed storage

The C has no erasable array for the program to use. Each AGC erasable
is one typed C object, at the scale the listing gives it, owned by one
module (`docs/TRANSLATION_GUIDE.md`, section 3; `docs/overview/README.md`,
section 4):

* **Shared erasables** -- read or written by more than one module --
  live in `src/core/` (state.c, erasable.c, counters.c,
  display_erasables.c, pad_loads.c, restart_delays.c) behind an accessor
  pair `agc_erase_<name>()` / `agc_erase_set_<name>()` declared in
  `include/agc/core/state.h`. The prolog of each gives the AGC name,
  page and scale (`core/erasable.c`):

  ```c
  static agc_dp_t  g_tig;                    /* TIG, cs, scale 2^28 */
  ...
   * AGC: TIG (ERASABLE_ASSIGNMENTS.agc C p. 117, L p. 139) -- read TIG:
   *   time of ignition (noun 33)
  agc_dp_t agc_erase_tig(void)
  ```

* **Module-private erasables** are `static` in the owning file (the
  waitlist's `lst1`, `lst2` in `exec/waitlist.c`) or fields of one state
  struct per subsystem reached through one accessor:
  `agc_guid_state()` for the burn and landing guidance
  (`include/agc/guidance/guidance.h`), `agc_rendz_erasable()` for the
  rendezvous programs, `lm_dap_state()` and `agc_csm_dap_com()` for the
  autopilots. Every field carries its AGC name and scale:

  ```c
      agc_vec_t    uhyp;     /* UHYP     cross-range half unit (R10); (2^1) */
      agc_vec_t    uhzp;     /* UHZP     downrange half unit (R10); (2^1) */
  ```

* **The state vectors** have their own type (`agc_state_vector_t`,
  chapter 2) and accessors: `agc_sv_get(AGC_SV_THIS, &sv)` is RN, VN,
  PIPTIME with MOONFLAG -- the vehicle the computer is in -- and
  `AGC_SV_OTHER` is the other vehicle's permanent state vector
  (`include/agc/core/state.h`).

Power of Ten forbids `extern` globals, which is why every shared word
is behind an accessor; the listing's page reference beside it makes the
accessor the place to look up what a word means. The complete map from
every AGC erasable name to its C owner is in `docs/modules/state-flags.md`.

## 5.3 One erasable, one C object

Typed storage has one trap that the address space did not: two names
can become two C variables. The rule the project ended with is in
`docs/CODING_STANDARD.md`: "An EQUALS / `=` alias in ERASABLE_ASSIGNMENTS
is the same object under two names: one owner, one declaration (the
alias named in its comment), one accessor or setter that every reader
and writer uses."

The LM ascent is the clearest case. Luminary's ERASABLE_ASSIGNMENTS
(L p.151):

```
UHYP       EQUALS   DALTRATE +1   # B(6)PRM SM UNIT VECTOR
QAXIS      =        UHYP
UHZP       EQUALS   UHYP +6       # B(6)PRM SM UNIT VECTOR
```

UHYP is the descent's cross-range axis; the ascent guidance computes
QAXIS, the normal to the CSM's orbit plane, under its own name -- and
by `=` it is the same six words. The C keeps one variable and writes
the alias in the comment where the ascent stores it
(`lm/ascent.c`, P12INIT):

```c
    agc_guid_state()->lm.uhyp = agc_vec_unit(ip, agc_mat_vec(ip, &m, h), NULL, NULL); /* QAXIS EQUALS UHYP */
```

When the two names were two variables the LM ascent in the NASSP
mission flew to a 1757.9 x 1901.0 km orbit, because "the ascent's
downrange axis never moved" (flight bug 9). The same mistake under other
names was four more flight bugs: VGDISP = DELVSAB (P30 showed a zero
burn), two copies of TLAND (alarm 01406 at P63), VGBODY (N85 showed
zeros), TGO = DVCNTR +1 (`docs/TRANSLATION_GUIDE.md`, section 8). Each
compiled and passed every unit test; only a closed-loop flight
showed it.

## 5.4 Erasable by address

Some things outside the flight program do see erasable as numbered
memory: pinball's machine-address nouns (V01 N01 lets the crew display
any address), the V71/V72 uplink that lets the ground load blocks of
erasable, the downlink lists that send fixed addresses to the ground,
and a host such as NASSP that saves and restores a scenario by address.
For them the C keeps an explicit **address map**,
`erasable_map/address_map.c`: a table per rope from each address of
ERASABLE_ASSIGNMENTS to the typed owner (`agc_erase_read_ecadr`,
`agc_erase_write_ecadr`). Its NOTES name the three kinds of address:

```c
 *   - the flagwords (STATE +0.., ECADR 074 in both ropes, so FLAGWRD0..
 *     FLGWRD13 = 074..0111) map onto agc_flagword_get/set;
 *   - the named erasables in the address map below (k_map_csm / k_map_lm)
 *     map onto their typed accessors: REFSMMAT, TEPHEM, the state vectors, ...
 *   - every other address reads and writes a raw image of the 15-bit
 *     address space ...
```

An entry is an address, a length and an item: `{01735u, 18, EC_REFSMMAT}`
says that the 18 words at 1735 octal are REFSMMAT. Two details show how
much of the machine's word-level behaviour survives even here: a DP
loaded one word at a time keeps "the words exactly as given, signs free
to disagree", and where the AGC overlays one address with erasables of
different phases (EQUALS sharing), a read returns the owner that wrote
last. The map sits above every other module because it reaches them
all (`docs/ARCHITECTURE.md`, "The address map at the top"); calls into
it are not counted as upward calls.

## 5.5 Flagwords

Hundreds of yes/no facts -- "the Moon is the sphere of influence", "the
IMU is in use", "average-G is running", "V37 is not allowed now" -- are
single bits packed into 12 (CSM) or 14 (LM) **flagwords**, FLAGWRD0 to
FLGWRD13, at the start of the STATE block. The interpreter sets, clears
and tests them directly (SET, CLEAR, BON, BOFF), and basic code with
masks. The listing defines each flag by its bit number counted from the
start of STATE (ERASABLE_ASSIGNMENTS.agc C p.46):

```
MOONFLAG   =     003D    # MOON IS SPHERE OF    EARTH IS SPHERE OF
                         # INFLUENCE            INFLUENCE
MOONBIT    =     BIT12
```

In C a flag is an id `(flagword << 4) | bit`, named `AGC_FLAG_<NAME>`, with
the listing's "set" and "reset" meanings as its comment
(`include/agc/core/flags_common.h`):

```c
    /* MOONFLAG  FLAGWRD0 bit 12 (003D)
     * SET: MOON IS SPHERE OF INFLUENCE  RESET: EARTH IS SPHERE OF INFLUENCE */
    AGC_FLAG_MOONFLAG = 0x0C,
```

`agc_flag_test`, `agc_flag_set`, `agc_flag_clear` are BON/SET/CLEAR
(`core/state.c`):

```c
bool agc_flag_test(agc_flag_id_t f)
{
    return ((g_flagword[f >> 4] & agc_flag_mask(f)) != 0u);
}
```

Flags common to both ropes (same word and bit) are in
`include/agc/core/flags_common.h`; the others in `include/agc/core/flags_csm.h` and `include/agc/core/flags_lm.h`, whose
values were checked against the assembled symbol tables
(`docs/overview/README.md`, section 4). A few interpretive "switches"
that are not flagword bits live in `agc_interp_t.sw` (chapter 3).

## 5.6 Channels

The I/O channels (chapter 1) are a 15-bit image per channel in
`core/channels.c`. READ, WRITE, ROR, WOR, WAND are `agc_chan_read`,
`agc_chan_write`, `agc_chan_or`, `agc_chan_and`, and each bit has a name
`AGC_CH<octal channel>_<NAME>` from the LM's channel-bit description
(`include/agc/core/channels.h`). One hardware detail the reader must
keep in mind: the input channels 30-33 read **zero when the signal is
present**. The host side applies the inversion in one place
(`agc_chan_set_signal`: "Inverted channels: signal present = bit
cleared"), so flight code tests bits exactly as the listing does.

## 5.7 Pad loads

Erasables marked PL are loaded on the pad before launch: mission
constants that are not known when the rope is manufactured. Examples
in `core/pad_loads.c`: TLAND, the nominal landing time (LM); AOTAZ /
AOTEL, the LM telescope's detent angles; RTED1, a coefficient of the CSM
return-to-Earth polynomial. Others sit in the owners' structs (the SPS
thrust and mass-flow constants EK1VAL, EK2VAL, EK3VAL, FANG, EMDOT for
the CSM burn timing). The prolog of pad_loads.c says what they are: "a
pad load holds whatever word the ground stored".

In this project a pad load arrives in one of two ways: the harness's
`--pad` file sets named erasables through their setters in engineering
units (`sim/pad.c`), or a host writes them by address through the
address map (NASSP's scenario files). Missing a pad-loaded erasable in
the map was itself a flight bug: "the map put EK3VAL over FANG" gave an
SPS burn of 2.53 s for 2.21 s (flight bug 1, `docs/TIPS.md`, section 5).

## 5.8 What a restart preserves

Erasable memory keeps its contents through a restart; what a restart
does is decide what to *throw away* (chapter 7 has the mechanics):

* A **hardware restart** (GOJAM) and a **software restart** (BAILOUT,
  POODOO, V37) keep all of erasable, clear only the executive and the
  waitlist and the display system (STARTSB2,
  `exec/fresh_start.c:startsb2`), and re-start whatever the phase tables
  say was running.
* A **fresh start** (V36, or a phase table found corrupt) kills every
  restart group, sets MODREG to -0 and reinitialises the flagwords from
  the rope's table SWINIT -- keeping only a few bits
  (`exec/fresh_start.c:dofstart_common`, DOFSTART C pp.183-185 / L
  pp.211-213):

  ```c
      /* DOFSTART flagword initialisation from SWINIT (C p.193 / L p.225).
       * Kept through a fresh start: CSM NODOP01, REFSMFLG, CMOONFLG,
       * LMOONFLG, SURFFLAG; LM REFSMFLG, SURFFLAG, CMOONFLG, LMOONFLG, APSFLAG.
  ```

  REFSMFLG says the IMU's orientation matrix REFSMMAT is valid; the moon
  flags say which body each vehicle orbits; SURFFLAG says the LM is on
  the surface. Those facts outlive any program, so they survive.
* Neither clears the state vectors, REFSMMAT or any program erasable:
  "the AGC never clears program erasable on a fresh start"
  (`docs/TIPS.md`, section 5).

One limitation of the C belongs here. A host such as NASSP saves the
whole running AGC (registers, core sets, waitlist) and expects to resume
it. The C has no addresses for those, so a scenario load goes through
the address map for erasable and then *restarts* the computer, letting
the phase tables bring the programs back -- as a power transient would
(`docs/ARCHITECTURE.md`, "Known limitations"). An erasable a resumed
program needs and the map does not carry is lost at a load; five flight
bugs (1, 5, 12, 13, 14) were such gaps.

Next: [6. The crew interface](06-crew-interface.md).
