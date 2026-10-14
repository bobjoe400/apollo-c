# nav-integ -- INTEGRATION_INITIALIZATION + ORBITAL_INTEGRATION

## Scope

Comanche055 INTEGRATION_INITIALIZATION.agc (pp. 1309-1333) and
ORBITAL_INTEGRATION.agc (pp. 1334-1354), diffed against Luminary099
(pp. 1203-1226 / 1227-1248). Common text translated once into src/nav/;
vehicle differences behind `agc_state_vehicle()`; the one LM-only routine
that needs an LM-only flag (SETITCTR, AVEGFLAG) is in its own file.

Files: include/agc/nav/integration.h, src/nav/integration/internal.h (private),
src/nav/integration/init.c (entries, lock, TESTLOOP/A-PCHK/RECTOUT, driver),
src/nav/integration/jobs.c (STATEINT/STATINT1/STATEUP/ENDINT, AVETOMID,
MIDTOAV1/2, INTWAKEU), src/nav/integration/orbital_integration.c (Encke step, ACCOMP,
GAMCOMP, OBLATE, DIFEQ, W matrix, RECTIFY, origin change),
src/nav/integration/flags_csm.c (CSM-only flags V96ONFLG/AMOONFLG),
src/nav/integration/setitctr_lm.c (SETITCTR, AVEGFLAG), src/nav/integration/pending.h
(placeholder job/task ids), tests/nav/integration.c.

Also translated here (because they are the body of the STATEINT job):
STATEUP/DOINT/STATEND/NO-INT and THISVINT from FRESH_START_AND_RESTART.agc
(CM p. 209-210, LM p. 236-237).

Design notes:
* One shared integration context (agc_integ_shared_ctx()) mirrors the single
  set of AGC integration erasables guarded by INTSTALL; tests may use their
  own agc_integ_ctx_t.
* The integration is a state machine (enum + switch, bounded passes) in
  agc_integ_run(): TESTLOOP -> TIMESTEP -> ENDSTATE -> (CKMID2 | W columns ->
  RELOADSV) -> TESTLOOP ... -> A-PCHK -> RECTOUT -> INTEXIT. GOBAQUE returns
  to TESTLOOP. No recursion, no goto.
* Permanent state vectors (RRECTCSM..XKEPCSM, RRECTLEM..XKEPLEM, 42 words
  each) are kept here; RRECT/VRECT/TET parts are mirrored through the shared
  accessors agc_erase_[set_]rrectcsm/vrectcsm/tetcsm (and LEM).
* PBODY (X2) is an int 0 (earth) / 2 (moon); DIFEQCNT is the stage 0..2;
  COLREG is the W column 0..8; interpretive index arithmetic on NORM counts
  is done with positive counts n (the AGC stores -n).

## CM vs LM differences (normalised diff, comments/whitespace ignored)

| Place | Comanche055 | Luminary099 | C |
|-------|-------------|-------------|---|
| STATINT1 | CLEAR V96ONFLG | (absent) | vehicle test |
| A-PCHK | MIDFLAG -> CHKSWTCH/ORIGCHNG before ANDOUT | no origin check | vehicle test |
| INTEXIT | CLEAR MIDAVFLG | CLEAR STATEFLG | vehicle test |
| INTSTALL/INTWAKE | 3 stall areas (INTSTALL, ERASTAL1, ERASTAL2) | 1 area | area arg, LM area 0 only |
| OKTOGRAB | no INHINT | INHINT before ADS RASFLAG | agc_inhint in both (harmless) |
| AVETOMID | W -> this vehicle (SETCOAST) -> other (unless SURFFLAG) | W -> other (OTHERS) -> this | vehicle test |
| MIDTOAV1/2 | T-TO-ADD stepping, 5.6 s margin, TIMEDELT 1250 | TIMEDELT 2000, simpler MID2 | vehicle test |
| ACCOMP | RQVV not stored | RQVV stored; MODREG=27 forces RQVV | vehicle test |
| TIMESTEP | CHKSWTCH (RPQ via LUNPOS, sphere test) | R.V*DT test, LSPOS, INLUNCHK/LUNSPH | vehicle test |
| SETITCTR | absent | ITERCTR 20 or 5 (AVEGFLAG) then LAMBERT | setitctr_lm.c |
| QUALITY1/2 | in RT8_OP_CODES (same code) | here | translated here |
| Constant pool MUEARTH.. | here | CONTROLLED_CONSTANTS (same values) | one static table |
| RME/RMM | 7178165 m / 2538.09 km | both = DPPOSMAX (MIDFLAG never set) | per-vehicle consts |

## Design table

| AGC label (page CM) | In | Out | C function |
|---|---|---|---|
| INTSTALL/ALLSTALL/OKTOGRAB (1324) | RASFLAG | grab or JOBSLEEP | agc_integ_install(area, resume_job) |
| INTWAKE/INTWAKE0/INTWAKE1/WAKE (1325) | REINTFLG | JOBWAKE sleepers, release | agc_integ_intwake(area) |
| CSMPREC/LEMPREC/CSMCONIC/LEMCONIC (1318) | TDEC1 | RATT.. | agc_integ_csmprec() etc. |
| INTEGRVS (1319) | RCV VCV TET, MOONFLAG | RATT.. | agc_integ_integrvs() |
| INTEGRV/INTEGRV1/INTEGRV2/ALOADED (1320) | flags, TDEC1 | RATT.. | agc_integ_integrv() |
| A-PCHK/ANDOUT/PHEXIT/RECTOUT/INTEXIT (1320) | | RATT VATT TAT RATT1 VATT1 MU(P) X1 X2 | static a_pchk(), rectout(), intexit() |
| RVCON (1321) | TDEC TET | conic | static rvcon() |
| TESTLOOP/NORFINAL/DT/2COMP/MAXDT/GETMAXDT/USEMAXDT/POOHCHK (1323) | | DT/2 (B-19 cs) | agc_integ_testloop() |
| ATOPCSM/ATOPLEM/PTOACSM/PTOALEM/MOVE* (1314) | | perm <-> temp | agc_integ_atop/ptoa() |
| USEPIOS (1316) | RLS, SURFFLAG | LM surface state | static usepios() |
| SETIFLGS/STATEINT/STATINT1/ENDINT/NOINT (1313) | | | agc_task_stateint(), agc_job_statint1() |
| AVETOMID/SETCOAST/OTHERS/INT/W (1327) | RN VN PIPTIME | perm SV | agc_integ_avetomid() |
| MIDTOAV1/MIDTOAV2/ENTMID*/CKMID2/TIMEINC/MID2/NOTIME (1329) | TDEC1 | RN1 VN1 PIPTIME1 | agc_integ_midtoav1/2(), static ckmid2() |
| INTWAKEU (1332) | UPSVFLAG | perm SV | agc_integ_intwakeu() |
| GRP2PC (1333) | | phase 04022 | static grp2pc() |
| KEPPREP (1334) | RCV VCV TAU TC XKEP | XKEPNEW, then KEPLER | agc_integ_kepprep() |
| FBR3 (1336) | DT/2 | TAU TET, conic advance | static fbr3() |
| ACCOMP/GETRPSV (1337) | ALPHAV RCV | FV, VECTAB | static accomp() |
| GAMCOMP (1338) | ALPHAV ALPHAM BETAV X1 X2 | FV += | static gamcomp() |
| GOBAQUE/INT-ABRT (1340) | | re-rectify / POODOO 00430 | static gobaque() |
| OBLATE/COMTERM/COSPHIE (1341) | ALPHAV ALPHAM | FV += J2..J4 | static oblate() |
| QUALITY1/2/3 (RT8 1516 / 1342) | URPV | lunar J22/C31 | static quality() |
| NBRANCH/DIFEQTAB (1344) | DIFEQCNT | stage dispatch | loop in agc_integ_timestep() |
| TIMESTEP/RECTEST/CALLRECT/INTGRATE/DIFEQ0 (1344) | | | agc_integ_timestep() |
| CHKSWTCH/RPQOK/EARSPH/DOSWITCH (1345) | RPQV | sphere test | static chkswtch() |
| ORIGCHNG (1346) | | origin switch | static origchng() |
| RECTIFY/MINIRECT (1347) | TDELTAV TNUV | RRECT VRECT | agc_integ_rectify(), agc_integ_minirect() |
| DIFEQ+0/+1/+2/DIFEQCOM (1348) | FV H | YV ZV | static difeq_*() |
| ENDSTATE/AMOVED/NEXTCOL/RELOADSV/WMATEND (1349) | | W columns | static endstate(), wmatrix_columns() |
| DOW../DOW..1 (1352) | VECTAB | FV for W | static dow() |
| SETITCTR (LM 1245) | AVEGFLAG | ITERCTR | agc_integ_setitctr() |
| Constant pools (1353-1354) | | | static const in orbital_integration.c |

## Deviations

* Control flow: the AGC's GOTO/CALL web (TESTLOOP, TIMESTEP, NBRANCH ->
  DIFEQ+k -> DIFEQCOM -> FBR3 -> KEPPREP -> ACCOMP, GOBAQUE -> TESTLOOP,
  RELOADSV -> INTEGRV2 -> ALOADED -> TESTLOOP) is a state machine
  (agc_integ_run, one switch, AGC_INTEG_MAX_PASSES = 60000 passes; the AGC
  has no bound). Exceeding it releases INTSTALL and returns NO_CONVERGE.
* A failure of a called C routine (ephemeris/planetary/conic returning an
  error; cannot happen in the AGC) releases INTSTALL via INTEXIT and
  returns that routine's status (ctx->fail). INT-ABRT (POODOO 00430)
  leaves the lock held, as the AGC.
* Push-list outputs (RATT..MU(P), X1, X2) are ctx->out; out.valid is false
  when INTEXIT is reached without RECTOUT (QUITFLAG, backward request,
  P00 state younger than 4 steps) -- the AGC leaves stale push-list data.
* INTSTALL sleepers: the AGC JOBWAKEs every job whose LOC = WAKESTAL; here
  each sleeper names its own continuation job (resume_job) and the lock
  keeps a per-area list that INTWAKE/ERASWAKn wake (bounded by the core-set
  count). A caller with no continuation (AGC_JOB_NONE) is not put to
  sleep (BUSY only); a full list returns FULL.
* Restart (review-nav-integ N12): a group-2 restart reruns the running job
  segment from its top (exec R11) instead of resuming after the GRP2PC.
  INTSTALL therefore takes over an area a restart left with REINTFLG set
  and INTFLBIT clear (else the rerun segment slept for ever), STATINT1
  integrates to a later TDEC1, and W columns not yet integrated when the
  restart came are re-integrated from the moved state (one step behind).
* LM OKTOGRAB does INHINT without RELINT; MIDTOAV returns in basic with
  INHINT set: here interrupts are re-enabled before returning.
* (review) atop(): after an A-PCHK origin change the AGC reaches ATOPCSM
  with X2 = ORIGEX (27205); SVDWN then stores RN = VN = 0.  Modelled
  (AGC_INTEG_X2_ORIGEX), no longer a deviation; see review-nav-integ.md N2.
* Jobs re-entered after JOBSLEEP: STATINT1/STATEUP keep TDEC1 in module
  statics (their VAC push list). For MIDTOAV1/2 (LM INTSTALL comes after
  the time checks) and AVETOMID a BUSY return means the caller re-runs the
  routine from the start (time is re-read; NOTIME may alarm again).
* OBLATE moon branch calls MOONMX for the M matrix instead of reusing the
  MMATRIX R-TO-RP left in the push list (same TET, same matrix).
* QUALITY3: OVFIND is reset after RP-TO-R explicitly ("OVERFLOW INDICATOR
  RESET IN RP-TO-R") since the nav-aux C routine need not do it.
* (review N8) DMP is DMPSUB without rounding and DMPR rounds, as the
  listing says at every site; GAMCOMP, COMTERM, QUALITY1-3 and the OBLATE
  tail run on AGC words (agc_mpac_*, local vec_words_t for VXSC / VAD /
  VSL / VSR incl. VRIGHT2), so MPAC+2 after SR / DOT / DMP and sign-
  disagreeing DSU / BDSU / DAD results take part exactly as in the AGC.
  Not word level: TDELTAV / TNUV between steps (agc_integ_sv_t values,
  shared with the nav lanes): 1 LSB in VATT after RECTIFY's VSR (N9).
* LM LUNVEL is a bare RVQ in Luminary099 (MPAC unchanged); unreachable
  (MIDFLAG never set in the LM); C returns a zero vector.
* TRKMKCNT = 0 in AVETOMID goes to a module static until the accessor
  exists (see Requested core additions).
* STATEUP / THISVINT live in FRESH_START_AND_RESTART.agc but are translated
  here as the body of the STATEINT job.

## Stubs

None. Every routine in the two files is translated; the LM-only MIDFLAG
paths (TIMESTEP R.V test/LUNSPH/INLUNCHK, ACCOMP RQVV) are translated even
though Luminary099 never reaches them.

## Tests written

tests/nav/integration.c (compiles with the project flags; 12 tests, 177
checks). Run against the current core/exec/nav objects (dispatch, SVDWN and
the DSKY entry points shimmed in the scratchpad since other lanes are not
linkable yet): all pass. Results:

| test | result |
|---|---|
| conic LEO 10 min vs analytic | 46.6 m (Kepler: nav-conic alone gives the same 38-47 m) |
| two-body Encke (LM, 100000 km) 10 min vs analytic | 17.6 m |
| Encke LEO 10 min with J2..J4 vs RK4 reference | 46.2 m (J2 effect itself 2406 m) |
| W column (100 m) 10 min vs finite-difference STM | 2 cm |
| lunar orbit 1 h, J2+J3+J22+C31 vs reference | 19.4 m (omitting C31/J22/J3 in the reference: 675/1031/177 m) |
| CSM 60000 km 6 h, moon + sun third bodies | 41.7 m (without moon+sun 4263 m; without sun 1136 m) |
| STATEINT (P00, 3000 s) | TETCSM = TETLEM = 284416 cs (11 steps of 25856 cs; last partial step left) |
| MIDTOAV1, NOTIME slip, AVETOMID, INTWAKEU, lock | pass |

The reference model's constants are the AGC's own: J2REQSQ = J2 mu Re^2,
2J3RE/J2 = J3 Re/J2, J4REQ/J3 = J4 Re/J3 (implying J3 = -2.3E-6, J4 =
-1.8E-6 for the earth), E3J22R2M = 3 J22 R^2 mu at B58, E32C31RM =
3/2 C31 R^3 mu at B80 (scale derived from the QUALITY3 comments and
confirmed by the test).

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/nav/integration/*.c`._

Encke integration of a state vector's deviation from a conic, plus the
jobs that keep the permanent state vectors current. Both vehicles; the
flags are the AGC's integration flags in FLAGWRD3 etc. (STATEFLG,
VINTFLAG, DIM0FLAG, D6OR9FLG, INTYPFLG, MIDFLAG, MOONFLAG, RPQFLAG,
PRECIFLG, NEWIFLG, MIDAVFLG, AVEMIDSW, QUITFLAG, REINTFLG).

* **Entrances** (`src/nav/integration/init.c`): CSMPREC / LEMPREC /
  CSMCONIC / LEMCONIC (`agc_integ_csmprec`, `agc_integ_lemprec`,
  `agc_integ_csmconic`, `agc_integ_lemconic` -> `special_entry`: precision
  or conic, no W, to TDEC1), INTEGRVS (`agc_integ_integrvs`: the caller's
  RCV/VCV/TET) and INTEGRV (`agc_integ_integrv`: the permanent state of
  VINTFLAG). `agc_integ_setiflgs` sets the flag set the caller wants.
* **INTSTALL lock** (`agc_integ_install`, `agc_integ_area_busy`): a busy
  stall area (RASFLAG bits) puts the job to sleep until INTWAKE
  (`agc_integ_intwake`: with REINTFLG set a phase change happened since
  INTSTALL, so PHASCHNG 04022 keeps the caller's QPRET). DEVIATION: after
  a restart the rerun job segment takes the area over instead of sleeping
  on it.
* **Driver** (`agc_integ_run`, TESTLOOP .. INTEXIT, one bounded loop over
  the labels; AGC_INTEG_MAX_PASSES is NO_CONVERGE, C only): `testloop`
  ends on QUITFLAG, sets MIDFLAG when |RCV| is beyond RME/RMM, chooses
  the step (`norfinal`: DT = .3 sqrt(R^3/MU) truncated to a multiple of 128 cs, at most 4000 s);
  `poohchk` ends when |DT/2| < 3 cs, and in P00 (MODREG = 0) stops before
  the last partial step; `a_pchk` writes the permanent state back when
  STATEFLG (ATOPCSM / ATOPLEM, GRP2PC phase change) and handles REINTFLG;
  `rectout` rectifies and leaves the output in the push list; `intexit`
  clears MIDAVFLG (CM) / STATEFLG (LM), AVEMIDSW, PRECIFLG and wakes the
  stall area.
* **One step** (`orbital_integration.c:agc_integ_timestep`): `rectest`
  rectifies (`agc_integ_rectify`: a new conic through the actual state)
  when |TDELTAV| or |TNUV| reaches 3/4 of scale or |TDELTAV| >= .01 |RCV|;
  with MIDFLAG the sphere-of-influence test `agc_integ_chkswtch`
  (CSM: moon primary RSPHERE - |R|, earth primary |R - RPQ| ...) may change
  origin (`agc_integ_origchng`, MOONFLAG); `lm_sphere` is Luminary's form,
  never reached because Luminary099 never sets MIDFLAG. `integrate_step`
  runs three Nystrom stages (`nystrom_start`, `nystrom_stage`,
  `nystrom_end`) with ACCOMP (`accomp`: third body by DIM0FLAG/MOONFLAG,
  OBLATE earth J2-J4 or lunar QUALITY1-3). An overflow goes to GOBAQUE
  (`gobaque`): back the conic up by H, rectify and retry; with no deviation
  left it is POODOO 00430 (INT-ABRT). W columns (`agc_integ_wcolumns`):
  6x6 (D6OR9FLG clear) or 9x9; W overflow is ALARM 00421 (WMATEND) and W is
  marked invalid (ORBWFLAG / RENDWFLG cleared).
* **Jobs** (`jobs.c`): `agc_task_stateint` (STATEINT, every 600 s through
  `endint`'s LONGCALL) starts `agc_job_stateint_update` (STATINT1: QUITFLAG
  set by V96 = NOINT; else TDEC1 = now, INTSTALL, `stateup` integrates the
  CSM permanent state and its W when ORBWFLAG / RENDWFLG / SURFFLAG make it
  valid, then the LM's (`stateup_lm_state`)). Going to thrusting:
  `agc_integ_midtoav_at_time` (MIDTOAV1) integrates this vehicle to TIG-30
  and `agc_integ_midtoav_check` (CKMID2) carries on while TDEC is at least
  TIMEDELT (`timedelt`: 12.5 s CM / 20 s LM) in the future; too late is
  NOTIME (`notime`: ALARM 01703, CALLOC +2 return). Coming back,
  `agc_integ_avetomid` (AVETOMID) copies RN/VN/PIPTIME into the permanent
  state (`setcoast` CM / `others` LM) after INT/W when W is valid.
* **Others**: `agc_integ_perm_get` / `agc_integ_perm_set` (the 42-word
  permanent blocks, also the ECADR map's path for NASSP scenario
  restores), `setitctr_lm.c:agc_integ_setitctr` (LAMBERT ITERCTR 5 with
  AVEGFLAG, else 20), `svdwn.c:agc_integ_svdwn` (downlink copy).

### Decision points by function

52 of the lane's 106 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_integ_csm_resume_idle_integration` (flags_csm.c) | STATINT1 | - | =IDLE_INTEGRATION_INHIBITED | - | - | - |
| `agc_integ_csm_amoonflg` (flags_csm.c) | SETCOAST | AMOONFLG | - | - | - | - |
| `agc_integ_csm_set_amoonflg` (flags_csm.c) | MIDTOAV1 | - | =AMOONFLG | - | - | - |
| `agc_integ_setiflgs` (init.c) | SETIFLGS | - | =STATEFLG =INTYPFLG =DIM0FLAG =D6OR9FLG | - | - | - |
| `agc_integ_thisvint` (init.c) | THISVINT | - | =VINTFLAG | - | - | - |
| `atop` (init.c) | ATOPCSM ATOPLEM | MOONFLAG | - | - | - | - |
| `agc_integ_ptoacsm` (init.c) | PTOACSM | CMOONFLG | - | - | - | - |
| `usepios` (init.c) | USEPIOS | - | =MOONFLAG | - | - | - |
| `agc_integ_ptoalem` (init.c) | PTOALEM | LMOONFLG SURFFLAG | - | - | - | - |
| `poohchk` (init.c) | POOHCHK | NEWIFLG PRECIFLG | =NEWIFLG | - | - | - |
| `testloop` (init.c) | TESTLOOP | QUITFLAG | =STATEFLG =MIDFLAG | - | - | - |
| `a_pchk` (init.c) | A-PCHK ANDOUT PHEXIT | MIDFLAG STATEFLG VINTFLAG | =STATEFLG =REINTFLG | - | - | - |
| `rectout` (init.c) | RECTOUT | MOONFLAG | - | - | - | - |
| `intexit` (init.c) | INTEXIT | - | =MIDAVFLG =STATEFLG =AVEMIDSW =PRECIFLG | - | - | - |
| `aloaded` (init.c) | ALOADED | INTYPFLG | - | - | - | - |
| `load_permanent_state` (init.c) | INTEGRV2 | VINTFLAG | - | - | - | - |
| `wmatrix` (init.c) | ENDSTATE | VINTFLAG | =REINTFLG | - | - | - |
| `agc_integ_run` (init.c) | TESTLOOP INTEXIT | - | - | POODOO 0430 | - | - |
| `agc_integ_integrate_permanent` (init.c) | INTEGRV1 | - | =RPQFLAG =NEWIFLG | - | - | - |
| `special_entry` (init.c) | CSMPREC LEMPREC IFLAGP | - | =VINTFLAG =PRECIFLG =DIM0FLAG =INTYPFLG | - | - | - |
| `agc_integ_integrvs` (init.c) | INTEGRVS | MOONFLAG | =PRECIFLG =DIM0FLAG =NEWIFLG =RPQFLAG | - | - | - |
| `agc_task_stateint` (jobs.c) | STATEINT | - | - | - | - | job stateint_update |
| `endint` (jobs.c) | ENDINT | - | =STATEFLG | - | - | task stateint |
| `stateup_lm_state` (jobs.c) | STATEUP | RENDWFLG | =VINTFLAG =DIM0FLAG =D6OR9FLG =PRECIFLG =NODOFLAG | - | - | - |
| `stateup` (jobs.c) | STATEUP | ORBWFLAG RENDWFLG SURFFLAG | =VINTFLAG =DIM0FLAG =PRECIFLG =NODOFLAG | - | - | - |
| `state_integration_grab` (jobs.c) | STATINT1 | - | =NODOFLAG | - | - | - |
| `agc_job_stateint_update` (jobs.c) | STATINT1 | QUITFLAG | =QUITFLAG | - | - | - |
| `int_w` (jobs.c) | INT/W | - | =DIM0FLAG =AVEMIDSW =D6OR9FLG =VINTFLAG | - | - | - |
| `setcoast` (jobs.c) | SETCOAST | SURFFLAG | =CMOONFLG =VINTFLAG | - | - | - |
| `others` (jobs.c) | OTHERS | LMOONFLG | =VINTFLAG | - | - | - |
| `agc_integ_avetomid` (jobs.c) | AVETOMID | ORBWFLAG RENDWFLG | - | - | - | - |
| `notime` (jobs.c) | NOTIME | - | =MID1FLAG | ALARM 1703 | - | - |
| `midtoav_integrate` (jobs.c) | ENTMID1 | - | =DIM0FLAG =INTYPFLG =MIDAVFLG | - | - | - |
| `agc_integ_midtoav_at_time` (jobs.c) | MIDTOAV1 | - | =MID1FLAG | - | - | - |
| `agc_integ_midtoav_now` (jobs.c) | MIDTOAV2 | - | =MID1FLAG | - | - | - |
| `agc_integ_midtoav_check` (jobs.c) | CKMID2 MID2 | MID1FLAG | - | - | - | - |
| `agc_integ_intwakeu` (jobs.c) | INTWAKEU | - | =MOONFLAG =ORBWFLAG =RENDWFLG | - | - | - |
| `agc_integ_setmoon` (orbital_integration.c) | SETMOON CLRMOON | - | =MOONFLAG | - | - | - |
| `agc_integ_kepprep` (orbital_integration.c) | KEPPREP | MOONFLAG | - | - | - | - |
| `gobaque` (orbital_integration.c) | GOBAQUE INT-ABRT | - | =RPQFLAG | - | - | - |
| `third_body` (orbital_integration.c) | ACCOMP | DIM0FLAG MOONFLAG | =RPQFLAG | - | - | - |
| `oblate` (orbital_integration.c) | OBLATE | MOONFLAG | - | - | - | - |
| `accomp` (orbital_integration.c) | ACCOMP | DIM0FLAG MIDFLAG | - | - | - | - |
| `dow` (orbital_integration.c) | - | MIDFLAG MOONFLAG | - | - | - | - |
| `agc_integ_wcolumns` (orbital_integration.c) | AMOVED NEXTCOL DIFEQ+2 | D6OR9FLG | =JSWITCH =DIM0FLAG =ORBWFLAG =RENDWFLG =STATEFLG | ALARM 0421 | - | - |
| `lm_sphere` (orbital_integration.c) | TIMESTEP | MOONFLAG RPQFLAG | - | - | - | - |
| `integrate_step` (orbital_integration.c) | INTGRATE ENDSTATE | DIM0FLAG MIDAVFLG | =JSWITCH | - | - | - |
| `agc_integ_timestep` (orbital_integration.c) | TIMESTEP RECTEST CALLRECT | MIDFLAG | - | - | - | - |
| `agc_integ_chkswtch` (orbital_integration.c) | CHKSWTCH RPQOK EARSPH | MOONFLAG RPQFLAG | - | - | - | - |
| `agc_integ_origchng` (orbital_integration.c) | ORIGCHNG | MOONFLAG | - | - | - | - |
| `agc_integ_setitctr` (setitctr_lm.c) | SETITCTR | AVEGFLAG | - | - | - | - |
| `agc_integ_svdwn` (svdwn.c) | SVDWN1 SVDWN2 | AVEMIDSW | - | - | - | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `modreg` | MODREG major mode (program) | 2 | 0 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 4 | 1 |
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 1 | 0 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 1 | 1 |
| `rrectcsm` | permanent CSM position, meters, scale 2^29 earth / 2^27 moon (RRECTCSM) | 1 | 1 |
| `rrectlem` | permanent LM position, meters, scale 2^29 earth / 2^27 moon (RRECTLEM) | 1 | 1 |
| `tephem` | time of the ephemeris epoch (TEPHEM = TIMSUBO, TP, centiseconds, scale 2^42: L "CSEC B-42 (TRIPLE PRECISION)"; P11 sets it at liftoff) | 2 | 0 |
| `tetcsm` | time of the permanent CSM state vector, centiseconds, scale 2^28 (TETCSM) | 2 | 1 |
| `tetlem` | time of the permanent LM state vector, centiseconds, scale 2^28 (TETLEM) | 1 | 1 |
| `trkmkcnt` | TRKMKCNT: rendezvous tracking mark count (noun 45 R1), whole number. | 0 | 1 |
| `vn` | this vehicle's velocity, m/cs, scale 2^7 (earth) or 2^5 (moon) (VN, B(6)PRM) | 1 | 1 |
| `vrectcsm` | permanent CSM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTCSM) | 1 | 1 |
| `vrectlem` | permanent LM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTLEM) | 1 | 1 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
