# nav-integ -- INTEGRATION_INITIALIZATION + ORBITAL_INTEGRATION

## Scope

Comanche055 INTEGRATION_INITIALIZATION.agc (pp. 1309-1333) and
ORBITAL_INTEGRATION.agc (pp. 1334-1354), diffed against Luminary099
(pp. 1203-1226 / 1227-1248). Common text translated once into src/nav/;
vehicle differences behind `agc_state_vehicle()`; the one LM-only routine
that needs an LM-only flag (SETITCTR, AVEGFLAG) is in its own file.

Files: include/agc/agc_integ.h, src/nav/integ_internal.h (private),
src/nav/integration_init.c (entries, lock, TESTLOOP/A-PCHK/RECTOUT, driver),
src/nav/integration_jobs.c (STATEINT/STATINT1/STATEUP/ENDINT, AVETOMID,
MIDTOAV1/2, INTWAKEU), src/nav/orbital_integration.c (Encke step, ACCOMP,
GAMCOMP, OBLATE, DIFEQ, W matrix, RECTIFY, origin change),
src/nav/integration_csm.c (CSM-only flags V96ONFLG/AMOONFLG),
src/nav/integration_lm.c (SETITCTR, AVEGFLAG), src/nav/integ_pending.h
(placeholder job/task ids), tests/test_integration.c.

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
| SETITCTR | absent | ITERCTR 20 or 5 (AVEGFLAG) then LAMBERT | integration_lm.c |
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

## Progress

* design: DONE (this note)
* agc_integ.h: DONE include/agc/agc_integ.h
* integ_internal.h / integ_pending.h: DONE src/nav/
* integration_init.c: DONE (compiles) -- INTSTALL/ERASTAL1/2/OKTOGRAB agc_integ_install, INTWAKE agc_integ_intwake, INTWAKE0/1 agc_integ_intwake0, ERASWAK1/2 agc_integ_eraswake, WAKE wake, CSMPREC/LEMPREC/CSMCONIC/LEMCONIC special_entry, INTEGRVS agc_integ_integrvs, INTEGRV agc_integ_integrv, INTEGRV1 agc_integ_integrv1, INTEGRV2 integrv2, ALOADED aloaded, ATOPCSM/ATOPLEM atop, PTOACSM/PTOALEM agc_integ_ptoacsm/ptoalem, USEPIOS usepios, TESTLOOP testloop, NORFINAL norfinal, DT/2COMP..USEMAXDT dt2comp, POOHCHK poohchk, A-PCHK/ANDOUT/PHEXIT a_pchk, RECTOUT rectout, INTEXIT intexit, RVCON rvcon, ENDSTATE W part wmatrix, RELOADSV reload, driver agc_integ_run, SETIFLGS, THISVINT, GRP2PC, permanent SV agc_integ_perm_get/set
* integration_jobs.c: DONE (compiles) -- STATEINT agc_task_stateint, STATINT1 agc_job_statint1(+_stall), STATEUP stateup/stateup_lm(+agc_job_stateup_stall), ENDINT endint, NOINT (in statint1), AVETOMID agc_integ_avetomid (INT/W int_w, SETCOAST setcoast_csm, OTHERS others_lm, MOVATHIS move_this), MIDTOAV1/2 agc_integ_midtoav1/2, ENTMID1 entmid1, NOTIME notime, CKMID2/MID2 agc_integ_ckmid2, TIMEINC timeinc, INTWAKEU agc_integ_intwakeu
* orbital_integration.c: DONE (compiles) -- KEPPREP wrapper agc_integ_kepprep, FBR3 fbr3, GOBAQUE gobaque, GAMCOMP gamcomp/gamcomp_rho, ACCOMP accomp/third_body, OBLATE oblate/moon_axes/legendre_terms/comterm_tvec, QUALITY1-3 quality2/quality3, DOW.. dow/dow1, DIFEQ0/+0/+1/+2/DIFEQCOM difeq0/difeq01/difeq2, ENDSTATE state_step, AMOVED/NEXTCOL/WMATEND agc_integ_wcolumns/wmatend, RECTEST rectest, TIMESTEP agc_integ_timestep (+ LM lm_sphere), CHKSWTCH agc_integ_chkswtch, ORIGCHNG agc_integ_origchng, RECTIFY/MINIRECT agc_integ_rectify/minirect, SETMOON/CLRMOON agc_integ_setmoon
* integration_csm.c / integration_lm.c: DONE (SETITCTR agc_integ_setitctr)
* assertion density top-up: DONE (check_rules: no warnings on the nav-integ files)
* scratch run of the test: DONE (see Tests written)
* tests/test_integration.c: DONE (12 tests, 177 checks; all pass against the current core/exec/nav objects with a scratch link shim for dispatch/svdwn/dsky)

## Deviations

* Control flow: the AGC's GOTO/CALL web (TESTLOOP, TIMESTEP, NBRANCH ->
  DIFEQ+k -> DIFEQCOM -> FBR3 -> KEPPREP -> ACCOMP, GOBAQUE -> TESTLOOP,
  RELOADSV -> INTEGRV2 -> ALOADED -> TESTLOOP) is a state machine
  (agc_integ_run, one switch, AGC_INTEG_MAX_PASSES = 60000 passes; the AGC
  has no bound). Exceeding it releases INTSTALL and returns NO_CONVERGE.
* A failure of a called C routine (ephemeris/planetary/conic returning an
  error; cannot happen in the AGC) releases INTSTALL via INTEXIT and
  returns BAD_ARG. INT-ABRT (POODOO 00430) leaves the lock held, as the AGC.
* Push-list outputs (RATT..MU(P), X1, X2) are ctx->out; out.valid is false
  when INTEXIT is reached without RECTOUT (QUITFLAG, backward request,
  P00 state younger than 4 steps) -- the AGC leaves stale push-list data.
* INTSTALL sleepers: the AGC JOBWAKEs every job whose LOC = WAKESTAL; here
  each sleeper names its own continuation job (resume_job) and the lock
  keeps a per-area list that INTWAKE/ERASWAKn wake (bounded by the core-set
  count). A job whose JOBSLEEP is refused (not inside a job) is not queued.
* LM OKTOGRAB does INHINT without RELINT; MIDTOAV returns in basic with
  INHINT set: here interrupts are re-enabled before returning.
* atop(): after ORIGCHNG the AGC reaches ANDOUT/ATOPCSM with X2 = ORIGEX (a
  return address) as the SVDWN shift; C passes PBODY.
* Jobs re-entered after JOBSLEEP: STATINT1/STATEUP keep TDEC1 in module
  statics (their VAC push list). For MIDTOAV1/2 (LM INTSTALL comes after
  the time checks) and AVETOMID a BUSY return means the caller re-runs the
  routine from the start (time is re-read; NOTIME may alarm again).
* OBLATE moon branch calls MOONMX for the M matrix instead of reusing the
  MMATRIX R-TO-RP left in the push list (same TET, same matrix).
* QUALITY3: OVFIND is reset after RP-TO-R explicitly ("OVERFLOW INDICATOR
  RESET IN RP-TO-R") since the nav-aux C routine need not do it.
* DMP followed by DP use is agc_dp_mul (rounded) except where the AGC
  normalises or shifts the TP product (GAMCOMP NORM 30D, OBLATE R^4, DOW..
  R^3, NORFINAL, COMTERM DMP SL1R), which keep the TP via agc_dp_mul_tp,
  agc_tp_add and a local TP NORM. NORFINAL's SR1R after DMP rounds twice
  (TP->DP, then SRR 1): at most 1 LSB.
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

## Requested core additions

* `void agc_erase_set_trkmkcnt(agc_sp_t v);` (TRKMKCNT, B(1)PRM, owner
  prog-rendz) -- AVETOMID zeroes it. Meanwhile `trkmkcnt_pending` static in
  integration_jobs.c.
* Layering: ATOPCSM/ATOPLEM call `agc_align_svdwn()` (SVDWN1/SVDWN2,
  agc_align.h, src/programs/align_locsam.c), i.e. nav calls programs.
  Request: move SVDWN to nav (e.g. `agc_integ_svdwn`) or core.
* agc_state.h comment fix: RN/VN are always 2^29 m / 2^7 m/cs (AVERAGEG);
  SETCOAST shifts them left 2 for the moon scale (VSR* 0,2 with X2 = 2).
* agc_nav.h: `agc_conic_kepprep()` must take the current conic point
  (RCV, VCV at TC) in in->rrect/in->vrect -- KEPPREP reads RCV/VCV, not
  RRECT/VRECT; agc_integ_kepprep() passes them that way (works with the
  current conic_kepler.c: all integration tests pass).
* E3J22R2M / E32C31RM (SP pad loads at FANG+2/+3, CM P40 pad-load area) are
  module statics here with `agc_integ_set_lunar_coeffs()`; if the uplink /
  pad-load code needs them, promote to shared accessors.

## Requested jobs-tasks

Placeholders in src/nav/integ_pending.h (range 0x0F):

| id | AGC | body | how started |
|---|---|---|---|
| AGC_JOB_STATINT1 | STATINT1 | agc_job_statint1 | FINDVAC PRIO5 from task STATEINT |
| AGC_JOB_STATINT1_STALL | STATINT1 at INTSTALL +1 | agc_job_statint1_stall | JOBWAKE (INTWAKE) |
| AGC_JOB_STATEUP_STALL | STATEUP 2nd INTSTALL +1 | agc_job_stateup_stall | JOBWAKE (INTWAKE) |
| AGC_TASK_STATEINT | STATEINT | agc_task_stateint | LONGCALL 600SECS from ENDINT (and fresh start) |

Callers of the CSMPREC/... entrances, AVETOMID and MIDTOAV1/2 pass their own
continuation job (resume_job) for the INTSTALL sleep.

## Tests written

tests/test_integration.c (compiles with the project flags; 12 tests, 177
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

## Oracle cases

Proposed for tools/oracle (Comanche055 unless noted; inputs poked into the
erasables, interpretive entry through INTPRET):
1. RECTIFY with nonzero TDELTAV/TNUV, PBODY 0 and 2: RRECT VRECT RCV VCV.
2. GAMCOMP primary term (X1 = X2 = 0) for a LEO RCV and a 1000 m ALPHAV:
   FV bit pattern (checks ASCALE and the NORM bookkeeping).
3. GAMCOMP sun term (X1 = 8, X2 = 4) and moon term (X1 = 4, X2 = 2).
4. OBLATE earth (COSPHIE) at 45 deg latitude: FV.
5. OBLATE moon with QUALITY1-3 (needs MOONMX pad loads): FV.
6. One full TIMESTEP (INTGRATE..ENDSTATE) from a fixed state: TDELTAV,
   TNUV, RCV, VCV, TET, TC, XKEP (KEPLER included).
7. TESTLOOP/NORFINAL/DT/2COMP: DT/2 and 12D for r = 6678 km, 60000 km,
   1850 km (moon).
8. DOW.. for one W column: FV.
9. Luminary099: CSMPREC over 10 minutes end to end (RATT VATT TAT).

## Open questions

* RN/VN scale comment in agc_state.h (see above).
* The integration is synchronous in C: a caller that holds INTSTALL across
  a JOBSLEEP elsewhere would block STATEINT exactly as the AGC; no
  pre-emption inside an integration is modelled (CHANG1/CHANG2 are not
  called inside it).
* PHASCHNG codes are passed as the full octal word (04022, 00052, 020032,
  00002, 04026) to agc_phase_change(group, code); type C 04022 restarts
  "at the return address" which the C model cannot resume mid-integration.
* VPROJ argument order assumed agc_vec_proj(ip, mpac, x) = (mpac.x) x
  (DOW..1 passes ALPHAV, unit R); the W test confirms the result.
