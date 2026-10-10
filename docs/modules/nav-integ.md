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
* integration_init.c (INTSTALL/INTWAKE, entries, PTOA/ATOP, TESTLOOP, A-PCHK, RECTOUT, INTEXIT, RVCON, driver): TODO
* integration_jobs.c (STATEINT..ENDINT, AVETOMID, MIDTOAV, INTWAKEU, GRP2PC): TODO
* orbital_integration.c: DONE (compiles) -- KEPPREP wrapper agc_integ_kepprep, FBR3 fbr3, GOBAQUE gobaque, GAMCOMP gamcomp/gamcomp_rho, ACCOMP accomp/third_body, OBLATE oblate/moon_axes/legendre_terms/comterm_tvec, QUALITY1-3 quality2/quality3, DOW.. dow/dow1, DIFEQ0/+0/+1/+2/DIFEQCOM difeq0/difeq01/difeq2, ENDSTATE state_step, AMOVED/NEXTCOL/WMATEND agc_integ_wcolumns/wmatend, RECTEST rectest, TIMESTEP agc_integ_timestep (+ LM lm_sphere), CHKSWTCH agc_integ_chkswtch, ORIGCHNG agc_integ_origchng, RECTIFY/MINIRECT agc_integ_rectify/minirect, SETMOON/CLRMOON agc_integ_setmoon
* integration_csm.c / integration_lm.c: TODO
* tests/test_integration.c: TODO

## Deviations

(filled in as code is written)

## Stubs

## Requested core additions

## Requested jobs-tasks

## Tests written

## Oracle cases

## Open questions
