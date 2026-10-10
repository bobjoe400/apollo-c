# prog-align -- IMU alignment programs, optics marking, geometry, S-band, R63, R47

## Scope

| AGC file (vehicle) | pages | C target |
|---|---|---|
| P51-P53.agc (CM, LM; diffed) | CM 737-784, LM 926-983 | src/programs/alignment/align_*.c (shared math), src/csm/align_programs_csm.c, src/lm/align_programs_lm.c, src/lm/lunar_surface_align.c |
| SXTMARK.agc (CM) | 222-235 | src/csm/optics/sxtmark.c |
| AOTMARK.agc (LM) | 244-261 | src/lm/aotmark.c |
| R63.agc (LM) | 338-341 | src/lm/rendezvous_final_attitude.c |
| CSM_GEOMETRY.agc (CM) | 285-296 | src/csm/optics/csm_geometry.c |
| LEM_GEOMETRY.agc (LM) | 320-325 | src/lm/lem_geometry.c |
| S-BAND_ANTENNA_FOR_CM.agc / _FOR_LM.agc | CM 934-935, LM 486-489 | src/csm/optics/s_band_antenna_csm.c, src/lm/s_band_antenna_lm.c |
| LUNAR_LANDMARK_SELECTION_FOR_CM.agc | 936 | empty in Comanche 055 (only a page header); nothing to translate |
| AGS_INITIALIZATION.agc (LM, R47) | 206-210 | src/lm/ags_init.c |

Public header: include/agc/agc_align.h. Interfaces assumed from other lanes
(not yet present when written) are declared in one place,
include/agc/agc_align_pending.h, so integration only has to edit that file.

## Design (written before coding)

Scales: half-unit vectors are DP vectors of magnitude 0.5 (AGC "UNIT"
output, B-1). Angles: "revs" = DP fraction of one revolution (B0);
"CDU" = SP 2's-complement half-revolutions (one count = 360/32768 deg).

| AGC label (file, page) | inputs (scale) | outputs (scale) | C function |
|---|---|---|---|
| CDULOGIC / 1STO2S (RTB, used here) | CDU SP half-revs | DP revs / SP half-revs | agc_align_cdulogic, agc_align_1sto2s |
| CDUTRIG equivalent (sin/cos of CDUs) | CDUX,Y,Z | SINCDU,COSCDU (sin/2) | agc_align_cdutrig |
| CALCSMSC (P51-P53 CM p.763) | SINCDU,COSCDU | XNB,YNB,ZNB half-unit wrt SM | agc_align_calcsmsc |
| MATMOVE (p.762/948) | matrix | matrix | struct assignment (inline) |
| S52.2 (p.774/953) | XSMD..(half-unit, ref), REFSMMAT, CDUs | XSM.., THETAD (CDU) | agc_align_gimbal_angles |
| S52.3 (p.780/954) CM: Z=-R; LM: X=R | TALIGN cs B28 | XSMD.. | agc_align_nominal_orientation |
| P52LS (CM p.742) / P52LS+N89DISP+LSORIENT (LM p.932-933, 983) | RLS, time | XSMD.. | agc_align_landing_site_orientation_csm/_lm, agc_align_lsorient |
| CAL53A/CALOOP (p.761/947) | THETAD, CDUs | coarse needed? | agc_align_coarse_needed |
| GYCRS / GYCOARS (p.741/944) | XSMD, REFSMMAT | XDC, OGC.. | agc_align_gyro_coarse_dc |
| S50/LOCSAM + OCCOS (p.748-751 / 934-936; diff) | time | VEARTH VSUN VMOON CEARTH CSUN CMOON (cos/4) VEL/C(CM) | agc_align_locsam |
| PICAPAR/R56 + OCCULT (p.752-755 / 937-940; diff) | CDUs, REFSMMAT, LOCSAM | BESTI,BESTJ (star*6), VFLAG | agc_align_picapar |
| PLANET (p.782-784 / 961-962; diff) | star code, time | half-unit LOS ref (CM: + aberration VEL/C) | agc_align_planet |
| CHKSDATA/R54 math (p.759-761 / 945-947) | STARAD,+6, 6D,12D | NORMTEM1 = abs angle diff revs | agc_align_chksdata_angle |
| R51 math (p.756-758 / 941-943) | cat stars (ref), obs stars (SM), REFSMMAT | XDC (via AXISGEN) | agc_align_fine_solve |
| R55 / CHKSDATA / CAL53A / flows | | | job functions agc_job_<label> |
| SXTNB (CSM_GEOMETRY p.285) | SA,TA CDU | STARM half-unit NB | agc_csm_sxtnb |
| SXTLOGIC (p.285) | TA CDU | TA revs (19.775 deg offset, /4) | agc_csm_sxtlogic |
| CALCSXA / SXTANG / SXTANG1 (p.286-288) | STAR (SM or any), XNB.. | SAC,PAC (CDU), CULTFLAG | agc_csm_calcsxa, agc_csm_sxtang |
| SVDWN1/SVDWN2 (both GEOMETRY files; names swapped LM) | RCV,VCV,TDELTAV,TNUV,TET, X2 | RN,VN,PIPTIME or R/V-OTHER | agc_align_svdwn_this / _other |
| LOG (p.291) | x | -ln(x) | agc_csm_log |
| EARROT1/EARROT2/OVERADAY (p.293-295) | RTINIT, UNITW, DTEAROT | RT, RTEAST, RTNORM | agc_csm_earrot1/2 |
| RRANGLES (LEM_GEOMETRY p.322-323) | NB LOS half-unit, RADMODES | MODEA, MODEB (T,S CDU) | agc_lm_rrangles |
| RRNB / RRNBMPAC (p.324) | T,S CDU | NB LOS half-unit | agc_lm_rrnb |
| SBANDANT (CM R05 p.934) | CDUs, CSM state, LUNPOS | RHOSB (pitch), GAMMASB (yaw) revs | agc_sband_cm_angles (pure) + job |
| SBANDANT (LM R05 p.486) | idem, LM | PITCHANG, YAWANG revs | agc_sband_lm_angles (pure) + job |
| V89CALL (R63 p.338-341) | states, OPTION1+1 axis | CPHI.. FDAI angles | agc_rendezvous_final_attitude_los + jobs |
| AGSINIT/SCALEVEC (R47 p.206-210) | LM/CSM states, AGSK | AGSBUFF (14 words) | agc_ags_scalevec + jobs |
| SXTMARK, MKRELEAS, MARKRUPT, MARKCONT, MARK2/VACSTOR, MKREJECT, MKVB51/50, MARKIT, MARKDIF, DIFCHK (SXTMARK.agc) | | | src/csm/optics/sxtmark.c |
| AOTMARK, MKRELEAS, GETDAT, DODAT, OPTAXIS, OANB, SURFSTAR, GETMKS, MARKCHEX, AVESTAR, AVEIT, MARKRUPT, MKREJ, MARKTYPE, VACSTOR, REMARK, CHANGEVB, DSPV6N79 (AOTMARK.agc) | | | src/lm/aotmark.c |
| R52 (CM AOP), SR52.1, ADVTRACK, ROTA, ADVORB, TERM52, R53, CHKSCODE, R56 (CM alt LOS), SXTSM | | | src/csm/optics_sighting_csm.c |
| R52 (LM), R59, P57, GVDETER, GRAVEL, LUNG, GYROTRIM, 2STARS, SURFLINE, COATRIM, MFREF, REFMF, IMUCHK | | | src/lm/lunar_surface_align.c, src/lm/star_acquisition_lm.c |

## Progress

Resumed session 2 (real core headers now exist): align_pending.h trimmed to
interfaces still missing; CALCSMSC now the imu lane's (agc_imu_calcsmsc),
CDUTRIG/CDULOGIC guid-powered's; displays via agc_dsky_display().

Session 3: NAMING.md rule applied (no P/R/V/N numbers in identifiers).
Files renamed: align_s52.c -> align_orientation.c, sband_cm.c ->
s_band_antenna_csm.c, sband_lm.c -> s_band_antenna_lm.c; new
src/programs/alignment/sighting_data_and_gyro_torquing.c (R54/R55) and
src/programs/alignment/imu_realign.c (shared P52 front end, CAL53A, GYCRS).
Job prototypes in agc_align.h and the provisional ids in align_pending.h
are regenerated by SCRATCH/prog-align/gen.py from the lane's .c files.

| item | status |
|---|---|
| include/agc/agc_align.h (public API, alignment erasable block) | DONE (rewritten for real APIs; job prototypes listed) |
| include/agc/agc_align_pending.h (missing external interfaces, provisional job ids) | DONE |
| include/agc/agc_align_internal.h + align_common.c (RAM, CDULOGIC wrap, 1STO2S, CDUTRIG glue, integration/display/stall/phase helpers, SET1/PDT) | DONE |
| align_s52.c: S52.2 agc_align_gimbal_angles, S52.3 agc_align_nominal_orientation(_axes), LSORIENT/P52LS tail agc_align_lsorient, CALOOP agc_align_coarse_needed, GYCRS math agc_align_gyro_coarse_dc | DONE |
| align_locsam.c: LOCSAM agc_align_locsam(_from), OCCOS, OCCULT, PICAPAR/R56, PLANET agc_align_planet_vector, CHKSDATA math, R51 AXISGEN agc_align_fine_solve, CHKSCODE | DONE |
| SVDWN1/SVDWN2 agc_align_svdwn -> src/nav/integration/svdwn.c (declared in agc_align.h) | DONE |
| R54/R55 shared jobs (sighting_data_and_gyro_torquing.c) | DONE |
| shared P52 front end PROG52..P52D, COARSTYP, CAL53A, GYCRS, COARSE/NCOARSE (imu_realign.c, align_common.c) | DONE |
| CSM_GEOMETRY.agc: SXTNB, SXTLOGIC, CALCSXA, SXTANG(1), LOG, EARROT1/2, OVERADAY (src/csm/optics/csm_geometry.c) | DONE |
| LEM_GEOMETRY.agc: RRANGLES, RRNB/RRNBMPAC (src/lm/lem_geometry.c) | DONE |
| S-band CM / LM: SBANDANT jobs + pure angle fns (src/csm/optics/s_band_antenna_csm.c, src/lm/s_band_antenna_lm.c) | DONE |
| CSM P52/P54, P51/P53 (align_programs_csm.c) | TODO |
| CSM R51, R52, SR52.1, ADVTRACK, R53, R56, SXTSM, PLANET job (r5x_csm.c) | TODO |
| SXTMARK.agc | TODO |
| LM P52, P51, R51, R52, R59 (align_programs_lm.c, r5x_lm.c) | TODO |
| LM P57 family (lunar_surface_align.c) | TODO |
| AOTMARK.agc | TODO |
| R63.agc | TODO |
| AGS_INITIALIZATION.agc | TODO |
| LUNAR_LANDMARK_SELECTION_FOR_CM.agc | DONE (file is empty in Comanche 055) |
| tests/test_align.c (P52 opt 3 identity, S-band +X, SXT/RR round trips, LOG, SVDWN, S52.3 axes) | DONE (71 checks pass, linked with unresolved-ignore while other lanes finish) |

## Requested core additions

All assumed interfaces are declared, with owner, in
include/agc/agc_align_pending.h (state-flags accessors beyond agc_erasable.c,
io-pinball display calls with continuation job + response code in
ctx->arg, exec GOTOPOOH/DELAYJOB/VAC claim/RESUME, imu CALCGTA/CALCGA/
AXISGEN/ARCTRIG/IMUCOARS/IMUFINE/IMUPULSE/IMUSTALL/OPTSTALL/VECPOINT/
BALLANGS/R60LEM/1/PIPA/GCOMPZER, guid-powered *SMNB* / *NBSM* / PIPASR,
nav-integ CSM/LEM CONIC/PREC, nav-aux LSPOS/star table/RP-TO-R/R-TO-RP/
LAT-LONG/LALOTORV, prog-rendz LLASRD/LLASRDA/R61CSM, guid-landing DESCBITS).

## Requested jobs-tasks

Provisional AGC_JOB_* / AGC_TASK_* constants in include/agc/agc_align_pending.h
(0x0A01..); each maps to an agc_job_<label>() in this module.
