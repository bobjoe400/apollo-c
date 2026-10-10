# prog-rendz -- rendezvous programs (P17, P20-P25, P31-P39, P72-P79, P76, P21, R30, R31, R36)

## Scope

AGC sources (Comanche055 = CSM, Luminary099 = LM):
P20-P25.agc (both), P32-P33_P72-P73.agc (CSM) / P32-P35_P72-P75.agc (LM),
P34-35_P74-75.agc (both), R30.agc, R31.agc, STABLE_ORBIT.agc, P76.agc,
GROUND_TRACKING_DETERMINATION_PROGRAM.agc (both), TPI_SEARCH.agc (CSM),
GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc, RADAR_LEADIN_ROUTINES.agc (LM).

C target: src/programs/ (all files, vehicle-only files suffixed _csm/_lm
because the assignment names src/programs as the target directory), public
header include/agc/agc_programs.h section "rendezvous".

Vehicle diff summary (code lines, comments stripped):
* CSI/CDH: identical algorithm. LM adds automatic TCSI (TCSI<0 -> TIMERAD
  to apocenter), uses TMIN (=60000 cs) where CM uses 600SEC (=60000 cs),
  TWOPI vs 2PISC SL2 (same value), CM 1DPB2 transcribed "1.08-2" (read as
  1.0 B-2, as in LM).
* P34/P35: LM sets CENTANG = 130 deg at P34 entry; LM S34/35.5 checks crew
  overwrite of DELVLVC word by word (NTARGFLG, GDT/2); LM INITVEL computes
  MU/A and MUASTEER at exit, stores RTMAG; LM MIDGIM is only GET+MGA. CM
  MAX250 has B-28 (same value 25000 cs).
* R30: LM has no DELRSPL (P11 splash) branch; moon flag AMOONFLG (CM) vs
  MOONTHIS (LM).
* R31: different. CM: STATEXTP/HAVEBASE, R34 (V65 phi) shares it, LM base
  vector code differs (MUNFLAG, SURFFLAG, GETRVN with REFSMMAT conversion).
* STABLE_ORBIT: identical except NN=2 setup placement and DSPTEM1 copy.
* P76: LM saves DELVLVC to DELVOV first; CM clears VHFCNT/MRKBUF2;
  P76SUB1 moon-index differs.
* P21: flag polarity of VINTFLAG test; LM recycles base vector only when
  not SURFFLAG.
* P20-P25: different programs (CM optics/VHF, LM rendezvous radar).

## Design table

Scales: r m 2^29 (2^27 moon body scale), v m/cs 2^7 (2^5), t cs 2^28,
angles revolutions 2^0, unit vectors 2^1, mu m^3/cs^2 2^36.

| AGC label (file, page) | inputs -> outputs | C function | file |
|---|---|---|---|
| SELECTMU (P34-35 p.494) | CMOONFLG -> RTX1 RTX2 RTMU RTSR1/MU | agc_rendz_selectmu | rendz_common.c |
| VECSHIFT / SHIFTR1 (p.499-500) | earth-scale <-> body-scale | agc_rendz_vec_to_body / agc_rendz_shiftr1 | rendz_common.c |
| PERIAPO / PERIAPO1 / SETRAD (p.495-497) | r,v -> rapo hapo rper hper ecc | agc_rendz_periapo1 | rendz_common.c |
| PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498) | TDEC1 -> RACT3 VACT3 RPASS3 VPASS3 | rz_precset | rendz_common.c |
| INTINT / INTINT2C / INTINT3P / GOINT (p.479, 677) | r v t1 t2 conic? -> RATT VATT | rz_intint | rendz_common.c |
| S34/35.1 (p.474) | RACT3 RPASS3 VACT3 -> ULOS UNRM | agc_rendz_s34_35_1 | rendz_common.c |
| LOMAT (p.479) | UNRM RACT3 -> LV matrix | agc_rendz_lomat | rendz_common.c |
| MIDGIM / GET+MGA / GET.LVC (p.492-493) | dV -> +MGA or DELVLVC | agc_rendz_get_mga / agc_rendz_get_lvc | rendz_common.c |
| AVFLAGA/P, P20FLGON, VNPOOH, VN1645, COMPTGO, P3XORP7X | flags / displays | rz_avflag, rz_p20flgon, job chain | rendz_common.c, rendz_display.c |
| ADVANCE / ROTATE (P32 p.675-676) | TIG -> UP1 R/V ACT1,2 PASS1,2 | agc_rendz_advance / agc_rendz_rotate | rendz_csi.c |
| CSI/A..CSINEXT4, SCNDSOL, ALMXIT (p.663-674) | -> DELVCSI DELVEET1 TCDH T1TOT2 T2TOT3 POSTCSI POSTCDH | agc_rendz_csi_a | rendz_csi.c |
| CDHMVR (p.680-682) | RACT2 VACT2 RPASS2 VPASS2 -> VACT3 DELVEET2 DIFFALT | agc_rendz_cdhmvr | rendz_csi.c |
| S32/33.1, S32/33.X, DISDVLVC (p.662,678,679) | DELVEET -> DELVSIN DELVSAB DELVLVC | agc_rendz_s32_33_1 / _x | rendz_csi.c |
| P32/P72, P33/P73 drivers (p.652-660) | DSKY chains | agc_prog_p32 / agc_prog_p33 + jobs | rendz_p32_p33.c |
| S33/34.1 ELCALC..TIMEX (p.469-473) | iterate TTPI / ELEV | agc_rendz_s33_34_1 | rendz_tpi.c |
| S34/35.2/.3/.4/.5 (p.475-480) | Lambert / LV / overwrite | agc_rendz_s34_35_2.. | rendz_tpi.c |
| INITVEL (p.486-491) | RINIT VINIT RTARG DELLT4 -> VIPRIME VTPRIME DELVEET3 RTARG | agc_rendz_initvel | rendz_initvel.c |
| P34/P74, P35/P75 drivers | DSKY chains | agc_prog_p34 / p35 | rendz_tpi.c |
| R36 (p.501-504) | -> RANGE(Y) RRATE(YDOT) RTHETA(PSI) | agc_rendz_r36_compute, agc_verb90_r36 | rendz_r36.c |
| V82CALL, SR30.1, MAXCHK (R30 p.514-523) | RONE VONE -> HAPOX HPERX TFF -TPER | agc_r30_sr30_1, agc_r30_maxchk, agc_verb82_r30 | r30.c |
| R31 V83 COMPDISP / STATEXTP (R31) | -> RANGE RRATE RTHETA | agc_r31_compdisp, agc_verb83_r31 | r31.c |
| P38/P39 (STABLE_ORBIT) | | agc_prog_p38 / p39, agc_rendz_p38_deltt | stable_orbit.c |
| S17.1 S17.2 TRANSANG P17 (TPI_SEARCH, CSM) | | agc_rendz_s17_1 / s17_2 / transang | tpi_search_csm.c |
| P31 (GLAG, LM) | TIG RTARG DELLT4 -> DELVSIN HAPO HPER | agc_prog_p31 | glag_lm.c |
| P76 | DELVLVC TIG -> other SV updated | agc_prog_p76, agc_p76_lv_to_ref | p76.c |
| P21 PROG21 (GTDP) | time -> lat long alt, vel, gamma | agc_prog_p21, agc_p21_flight_path | p21.c |
| P20-P25 CSM: BVECTORS, GETUM, R22 REND*, SETINTG, SHIFTNDX, CMPOS/LMPOS, INITIALW, RANGERD | marks -> state update | p20_csm.c | |
| P20-P25 LM: LPS20.1, LPS20.2, LRS22.1-3, RR routines | radar -> state update | p20_lm.c | |
| RADSAMP / DORSAMP / VARADAR (LM) | radar sampling test | radar_leadin_lm.c | |

External calls are funnelled through src/programs/rendz_glue.c (one place
to adapt when agc_nav.h / agc_integ.h / agc_navaux.h / agc_dsky.h / agc_imu.h
land); assumed signatures are in src/programs/rendz_ext.h.

## Progress

* TODO everything (note created before code; header include/agc/agc_programs.h written)

## Deviations

(filled in as translated)

## Stubs

## Requested core additions

* agc_state.h: `agc_vec_t agc_erase_rls(void);` landing site vector RLS
  (moon-fixed, m 2^27); used by SETRAD/PERIAPO, R30 MOONPAD, P22.
* Interfaces assumed from not-yet-existing headers: see rendz_ext.h
  (conic TIMETHET/TIMERAD/LAMBERT/APSIDES, CSM/LEM PREC/CONIC, INTEGRVS,
  INTSTALL, TFF routines, LAT-LONG, DSKY flash displays, GOTOPOOH, ENDEXT,
  DELAYJOB, CDUTRIG/*NBSM*/SXTNB).

## Requested jobs-tasks

(filled in as translated)

## Tests written

## Open questions

* CM 1DPB2 is transcribed "2DEC 1.08-2"; LM has "1.0 B-2". Used 1.0 B-2.
