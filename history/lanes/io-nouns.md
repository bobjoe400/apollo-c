# io-nouns -- lane history

Process diary of the io-nouns lane, moved verbatim out of [docs/modules/io-nouns.md](../../docs/modules/io-nouns.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress
* DONE noun tables CSM/LM: src/io/dsky/nouns/tables_csm.c, noun_tables_lm.c
* DONE SFINTAB/SFOUTAB + scale defs + agc_scale_*: src/io/dsky/nouns/scales.c
* DONE agc_nouns.h
* DONE agc_noun_lookup*, agc_noun_read/write: src/io/dsky/nouns/data.c (22 ids wired to core, rest UNIMPLEMENTED)
* DONE tests/io/nouns.c (3400 checks pass with core state/channels + scratch agc_word stub)
* TODO (when core adds accessors) add cases to noun_data.c for the ids below

## Requested core additions
Wired already (core accessors exist): TIME2, TIG, TTOGO, TTPI, PIPAX/Y/Z, CSMMASS, LEMMASS,
CDUX/Y/Z, CDUS, CDUT, DAPDATR1/2, FAILREG(+1,+2), THETAD(+1,+2), CHANNEL33.

Wanted: for each id below, `<agc_dp_t|agc_sp_t|uint16_t> agc_erase_<lowercase name>(void)`
and `void agc_erase_set_<name>(...)` (raw words as uint16_t, SP as agc_sp_t, DP as agc_dp_t,
the AGC scale kept as is). "X +n" ids are the n-th word after X (suffix _Pn); vectors such as
DELVLVC / +2 / +4 may be one agc_vec_t accessor plus component access. Note some of
these may already exist under other names in agc_erasable.c (e.g. DELVSLV vs DELVLVC:
identical storage in Comanche, separate in Luminary -- not wired for that reason; TET is
the time of the state vector being integrated, not PIPTIME).

| Data id (AGC erasable) | width | scale type | AGC scale | nouns |
|---|---|---|---|---|
| ABVEL | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:62,63 |
| ACTCENT | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:52 LM:52 |
| ALMCADR | SP raw | OCTAL | SP, octal word | CSM:08 LM:08 |
| ALMCADR_P1 | SP raw | OCTAL | SP, octal word | CSM:08 LM:08 |
| ALMCADR_P2 | SP raw | OCTAL | SP, octal word | CSM:08 LM:08 |
| ALPHASB | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:51 |
| ALT | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:43 LM:43 |
| ALTI | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:62 |
| AOTCODE | SP raw | OCTAL | SP, octal word | LM:70,71 |
| AOTCODE_P1 | SP raw | OCTAL | SP, octal word | LM:70,71 |
| AOTCODE_P2 | SP raw | OCTAL | SP, octal word | LM:70,71 |
| AZ | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:87 |
| BETASB | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:51 |
| CENTANG | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:55 LM:55 |
| CPHIX | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:17,96 |
| CPHIX_P1 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:17,96 |
| CPHIX_P2 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:17,96 |
| CURSOR | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:79 |
| D | DP | DRAG_ACCEL | DP, bit 1 of low word = 25 x 2^-28 g | CSM:64,74 |
| DATAGOOD | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:80 |
| DELHITE | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:72 |
| DELTAH | DP | COMP_ALT | DP, bit 1 of low word = 2^-4 m | LM:68 |
| DELTAR | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:57 LM:57 |
| DELVIMU | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:83 LM:83 |
| DELVIMU_P2 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:83 LM:83 |
| DELVIMU_P4 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:83 LM:83 |
| DELVLVC | DP | VEL3,VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:81,82,86 LM:81,82,86 |
| DELVLVC_P2 | DP | VEL3,VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:81,82,86 LM:81,82,86 |
| DELVLVC_P4 | DP | VEL3,VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:81,82,86 LM:81,82,86 |
| DELVOV | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:84 LM:84 |
| DELVOV_P2 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:84 LM:84 |
| DELVOV_P4 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:84 LM:84 |
| DELVTPF | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:58 LM:58 |
| DELVTPI | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:58 LM:58 |
| DIFFALT | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:75 LM:75 |
| DNRNGERR | DP | POS6 | DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi | CSM:66 |
| DSPTEM1 | SP raw | WHOLE,OCTAL,CDU_DEG | SP, bit 1 = 1 unit (max 16383); SP, octal word; SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:25,26,29,30,41,97 LM:25,26,41,97 |
| DSPTEM1_DP | DP | DP_DEG360,HMS | DP, bit 1 of low word = 360/2^28 deg; DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:05,31,34 LM:04,05,34 |
| DSPTEM1_P1 | SP raw | WHOLE,OCTAL,ELEV_DEG | SP, bit 1 = 1 unit (max 16383); SP, octal word; SP, bit 1 = 90/2^14 deg | CSM:25,26,30,41,97 LM:25,26,41,97 |
| DSPTEM1_P2 | SP raw | WHOLE,OCTAL | SP, bit 1 = 1 unit (max 16383); SP, octal word | CSM:25,26,30,97 LM:25,26,97 |
| DSPTEM2 | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:98 LM:98 |
| DSPTEM2_P1 | SP | FRACTION | SP, bit 1 = 2^-14 unit | CSM:98 LM:98 |
| DSPTEM2_P1_DP | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:24 LM:24 |
| DSPTEM2_P2 | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:98 LM:98 |
| DSPTEMX | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:14 |
| DSPTEMX_DP | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:16 LM:16 |
| DSPTEMX_P1 | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:14 |
| DSPTEMX_P2 | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:14 |
| DVLOS | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:59 LM:59 |
| DVLOS_P2 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:59 LM:59 |
| DVLOS_P4 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:59 LM:59 |
| DVTOTAL | DP | VEL3,VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:40,80 LM:40,62 |
| EL | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:87 |
| ELEV | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:55 LM:55 |
| FDAIX | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:18 |
| FDAIY | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:18 |
| FDAIZ | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:18 |
| FUNNYDSP | DP | TWO_INT | DP, high word = XX, low word = YY, each a positive integer < 100 | LM:64 |
| GAMMAEI | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:60 |
| GAMMASB | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:51 |
| GMAX | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:60 |
| HAPO | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:42 LM:42 |
| HAPOX | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:44 LM:44 |
| HCALC | DP | COMP_ALT | DP, bit 1 of low word = 2^-4 m | LM:60,64 |
| HCALC1 | DP | COMP_ALT | DP, bit 1 of low word = 2^-4 m | LM:63 |
| HDOT | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:62 |
| HDOTDISP | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:60,63,64 |
| HEADSUP | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:61 |
| HORIZON | SP raw | OCTAL | SP, octal word | CSM:70,71 |
| HPER | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:42 LM:42 |
| HPERX | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:44,50 LM:44 |
| LANDALT | DP | POS5 | DP, bit 1 of low word = 2 m | CSM:89 LM:89 |
| LANDLAT | DP | DP_DEG90 | DP, bit 1 of low word = 360/2^28 deg | CSM:89 LM:89 |
| LANDLONG | DP | DP_DEG90 | DP, bit 1 of low word = 360/2^28 deg | CSM:89 LM:89 |
| LANDMARK | SP raw | OCTAL | SP, octal word | CSM:70,71 |
| LAT | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:43,67 LM:43 |
| LAT_SPL | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:61 |
| LNG_SPL | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:61 |
| LONG | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:43,67 LM:43 |
| MRKBUF1_P3 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:87,94 |
| MRKBUF1_P5 | SP raw | Y_OPTICS_DEG | SP, bit 1 = 90/2^15 deg | CSM:87,94 |
| N49DISP | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:49 |
| MARK_DELTA_V | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:49 |
| MARK_SOURCE_CODE | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:49 |
| NEG_TPER | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:32 LM:32 |
| NN | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:55 |
| NN1 | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:55 |
| OGC | DP | DP_DEG90 | DP, bit 1 of low word = 360/2^28 deg | CSM:93 LM:93 |
| OGC_P2 | DP | DP_DEG90 | DP, bit 1 of low word = 360/2^28 deg | CSM:93 LM:93 |
| OGC_P4 | DP | DP_DEG90 | DP, bit 1 of low word = 360/2^28 deg | CSM:93 LM:93 |
| OMEGAD | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:80 |
| OPTION1 | SP raw | OCTAL | SP, octal word | CSM:06 LM:06 |
| OPTION2 | SP raw | OCTAL,WHOLE | SP, octal word; SP, bit 1 = 1 unit (max 16383) | CSM:06,72 LM:06 |
| OPTION3 | SP raw | OCTAL | SP, octal word | LM:06 |
| OPTIONX | SP raw | OCTAL | SP, octal word | CSM:12 LM:12 |
| OPTIONX_P1 | SP raw | OCTAL | SP, octal word | CSM:12 LM:12 |
| OUTOFPLN | DP | POS7 | DP, bit 1 of low word = 2^-4 m | LM:61 |
| P21ALT | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:73 LM:91 |
| P21GAM | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:73 LM:91 |
| P21VEL | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:73 LM:91 |
| PAC | SP raw | Y_OPTICS_DEG | SP, bit 1 = 90/2^15 deg | CSM:92 |
| PACTOFF | SP | TRIM_DEG | SP, low-order bit = 85.41 arc-sec | CSM:48 |
| PITCH | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:74 |
| PITTIME | SP | TRIM_DEG2 | SP, bit 1 = 0.01 sec of time | LM:48 |
| PLUS_MGA | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:45 LM:45 |
| POSCODE | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:79 |
| POSTTPI | DP | POS4 | DP, bit 1 of low word = 2 m | CSM:58 LM:58 |
| PRAXIS | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:95 |
| PRAXIS_P1 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:95 |
| PRAXIS_P2 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:95 |
| Q7 | DP | DRAG_ACCEL | DP, bit 1 of low word = 25 x 2^-28 g | CSM:69 |
| R22DISP | DP | POS4 | DP, bit 1 of low word = 2 m | LM:49 |
| RADAR_DELTA_V | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:49 |
| RANGE | DP | POS5 | DP, bit 1 of low word = 2 m | CSM:53,54,90 LM:54,90 |
| RANGEDSP | DP | POS7 | DP, bit 1 of low word = 2^-4 m | LM:68 |
| RDOT | DP | VEL_2VS | DP, fractional part of 2VS ft/sec (VS = 25766.1973) | CSM:68 |
| RDOTD | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:76 |
| RHOSB | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:51 |
| ROLLC | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:66,68,69,74 |
| ROLLTIME | SP | TRIM_DEG2 | SP, bit 1 = 0.01 sec of time | LM:48 |
| RRATE | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:53,54,90 LM:54,90 |
| RR_AZ | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:56 |
| RR_ELEV | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:56 |
| RSP_RREC | DP | POS6 | DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi | CSM:50 |
| RSTACK | DP | LR_VELX,RR_RANGE | DP, low bit of low word = -0.6440 ft/sec; DP, low bit of low word = 9.38 ft | LM:67,78 |
| RSTACK_P2 | DP | LR_VELY,RR_RATE | DP, low bit of low word = 1.212 ft/sec; DP, low bit of low word = -0.6278 ft/sec | LM:67,78 |
| RSTACK_P4 | DP | LR_VELZ | DP, low bit of low word = 0.8668 ft/sec | LM:67 |
| RSTACK_P6 | DP | LR_ALT | DP, low bit of low word = 1.079 ft | LM:66 |
| RTEDVD | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:56 |
| RTEGAM2D | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:56 |
| RTGO | DP | POS6 | DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi | CSM:63 |
| RTGON64 | DP | POS6 | DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi | CSM:64 |
| RTGON67 | DP | POS6 | DP, bit 1 of low word = (6373338)(2 PI) 2^-28 / 1852 nmi | CSM:67 |
| RTHETA | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:53,54,90 LM:54,90 |
| SAC | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | CSM:92 |
| SAMPTIME | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:65 LM:65 |
| SMODE | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:27 LM:27 |
| SPIRAL | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:79 |
| STARAD | SP | FRACTION | SP, bit 1 = 2^-14 unit | LM:88 |
| STARAD_P2 | SP | FRACTION | SP, bit 1 = 2^-14 unit | LM:88 |
| STARAD_P4 | SP | FRACTION | SP, bit 1 = 2^-14 unit | LM:88 |
| STARCODE | SP raw | OCTAL | SP, octal word | CSM:70,71 |
| STARSAV3 | DP | DP_FRACTION | DP, bit 1 of high word = 2^-14 unit | CSM:88 |
| STARSAV3_P2 | DP | DP_FRACTION | DP, bit 1 of high word = 2^-14 unit | CSM:88 |
| STARSAV3_P4 | DP | DP_FRACTION | DP, bit 1 of high word = 2^-14 unit | CSM:88 |
| T1TOT2 | DP | MIN_SEC | DP centiseconds, bit 1 of low word = 10^-2 s | CSM:75 LM:75 |
| T2TOT3 | DP | MIN_SEC | DP centiseconds, bit 1 of low word = 10^-2 s | CSM:75 LM:75 |
| T3TOT4 | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:39 |
| TANG | SP raw | CDU360 | SP, bit 1 = 360 - 360/2^15 deg | LM:73 |
| TANG_P1 | SP raw | CDU_DEG | SP, bit 1 = 360/2^15 deg (15-bit magnitude, 2s complement) | LM:73 |
| TCDH | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:13 LM:13 |
| TCSI | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:11 LM:11 |
| TET | DP | HMS | DP centiseconds, bit 1 of low word = 10^-2 s (max 745h39m14.55s) | CSM:38 LM:38 |
| TFF | DP | MIN_SEC | DP centiseconds, bit 1 of low word = 10^-2 s | CSM:44,50 LM:44 |
| THETZERO | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | CSM:72 |
| TRKMKCNT | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:45 |
| TTE | DP | MIN_SEC | DP centiseconds, bit 1 of low word = 10^-2 s | CSM:63 |
| TTFDISP | DP | MIN_SEC | DP centiseconds, bit 1 of low word = 10^-2 s | LM:61,68 |
| VGBODY | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:85 LM:85 |
| VGBODY_P2 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:85 LM:85 |
| VGBODY_P4 | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | CSM:85 LM:85 |
| VGDISP | DP | VEL3,VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:40,42,80 LM:40,42 |
| VHFCNT | DP | TWO_INT | DP, high word = XX, low word = YY, each a positive integer < 100 | CSM:45 |
| VHORIZ | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:60 |
| VIO | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:63 |
| VL | DP | VEL_2VS | DP, fractional part of 2VS ft/sec (VS = 25766.1973) | CSM:69 |
| VMAGI | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:62,64,68,74 |
| VPRED | DP | VEL2 | DP, bit 1 of high word = 2^-7 m/cs | CSM:60 |
| WHCHREAD | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | LM:49 |
| WWBIAS | DP | RADIANS | DP, bit 1 of high word = 2^-9 rad | LM:99 |
| WWOPT | SP | WHOLE | SP, bit 1 = 1 unit (max 16383) | CSM:99 |
| WWPOS | DP | POS9 | DP, bit 1 of low word = 2^-9 m | CSM:99 LM:99 |
| WWVEL | DP | VEL4 | DP, fractional part of m/cs | CSM:99 LM:99 |
| XRANGE | DP | POS4 | DP, bit 1 of low word = 2 m | LM:76 |
| XREG | SP raw | OCTAL | SP, octal word | CSM:07 LM:07 |
| XRNGERR | DP | POS8 | DP, bit 1 of low word = 4 x 6373338 x 2^-28 / 1852 nmi | CSM:66 |
| YACTOFF | SP | TRIM_DEG | SP, low-order bit = 85.41 arc-sec | CSM:48 |
| YAW | DP | DP_DEG360 | DP, bit 1 of low word = 360/2^28 deg | LM:74 |
| YDOT | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:77 |
| YREG | SP raw | OCTAL | SP, octal word | CSM:07 LM:07 |
| ZDOTD | DP | VEL3 | DP, bit 1 of high word = 2^-7 m/cs | LM:76 |
| ZREG | SP raw | OCTAL | SP, octal word | CSM:07 LM:07 |

## Requested jobs-tasks
None.

## Open questions
* Is the "raw 15-bit word as unsigned integer" convention what io-pinball wants for
  OCTAL/CDU scales? (SP-in-high-word would turn 0o77777 into -0.)
* HMS rule (only data_id[0] set): noun 65 in the AGC lists SAMPTIME three times.
* SF constants: the 2DEC* entries (LM landing radar) are rope values; their physical
  scale (E5 B-28) is quoted in the comment.

