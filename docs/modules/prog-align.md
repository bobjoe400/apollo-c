# prog-align -- IMU alignment programs, optics marking, geometry, S-band, R63, R47

History of this lane: [history/lanes/prog-align.md](../../history/lanes/prog-align.md)

## Scope

| AGC file (vehicle) | pages | C target |
|---|---|---|
| P51-P53.agc (CM, LM; diffed) | CM 737-784, LM 926-983 | src/programs/alignment/align_*.c (shared math), src/csm/alignment/*.c, src/csm/optics/*.c, src/lm/alignment/*.c |
| SXTMARK.agc (CM) | 222-235 | src/csm/optics/sxtmark.c |
| AOTMARK.agc (LM) | 244-261 | src/lm/optics/aotmark.c |
| R63.agc (LM) | 338-341 | src/lm/rendezvous_final_attitude.c |
| CSM_GEOMETRY.agc (CM) | 285-296 | src/csm/optics/geometry.c |
| LEM_GEOMETRY.agc (LM) | 320-325 | src/lm/optics/lem_geometry.c |
| S-BAND_ANTENNA_FOR_CM.agc / _FOR_LM.agc | CM 934-935, LM 486-489 | src/csm/optics/s_band_antenna.c, src/lm/optics/s_band_antenna.c |
| LUNAR_LANDMARK_SELECTION_FOR_CM.agc | 936 | empty in Comanche 055 (only a page header); nothing to translate |
| AGS_INITIALIZATION.agc (LM, R47) | 206-210 | src/lm/ags_initialization.c |

Public header: include/agc/programs/alignment.h. Interfaces assumed from other lanes
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
| AOTMARK, MKRELEAS, GETDAT, DODAT, OPTAXIS, OANB, SURFSTAR, GETMKS, MARKCHEX, AVESTAR, AVEIT, MARKRUPT, MKREJ, MARKTYPE, VACSTOR, REMARK, CHANGEVB, DSPV6N79 (AOTMARK.agc) | | | src/lm/optics/aotmark.c |
| R52 (CM AOP), SR52.1, ADVTRACK, ROTA, ADVORB, TERM52, R53, CHKSCODE, R56 (CM alt LOS), SXTSM | | | src/csm/optics_sighting_csm.c |
| R52 (LM), R59, P57, GVDETER, GRAVEL, LUNG, GYROTRIM, 2STARS, SURFLINE, COATRIM, MFREF, REFMF, IMUCHK | | | src/lm/lunar_surface_align.c, src/lm/star_acquisition_lm.c |

## Deviations

* MARKSTAT bits 1-9 (SXTMARK, AOTMARK) hold VAC index + 1, not the VAC
  address (src/csm/optics/sxtmark.c, src/lm/optics/aotmark.c).
* S52.2 DLOAD* of two SP THETAD words: the spurious low-order part (< 1
  CDU count) is ignored (src/programs/alignment/orientation.c).
* 1STO2S: local copy of the RTB op until sys-misc publishes it
  (src/programs/alignment/common.c).
* SCALEVEC: AGSRND1 + AGSRND2 (one's-complement low words -8191 and
  +16383) are added as one DP addend of +8192 (half a high-word LSB); the
  two-step add only existed to defeat ALSIGNAG's sign agreement.

## Stubs

None.

## Tests written

tests/programs/alignment.c, 83 checks, 0 failures: P52 option 3 (identity
REFSMMAT, two orthogonal stars -> identity XDC, zero torquing), S-band
CM/LM angles for earth along +X, SXTNB/SXTANG and RRNB/RRANGLES round
trips, LOG, SVDWN, S52.3 axes, CHKSCODE, 1STO2S, R47 SCALEVEC rounding
(moon-sphere states -> 512 ft / 0.5 ft/s AEA words, read as 2's
complement) and AGS time, R63 line of sight.

## Program flow and decisions

_Documentation wave (2026-10-12); prose and tables brought to the final tree (HEAD 3b97dc8) by the final pass (2026-10-14). The prose was written by reading the code; the two tables were extracted from the function bodies (flag, alarm, channel and scheduling calls; `agc_erase_*` accessors) and list only what the code does. Files: `src/programs/alignment/*.c`, `src/csm/alignment/*.c`, `src/csm/optics/*.c`, `src/lm/alignment/*.c`, `src/lm/optics/*.c`, `src/lm/ags_initialization.c`, `src/lm/rendezvous_final_attitude.c`._

P51-P54, P57 and the alignment and marking routines of both vehicles;
P52 is drawn in [flowcharts/imu-realign.md](../overview/flowcharts/imu-realign.md).
Every crew display and every IMU/optics stall is a job boundary; a
failing service call returns its status to the job, and the ALARM-type
ends (CURTAINS, BAILOUT done) end the job with OK
(`src/programs/alignment/common.c`).

* **P52 / P54** (`src/programs/alignment/imu_realign.c`):
  `agc_job_imu_realign` (CM: phase 4.25, UPDATFLG and TRACKFLG cleared;
  both: R02BOTH, the preferred option when PFRATFLG) flashes V04N06
  (`agc_job_imu_realign_option_display`); `agc_job_imu_realign_option`
  branches on OPTION2: preferred, nominal (S52.3 at the time from P52T,
  `agc_job_realign_time*`, zero = now), REFSMMAT, landing site (CSM
  `src/csm/alignment/imu_realign.c` P52LS, LM `src/lm/alignment/imu_realign.c`).
  P52D (`agc_job_realign_gimbal_display`) shows S52.2 gimbal angles;
  PROCEED -> COARSTYP: CAL53A (`orientation.c`, CALOOP) chooses coarse
  align (a gimbal change too large) or gyro torquing (GYCRS) to XSMD;
  ENTER redisplays after the crew moves the vehicle. Then R51.
* **R51 fine align** (CSM `src/csm/alignment/fine_align.c`, LM
  `src/lm/alignment/fine_align.c`): the star pair from LOCSAM + PICAPAR
  (`locsam.c`: occultation by sun, earth, moon cones; CSM conic state
  and 15 deg sun cone, LM precision state and 60 deg cone) or keyed in on
  V01N70; no pair is alarm 00405. CSM: R52 drives the sextant
  (`src/csm/optics/auto_optics_positioning.c`, every 0.5 s 1.3 s ahead,
  only with the optics in CMC mode), R53 marks (`sighting_mark.c`,
  `sxtmark.c`); LM: R52 turns the vehicle (`src/lm/alignment/auto_optics_positioning.c`),
  AOTMARK marks (`src/lm/optics/aotmark.c`). R54 (`sighting_data_and_gyro_torquing.c`)
  compares the catalogue and marked pair angles on V06N05; R55
  (`agc_job_gyro_torquing_display`): PROCEED torques (`_pulse`: IMUPULSE
  on OGC, IMUSTALL; a bad end is CURTAINS), ENTER skips.
* **Mark systems**: SXTMARK (`src/csm/optics/sxtmark.c`): marks or an
  extended verb busy 01211, no VAC 01207, vehicle rate at mark 00121, no
  input bits 00113, marks not accepted 00112, two rejects together 00110.
  AOTMARK (`src/lm/optics/aotmark.c`): marks busy 00105, extended verb
  busy 01211, more than five pairs 00107, not in pairs 00111, plus the
  same input-bit / acceptance alarms.
* **P51 / P53** (`src/csm/alignment/imu_orientation_determination.c`,
  `src/lm/alignment/imu_orientation_determination.c`): IMU off is alarm
  00210; optional coarse align to zero (ENTER on V50N25 00015); two marks;
  AXISGEN gives REFSMMAT and REFSMFLG.
* **P57** (`src/lm/alignment/lunar_surface_align.c`,
  `lunar_surface_gravity.c`, `lunar_surface_sighting.c`): OPTION2 chooses
  the attitude source (0 stored, 1 REFSMMAT, 2/3 two stars or one star +
  gravity); an option inconsistent with the flags is alarm 00701; GVDETER
  turns the stable member 180 deg about gravity to cancel PIPA biases;
  R59 finds the AOT detent containing the star (none: alarm 00404);
  SURFDISP (`agc_job_lunar_surface_aligned`) sets REFSMMAT = XSMD and
  REFSMFLG.
* **Other**: R05 S-band antenna (`src/csm/optics/s_band_antenna.c`,
  `src/lm/optics/s_band_antenna.c`), R47 AGS initialisation
  (`src/lm/ags_initialization.c`: no REFSMMAT alarm 00220), R63 final
  attitude (`src/lm/rendezvous_final_attitude.c`), RR gimbal geometry
  (`src/lm/optics/lem_geometry.c`). Word-exact results (ARCTRIG, 1STO2S,
  DOT/DMP followed by SL) go through `word_forms.c`.

### Decision points by function

85 of the lane's 363 functions test or change a flag, raise an alarm, test a channel bit or start a job/task (AGC = the label(s) the prolog names; "+F" sets flag F, "-F" clears it; "starts" = NOVAC/FINDVAC, WAITLIST, LONGCALL, DELAYJOB targets).

| C function | AGC | flags tested | flags set (+) / cleared (-) | alarms | channel bits | starts |
|---|---|---|---|---|---|---|
| `agc_job_csm_fine_align_next_star` (fine_align.c) | R51.3 | - | -TARG2FLG -TARG1FLG | - | - | - |
| `agc_job_csm_fine_align_data_checked` (fine_align.c) | R54 | FREEFLAG | - | - | - | - |
| `agc_job_csm_fine_align_torqued` (fine_align.c) | R55 | - | -PFRATFLG | - | - | - |
| `agc_job_csm_imu_orientation_determination` (imu_orientation_determination.c) | P51 | - | - | ALARM 0210 | - | - |
| `agc_job_csm_orientation_mark_setup` (imu_orientation_determination.c) | P51B | - | -TARG2FLG | - | - | - |
| `agc_job_csm_orientation_data_checked` (imu_orientation_determination.c) | CHKSDATA | FREEFLAG | -REFSMFLG +REFSMFLG | - | - | - |
| `realign_pick_stars` (imu_realign.c) | LOCSAM | - | - | ALARM 0405 | - | - |
| `agc_job_csm_imu_realign_landing_site` (imu_realign.c) | P52LS | - | +LUNAFLAG +ERADFLAG | - | - | - |
| `agc_job_csm_landing_site_accepted` (imu_realign.c) | LSDISP | ERADFLAG LUNAFLAG | - | - | - | - |
| `agc_csm_target_optics_angles` (auto_optics_positioning.c) | SR52.1 | ADVTRK ERADFLAG LUNAFLAG TARG1FLG TARG2FLG | - | - | - | - |
| `auto_optics_wait` (auto_optics_positioning.c) | R52F | - | - | - | - | job auto_optics_positioning_delayed |
| `auto_optics_target` (auto_optics_positioning.c) | R52D | SIGHTING_MARK_REQUESTED TARG1FLG TRUNFLAG | - | PRIOLARM 0404, ALARM 0407 | - | - |
| `auto_optics_cycle` (auto_optics_positioning.c) | R52C | SIGHTING_MARK_REQUESTED | - | - | - | job auto_optics_sighting |
| `auto_optics_start` (auto_optics_positioning.c) | R52VRB | TARG1FLG | -SIGHTING_MARK_REQUESTED +TRUNFLAG -TERMIFLG | - | - | - |
| `agc_csm_auto_optics_positioning` (auto_optics_positioning.c) | R52 | - | -ADVTRK | - | - | - |
| `agc_csm_auto_optics_advanced_orbit` (auto_optics_positioning.c) | ADVORB | - | +ADVTRK +LUNAFLAG +ERADFLAG | - | - | - |
| `agc_job_auto_optics_positioning_delayed` (auto_optics_positioning.c) | R52F | TARG1FLG TERMIFLG | - | - | - | - |
| `agc_job_auto_optics_positioning_tracked` (auto_optics_positioning.c) | R52HA | PRFTRKAT TRACKFLG UPDATFLG | - | - | - | - |
| `agc_job_sbandant_cm_recycle` (s_band_antenna.c) | SBANDANT | - | - | - | - | job sbandant_cm_recycle |
| `agc_csm_sighting_mark` (sighting_mark.c) | R53 | - | +SIGHTING_MARK_REQUESTED | - | - | - |
| `sighting_mark_done` (sighting_mark.c) | R53D | - | +TERMIFLG | - | - | - |
| `markok` (sxtmark.c) | MARKOK | - | - | BAILOUT 1207 | - | - |
| `sxtmark_common` (sxtmark.c) | SXTMARK | - | - | BAILOUT 1211 | - | job mark_request_display |
| `agc_sxtmark_mkreleas` (sxtmark.c) | MKRELEAS MKRLEES | - | - | - | ch12 ENBOCDUERR | - |
| `agc_sxtmark_markrupt` (sxtmark.c) | MARKRUPT | - | - | ALARM 0113 | ch16 MARK, ch16 MARKREJECT, ch16 NAVKEY_MASK | task markdif |
| `agc_task_markdif` (sxtmark.c) | MARKDIF | - | - | ALARM 0121 | - | - |
| `agc_sxtmark_markcont` (sxtmark.c) | MARKCONT | CALIBRATION_MARKING MARKRUPT_OPTION_ONE | - | ALARM 0122 | - | job optics_calibration_mark_display |
| `agc_sxtmark_mark2` (sxtmark.c) | MARK2 VACSTOR | - | - | ALARM 0114 | - | job mark_complete_display |
| `agc_sxtmark_mkreject` (sxtmark.c) | MKREJECT MRKREJCT REJECT2 | MARKRUPT_OPTION_ONE | - | ALARM 0112, ALARM 0110 | - | job mark_request_display |
| `agc_job_termsxt` (sxtmark.c) | TERMSXT | - | +TRM03FLG | - | - | - |
| `agc_job_entanswr` (sxtmark.c) | ENTANSWR | - | - | - | - | task endmarks |
| `agc_job_ags_initialization` (ags_initialization.c) | AGSINIT | REFSMFLG | - | ALARM 0220 | - | - |
| `agc_job_ags_initialization_compute` (ags_initialization.c) | AGSVCALC | - | +NODOFLAG +XDSPFLAG | - | - | - |
| `agc_job_ags_initialization_csm_state` (ags_initialization.c) | CSMPREC | - | - | - | - | job ags_initialization_sent |
| `ags_end` (ags_initialization.c) | AGSEND | - | -NODOFLAG | - | - | - |
| `agc_job_ags_initialization_zero_imu` (ags_initialization.c) | CKSTALL | - | - | - | - | job ags_initialization_zero_imu |
| `agc_job_ags_initialization_sent` (ags_initialization.c) | AGSWORD | IMUSE | - | - | - | - |
| `agc_job_lm_auto_optics_orient` (auto_optics_positioning.c) | R52B | - | -3AXISFLG | - | - | - |
| `fine_align_pick` (fine_align.c) | LOCSAM | - | - | ALARM 0405 | - | - |
| `agc_job_lm_fine_align_data_checked` (fine_align.c) | R54 | FREEFLAG | - | - | - | - |
| `agc_job_lm_fine_align_torqued` (fine_align.c) | R55 | - | -PFRATFLG | - | - | - |
| `agc_lm_imu_check` (imu_orientation_determination.c) | IMUCHK | - | +IMUSE | ALARM 0210 | - | - |
| `agc_job_lm_orientation_data_checked` (imu_orientation_determination.c) | CHKSDATA | FREEFLAG | +REFSMFLG | - | - | - |
| `latlong_options` (imu_realign.c) | - | ERADFLAG LUNAFLAG | - | - | - | - |
| `agc_job_lm_imu_realign_landing_site` (imu_realign.c) | P52LS | - | +LUNAFLAG -ERADFLAG | - | - | - |
| `agc_align_refmf` (lunar_surface_align.c) | REFMF | - | +ATTFLAG | - | - | - |
| `pack_options` (lunar_surface_align.c) | PACKOPTN | ATTFLAG REFSMFLG | - | - | - | - |
| `by_lm_attitude` (lunar_surface_align.c) | BYLMATT | - | +INITALGN | - | - | - |
| `agc_job_lunar_surface_options_chosen` (lunar_surface_align.c) | DSPOPTN | ATTFLAG REFSMFLG | - | ALARM 0701 | - | - |
| `agc_job_lunar_surface_attitude_check` (lunar_surface_align.c) | ATTCHK | ATTFLAG | - | - | - | - |
| `agc_job_lunar_surface_realign` (lunar_surface_align.c) | P57JUMP | - | -INITALGN | - | - | - |
| `agc_lm_lunar_surface_surfline` (lunar_surface_align.c) | SURFLINE | INITALGN | - | - | - | - |
| `agc_job_lunar_surface_data_checked` (lunar_surface_align.c) | DOALIGN | FREEFLAG | - | - | - | - |
| `agc_job_lunar_surface_solve` (lunar_surface_align.c) | INITBY | INITALGN | - | - | - | - |
| `agc_job_lunar_surface_aligned` (lunar_surface_align.c) | SURFDISP | INITALGN | +REFSMFLG | - | - | - |
| `agc_job_lunar_surface_post_enter` (lunar_surface_align.c) | P57POST | - | +LUNAFLAG -ERADFLAG | - | - | - |
| `next_sample` (lunar_surface_gravity.c) | GREED | - | - | BAILOUT 1203 | - | task lunar_gravity_sample |
| `agc_task_lunar_gravity_sample` (lunar_surface_gravity.c) | GRABGRAV | - | - | - | - | job lunar_gravity_accumulate |
| `agc_lm_lunar_surface_gravity` (lunar_surface_gravity.c) | GVDETER | - | -REFSMFLG | - | - | - |
| `agc_job_lunar_gravity_measured` (lunar_surface_gravity.c) | LUNG | - | -FREEFLAG | - | - | - |
| `agc_job_lunar_gravity_recycle` (lunar_surface_gravity.c) | UPFLAG | - | +FREEFLAG | - | - | - |
| `agc_job_lunar_gravity_accepted` (lunar_surface_gravity.c) | PROGRAV | FREEFLAG | - | - | - | - |
| `agc_lm_lunar_surface_coarse_trim` (lunar_surface_gravity.c) | COATRIM | INITALGN | - | - | - | - |
| `agc_job_lunar_surface_star_acquisition` (lunar_surface_sighting.c) | R59 | REFSMFLG | - | - | - | - |
| `agc_job_lunar_surface_star_chosen` (lunar_surface_sighting.c) | R59A | - | - | ALARM 0404 | - | - |
| `surface_marks` (aotmark.c) | MARKTYPE | SURFFLAG | - | - | - | - |
| `agc_aotmark` (aotmark.c) | AOTMARK | - | - | POODOO 1211, BAILOUT 1211, BAILOUT 1207 | - | job aot_mark_star_display |
| `agc_job_aot_mark_check` (aotmark.c) | MARKCHEX | - | - | ALARM 0111, BAILOUT 1203 | - | task aot_mark_release |
| `surface_again` (aotmark.c) | SURFAGAN | - | - | ALARM 0107 | - | - |
| `vacstor` (aotmark.c) | VACSTOR | - | - | - | - | job aot_mark_next_action |
| `mark_button` (aotmark.c) | XMKRUPT YMKRUPT | - | - | ALARM 0107, ALARM 0114 | - | - |
| `agc_aotmark_markrupt` (aotmark.c) | MARKRUPT | - | - | ALARM 0113, ALARM 0112 | ch16 AOTREJECT, ch16 YAXISMARK | job aot_mark_next_action |
| `agc_job_rr_los_display` (lem_geometry.c) | RRLOSDSP | - | - | - | - | job rr_los_display_next |
| `agc_job_rendezvous_final_attitude_maneuver` (rendezvous_final_attitude.c) | V06N18 | - | -3AXISFLG | - | - | - |
| `align_stall` (common.c) | IMUSTALL | - | - | CURTAINS 0217 | - | - |
| `align_stall_outcome` (common.c) | - | - | - | CURTAINS 0217 | - | - |
| `align_set1_pdt` (common.c) | SET1/PDT | - | +DRIFTFLG | - | - | - |
| `sxtang1` (geometry_csm.c) | SXTANG1 | - | =CULTFLAG | - | - | - |
| `agc_job_imu_realign` (imu_realign.c) | PROG52 | PFRATFLG | -UPDATFLG -TRACKFLG | - | - | - |
| `agc_job_coarse_align_fineonly` (imu_realign.c) | FINEONLY | - | +REFSMFLG -PFRATFLG | - | - | - |
| `agc_job_gycrs` (imu_realign.c) | GYCRS | - | -DRIFTFLG -REFSMFLG | - | - | - |
| `agc_job_gycrs_torqued` (imu_realign.c) | GYCRS | - | -PFRATFLG +REFSMFLG | - | - | - |
| `agc_align_picapar` (locsam.c) | PICAPAR | - | =VFLAG | - | - | - |
| `agc_align_sighting_data_display` (sighting_data_and_gyro_torquing.c) | CHKSDATA | - | +FREEFLAG | - | - | - |
| `agc_job_chksdata_enter` (sighting_data_and_gyro_torquing.c) | ENTER | - | -FREEFLAG | - | - | - |

### Shared erasables read and written

Accessors of `include/agc/core/state.h` and the lanes' public headers called by this lane, with the number of its functions that read / write each. Lane-private erasables are `static` in the lane's files or in its state struct; their scales are on the declarations.

| erasable (accessor) | meaning and scale (accessor prolog) | read by | written by |
|---|---|---|---|
| `agsk` | AGSK (LM): the AGS clock zero, sent on the downlists. AGC: AGSK (Luminary099 ERASABLE_ASSIGNMENTS.agc); returns agc_dp_t -- AGSK, cs 2^28 | 1 | 0 |
| `agsword` | none -- AGSWORD (LM): the downlist to return to after AGS initialization. RETURNS: AGSWORD; returns AGSWORD | 1 | 0 |
| `aotaz` | AOTAZ -1+n, AOTEL -1+n (LM, detent n = 1..6): AOT azimuth / elevation, CDU words (2^15 = 1 revolution). | 3 | 0 |
| `aotel` | AOTAZ -1+n, AOTEL -1+n (LM, detent n = 1..6): AOT azimuth / elevation, CDU words (2^15 = 1 revolution). | 1 | 0 |
| `cduchkwd` | CDUCHKWD (CSM): CDU check delay, cs (0 = use the default). | 2 | 0 |
| `cdus` | CSM optics shaft CDU raw word (CDUS / OPTX); LM: rendezvous radar shaft | 3 | 0 |
| `cdut` | CSM optics trunnion CDU raw word (CDUT / OPTY); LM: rendezvous radar trunnion | 3 | 0 |
| `cdux` | CDU X (outer gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (register CDUX, 032) | 5 | 0 |
| `cduy` | CDU Y (inner gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUY) | 5 | 0 |
| `cduz` | CDU Z (middle gimbal) counter, raw 15-bit word, 2^15 counts = 1 revolution (CDUZ) | 5 | 0 |
| `delv` | accumulated PIPA delta-V, stable-member axes (DELV, I(6)): PIPA pulses scale 2^14 (count in the high word); 1 pulse = 5.85 cm/s CSM, 1 cm/s LM. KPIP1  | 1 | 1 |
| `desopts` | DESOPTS (ERASABLE_ASSIGNMENTS.agc, CM) -- the shaft angle OPTDRIVE drives to, read RETURNS: DESOPTS, CDU word (2^15 = 1 rev); returns DESOPTS, CDU wor | 0 | 3 |
| `desoptt` | DESOPTT (ERASABLE_ASSIGNMENTS.agc, CM) -- the trunnion angle OPTDRIVE drives to, read RETURNS: DESOPTT, CDU word (2^15 = 1 rev); returns DESOPTT, CDU  | 0 | 3 |
| `dsptemx` | none (erasable read) DSPTEMX,+1 (= DSPTEM2 +1, +2) as a DP.; returns agc_dp_t -- DSPTEMX,+1 as a DP | 1 | 1 |
| `extvbact` | EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the extended verb activity bits, read RETURNS: EXTVBACT, 15 bits; returns EXTVBACT, 15  | 6 | 3 |
| `imodes30` | last-sampled channel 30 bits, T4RUPT IMUMON (IMODES30, B(1)PRM) | 2 | 0 |
| `imucadr` | C(IMUCADR) = MODECADR +0 (ERASABLE_ASSIGNMENTS.agc; read by SVCT3).; returns agc_sp_t -- the word (+0: nobody uses the IMU) | 1 | 0 |
| `landmark` | LANDMARK (CM P22 code ABCDE, V52), a signed SP number | 1 | 0 |
| `mark2dwn` | MARKDOWN +0..+6 / MARK2DWN +0..+6: mark data for the downlink (time high/low, CDUY, CDUS(OPTX), CDUZ, CDUT(OPTY), CDUX), raw words. | 0 | 1 |
| `markdown` | MARKDOWN +0..+6 / MARK2DWN +0..+6: mark data for the downlink (time high/low, CDUY, CDUS(OPTX), CDUZ, CDUT(OPTY), CDUX), raw words. | 0 | 2 |
| `markstat` | mark status bit word (MARKSTAT, B(1)PRM; SXTMARK/R21/R22/P20/P23) | 16 | 15 |
| `modecadr` | C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").; returns agc_sp_t -- the word (+0 none, positive a waiting job) | 0 | 1 |
| `modreg` | MODREG major mode (program) | 1 | 0 |
| `mrkbuf` | MRKBUF1 / MRKBUF2 (CM): two seven-word mark buffers (time DP, CDUY, word 0 = -1 (77776) marks a buffer empty. mrkbuf2 / set_mrkbuf2 read and store MRK | 2 | 2 |
| `optind` | OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0; returns OPTIND: -1, 0, 1, or  | 0 | 2 |
| `option1` | OPTION1 +index (index 0..2 = OPTION1, OPTION2, OPTION3; noun 06), octal. | 1 | 3 |
| `optionx` | none (erasable read) OPTIONX +index (= DSPTEM2 +1 +index).; returns agc_sp_t -- OPTIONX +index as SP | 1 | 1 |
| `optmodes` | OPTMODES (ERASABLE_ASSIGNMENTS.agc, CM) -- the optics mode bit word, read RETURNS: OPTMODES, 15 bits (0 in an LM build); returns OPTMODES, 15 bits (0  | 1 | 1 |
| `pipax` | PIPA X pulse count since last read (register PIPAX, 037), unscaled pulses: 1 pulse = 5.85 cm/s (CSM SERVICER207), 1 cm/s (LM) | 0 | 1 |
| `pipay` | PIPA Y pulse count (register PIPAY) | 0 | 1 |
| `pipaz` | PIPA Z pulse count (register PIPAZ) | 0 | 1 |
| `piptime` | time of the PIPA reading / this vehicle's state vector, centiseconds, scale 2^28 (PIPTIME, B(2)PRM) | 0 | 1 |
| `refsmmat` | reference-to-stable-member matrix, row-major, DP elements, scale 2^1 (REFSMMAT, I(18D)PRM; 'REFSMMAT SCALE OF B1') | 17 | 5 |
| `rls` | landing site vector, moon-fixed reference, meters, scale 2^27 (RLS; R30 'SCALED AT (-27)M'); used by P20/P22/P52 lat-long, R30, P57, landing | 4 | 1 |
| `rn` | this vehicle's position, meters, scale 2^29 (earth) or 2^27 (moon) (RN, B(6)PRM) | 0 | 1 |
| `rr_az` | RR-AZ, RR-ELEV (LM, noun 56): rendezvous radar LOS angles, revs 2^0. | 0 | 1 |
| `rr_elev` | RR-AZ, RR-ELEV (LM, noun 56): rendezvous radar LOS angles, revs 2^0. | 0 | 1 |
| `rrectcsm` | permanent CSM position, meters, scale 2^29 earth / 2^27 moon (RRECTCSM) | 2 | 0 |
| `samptime` | SAMPTIME: sampled TIME2,TIME1 (noun 65), cs 2^28. | 1 | 1 |
| `swsample` | SWSAMPLE (ERASABLE_ASSIGNMENTS, CM; set in OPTMON p.160) -- the optics mode switch as last sampled (read by V41 N91 and R52) RETURNS: +1 computer (CSC | 1 | 0 |
| `thetad` | desired gimbal angles THETAD +0..+2 (CPHI/CTHETA/CPSI), raw 15-bit CDU-style words, 2^15 = 1 revolution Index 0..2. | 4 | 4 |
| `tig` | time of ignition, centiseconds, scale 2^28 (TIG; DSP noun 33) | 1 | 1 |
| `time1` | ---- time: TIME1/TIME2 (centiseconds, DP), TIME3..TIME6 are exec-private | 1 | 0 |
| `tland` | TLAND (LM, = W): nominal time of landing, cs 2^28. | 1 | 1 |
| `vrectcsm` | permanent CSM velocity, m/cs, scale 2^7 earth / 2^5 moon (VRECTCSM) | 2 | 0 |

### AGC labels -> C functions

The "AGC" column above names the label each listed function translates; the complete label -> function -> file:line index for the lane (and the tree) is [docs/overview/xref.md](../overview/xref.md), generated by `tools/xref.py`.
