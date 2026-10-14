# 10. Lunar descent and ascent

The LM's descent from lunar orbit to the surface is the most demanding
thing either computer did: a long burn whose target moves (the
guidance rotates the landing site with the Moon every pass, TTFINCR), whose altitude is first known only
from the state vector and then measured by radar, and whose last
minutes the crew flies with the computer as co-pilot. This chapter
follows the landing as Luminary 099 programs it -- P63 braking, P64
approach, P66 rate of descent -- and then the ascent back to orbit,
P12. Code: `src/lm/landing/`, `src/lm/burns/`, `src/lm/ascent.c`;
listings THE_LUNAR_LANDING.agc (L pp.785-792),
LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (L pp.797-828),
THROTTLE_CONTROL_ROUTINES.agc (L pp.793-797), P12.agc and
ASCENT_GUIDANCE.agc (L pp.838-856). The whole sequence is drawn in
`docs/overview/flowcharts/lunar-landing.md` and `docs/overview/flowcharts/powered-ascent.md`.

## 10.1 One guidance loop, several phases

The landing is one guidance computation, LUNLAND, run every 2 seconds as
SERVICER's AVGEXIT (chapter 9, 9.1), whose behaviour is selected by an
erasable phase index WCHPHASE. The listing organises it as a table of
routines indexed by phase, and the C keeps the table as its file prolog
(`lm/landing/guidance.c`, from L p.798):

```c
 *   Flight sequence tables (p. 798), indexed by WCHPHASE:
 *     WCHPHASE  NEWPHASE  PREGUIDE  WHATGUID  AFTRGUID  WHATEXIT  WHATDISP
 *     -1 IGNALG TTFINCR   CALCRGVG  TTF/8CL   CGCALC    EXGSUB    --
 *      0 BRAK   TTFINCR   RGVGCALC  TTF/8CL   CGCALC    EXBRAK    P63DISPS
 *      1 APPR   STARTP64  REDESIG   TTF/8CL   CGCALC    EXNORM    P64DISPS
 *      2 VERT   P65START  RGVGCALC  VERTGUID  STEER?    --        VERTDISP
 *   The AGC's "INDEX WCHPHASE / TCF table" jumps become switch statements.
```

The phases end by *time to go*, not by altitude: braking ends when the
time to the braking target passes the pad-loaded TENDBRAK, approach
when it passes TENDAPPR (`lm/landing/guidance.c:ldg_extlogic`, EXTLOGIC L p.811):

```c
    if ((int32_t) tend + agc_lm_ttf8_high() <= 0)
    {
        return (l->wchphase); /* BZMF WHATEXIT    */
    }

    agc_lm_fastchng();
    l->wchphase = (agc_sp_t) (l->wchphold + 1); /* next phase       */
```

Each phase flies toward its own *target* -- a position, velocity and
acceleration in a guidance frame centred on the landing site -- loaded
on the pad. The major mode the crew sees (P63, P64) is set by the first
pass of the new phase (STARTP64, `agc_lm_start_approach_phase`).

## 10.2 The quadratic guidance law

The braking and approach phases use one law: command the acceleration
that would take the vehicle from its present state to the target state
along a trajectory whose acceleration varies linearly in time. The
listing gives it as published and as programmed (L p.808):

```
#   AS PUBLISHED --
#                     ___   __       ___   __
#       ___   ___   6(VDG + VG)   12(RDG - RG)
#       ACG = ADG + ----------- + ------------
#                       TTF        (TTF)(TTF)
#   AS HERE PROGRAMMED --
#                    ___   __
#             3 (1/4(RDG - RG)   ___   __)
#             - (------------- + VDG + VG)
#       ___   4 (    TTF/8               )   ___
#       ACG = ---------------------------- + ADG
#                         TTF/8
```

The rewrite is a scaling device: time to go is stored divided by 8
(TTF/8) so that it fits, and the constants 6 and 12 become 3/4 and 1/4.
The C (`lm/landing/guidance.c:agc_lm_quadguid`, QUADGUID L pp.808-809) follows the
"as programmed" form, with the lag-compensation coefficients `c[0..3]`
that the listing computes in basic code just before it:

```c
    agc_vec_t v1 = agc_vec_scale(ip, l->vgu, agc_dp_from_sp(c[0]));            /* VGU term, 2^10 m/cs */
    agc_vec_t v2 = agc_vec_scale(ip, t->vfg, agc_dp_from_sp(c[2]));            /* VDG term, 2^10 m/cs */
    agc_vec_t a  = agc_vec_div(ip, agc_vec_sub(ip, t->rfg, l->rgu), l->ttf_8); /* (RDG - RGU)/(TTF/8), 2^10 m/cs */

    a = agc_vec_scale(ip, agc_vec_shr(a, 2), agc_dp_from_sp(c[1]));
    a = agc_vec_add(ip, agc_vec_add(ip, a, v2), v1);
    a = agc_vec_scale(ip, agc_vec_div(ip, a, l->ttf_8), LDG_3_4DP);
    a = agc_vec_add(ip, agc_vec_scale(ip, t->afg, agc_dp_from_sp(c[3])), a);
```

Time to go itself is the root of a cubic built from the target's jerk,
acceleration, velocity and position along the downrange axis ("the
time-to-go cubic", `lm/landing/guidance.c:agc_lm_ttf8cl`, L pp.807-808), solved by
Newton iteration (TTF/8CL with ROOTPSRS, at most eight
passes, `lm/landing/guidance.c:agc_lm_ttf8cl`, `agc_lm_rootpsrs`). No root is
alarm 01406; any overflow in the guidance is alarm 01410, and both stop
the rate commands for the cycle (`agc_lm_steer_exit`).

From the commanded acceleration ACG, AFCCALC forms the thrust direction
UNFC/2 and magnitude /AFC/; THROTTLE turns the magnitude into engine
thrust, and FINDCDUW turns the direction into attitude commands for the
DAP (10.4).

## 10.3 P63: finding the ignition time

The descent burn begins at a time that is not given: P63 computes it.
IGNALG (`lm/landing/lunar_landing.c:ll_ignalg_prec`, `ll_ignaloop`, L
pp.785-786) integrates the LM's state to a trial time, runs the
guidance three times as if the burn were in progress, and from the
result corrects the ignition time by DDUM; it repeats until the
correction is small:

* integrate the LM state to TLAND - GUIDDURN with LEMPREC (it may sleep
  in INTSTALL: chapter 4 showed this exact code);
* IGNALOOP: three quadratic guidance passes (`ll_ign_guidance` ->
  `lm/landing/guidance.c:agc_lm_guidpass`), then DDUMCALC: TDEC1 = PIPTIME1 + DDUM;
* |DDUM| >= DDUMCRIT: integrate to TDEC1 and loop; 40 passes without
  convergence is alarm 01412;
* otherwise TIG = TDEC1 - ZOOMTIME, and the countdown begins
  (`docs/overview/flowcharts/lunar-landing.md`).

Before ignition P63 also asks for a fine alignment check (R51), turns to
the burn attitude, and checks that the landing radar antenna is in
position 1 (a channel 33 bit), asking the crew to move it with V50 N25
otherwise (`agc_lm_job_braking_phase_burn_attitude`,
`agc_lm_job_braking_phase_radar_init`). The countdown is BURNBABY, the
LM's shared TIG-35 / TIG-30 / TIG-5 / TIG-0 chain
(`lm/burns/ignition/burnbaby.c`), the same shape as the CSM's (chapter 9,
9.6) with V99 and ignition. At ignition the descent engine is at low
thrust; ZOOMTIME later the P63ZOOM task sets AVGEXIT = LUNLAND and full
thrust (`lm/burns/ignition/burnbaby.c:agc_task_braking_phase_zoom`, "FLATOUT (full
thrust)").

## 10.4 Throttle and attitude

The descent engine is throttleable, but not everywhere: THROTTLE
(`lm/landing/throttle.c`, L pp.793-797) "applies the 10%..94% region
logic with its 'throttle-up' to the fixed full-thrust region, and sends
PIF = FC - FP (plus an offset PIFPSET) to the THRUST counter" (file
PURPOSE): FC is the force guidance wants, FP the force the
accelerometers measured, both in throttle pulses; the difference is a
pulse train on the THRUST counter cell (chapter 1). The file is also an
example of why chapter 2 matters: "FP, FCODD, PIF and FWEIGHT are kept
as the AGC's two words ... DAS leaves them sign-disagreeing and the code
then reads the high word alone".

FINDCDUW (`lm/landing/findcduw.c`) converts the thrust and window
directions into gimbal-angle increments and rates for the DAP. It limits
the middle gimbal to 70 degrees from gimbal lock (alarm 00401) -- the
code of flight bug 7 in chapter 2.

## 10.5 The landing radar

Below about 50,000 ft the landing radar measures altitude and velocity,
and SERVICER folds the measurements into the state vector
(`lm/burns/servicer/landing_radar.c`, SERVICER.agc L pp.884-892). The
altitude update (`agc_lm_serv_updatchk`, UPDATCHK / POSUPDAT L
pp.884-886) projects the radar's slant range along its beam onto the
local vertical and compares it with the altitude computed from the state
vector:

```c
    agc_dp_t dh = agc_dp_sub(&ip, agc_vec_dot(&ip, sv, g->avg.unitr), g->lm.hcalc); /* DELTAH, m 2^24 */

    agc_lm_ldg()->deltah = dh; /* STORE DELTAH           */
```

A reasonableness test rejects a reading too far from the estimate; an
accepted one moves the position by "DELTAH * WH * (1 - H/HMAX)" --
a pad-loaded weight that falls to zero at HMAX -- along the local
vertical (`lm/burns/servicer/landing_radar.c`, comment at the GNUR computation): chapter
8's measurement idea with a fixed weight instead of a W matrix. The crew sees DELTAH on
N63 and can inhibit the update. The velocity beams follow (VMEASCHK).
This is where the C's one landing-specific ordering deviation lives:
the velocity read waits while the altitude read holds the radar
(chapter 4, 4.6) -- without it no radar update was ever accepted.

## 10.6 P64: the landing point designator

When braking ends, the approach phase P64 begins (STARTP64: the
landing point designator is enabled, RUPT10 on), and the computer shows on N64 "time
left for redesignation -- LPD angle" (`io/dsky/nouns/tables_lm.c`), the
angle at which the crew finds the site on the window's landing point
designator (LPD) scale. If the crew does not like the site, each
click of the hand controller moves it: the clicks arrive through the
hand-controller interrupt (RUPT10, `lm/landing/lunar_landing.c:agc_lm_pitfall`),
are counted by the REDESMON task every 70 ms, and REDESIG
(`agc_lm_redesig`, L pp.804-805) "move[s] the landing site by the
accumulated LPD increments ELINCR (about the Y body axis) and AZINCR,
keeping the line of sight below DEPRCRIT". The next guidance pass simply
flies to the new target.

## 10.7 P66: rate of descent, and the crew's modes

Near the surface the crew takes over the attitude and lets the computer
hold the descent rate. R13 (GUILDENSTERN, `lunar_landing.c:
agc_lm_guildenstern`, every 2 s) watches the crew's switches: the
appearance of attitude hold, or a click of the rate-of-descent switch,
starts **P66**; manual throttle starts P67
(`docs/overview/flowcharts/lunar-landing.md`). In P66, RODCOMP
(`lm/landing/lunar_landing.c:ll_rodcomp`, L pp.816-819) runs every second: each
click of the ROD switch changes the commanded vertical velocity, and the
throttle is driven to reach it:

```c
 * RODCOMP (pp. 816-819): P66 rate-of-descent guidance, every second
 * (from P66VERTA in the guidance job, and 1 s later from RODTASK).
```

It propagates the velocity from the last 2-second SERVICER state with
the accelerometer counts read since (`ll_rod_pipas`), because a second
is too long to wait at a few metres above the surface. Meanwhile the
landing analog displays (R10, `lm/landing/displays.c`) drive the
altitude/altitude-rate meter and the cross-pointers four times a second
-- the routine whose acceptance of a -0 altitude just after touchdown
was flight bug 8 (chapter 2).

After touchdown the crew selects P68 (landing confirmation): LANDJUNK
zeroes the attitude errors, sets the surface flags, computes the landing
site's latitude, longitude and altitude, and shows them on N43
(`agc_lm_job_landing_confirmation`).

## 10.8 Aborts

Throughout the descent R10/R11 (`lm/landing/aborts.c:
agc_lm_task_abort_monitor`, every quarter second) watches the ABORT and
ABORT STAGE switches: with aborts enabled (LETABORT), ABORT starts P70
(abort on the descent engine) and ABORT STAGE starts P71 (abort on the
ascent engine). Both continue in the ascent guidance below.

## 10.9 P12: powered ascent

Lift-off from the Moon is P12 (`lm/ascent.c`, P12.agc L pp.838-842).
The crew loads the time of ignition; the program builds the target --
an insertion orbit whose radius is the landing site radius plus 60,000
ft, in the CSM's orbital plane (QAXIS, chapter 5, 5.3) -- shows the
cross-range and apolune (N76), and runs the same BURNBABY countdown.

The ascent guidance (ATMAG and ASCENT, ASCENT_GUIDANCE.agc L
pp.843-856) is, in the file's words, "a linear-tangent-like law in the
frame of UNIT/R/ (radial), LAXIS (cross range) and ZAXIS1 (downrange):
the thrust acceleration components ATR = (A + B t)/TBUP - GEFF and
ATY = (C + D t)/TBUP, ATP from the remaining magnitude" (`lm/ascent.c`,
PURPOSE). Every 2 s:

1. **Vertical rise.** For the first seconds the thrust is radial
   (KEEPVR), until the LM is above the site by 25,000 ft or the line of
   sight to the CSM's plane and the rise rate allow pitch-over
   (`as_exitvr`).
2. **Time to go.** From the rocket equation: "MAINENG (p. 846): TGO =
   TBUP (VG/VE)(1 - KT VG/VE) - TTO" (`lm/ascent.c`, line 662), where TBUP
   is the time to burn up all the mass at the present thrust and VE the
   exhaust velocity.
3. **Steering.** The constants A, B, C, D are re-solved each pass
   (T2TEST / CONST), the thrust direction formed one second ahead
   (CMPONENT / AIMER), and FINDCDUW gives the attitude commands.
4. **Cut-off.** With less than 4 s to go, ENGOFF schedules the
   engine-off task at TGO (`as_engoff`); after cut-off the program shows
   V16 N63 and then V16 N85 -- the velocity residuals in body axes -- for
   an RCS trim, exactly as the CSM's P40 does (chapter 9, 9.6), and ends
   in P00 (TERMASC).

The ascent's erasables are shared by EQUALS with the descent's (QAXIS =
UHYP, ZAXIS1 = UHZP, TGO = DVCNTR +1, the burn's VGBODY): one C variable
each. Three flight bugs (9, 10, 11) were copies of these, and the
flowchart file ends by naming each single owner
(`docs/overview/flowcharts/powered-ascent.md`).

Next: [11. Entry](11-entry.md).
