# 9. Guidance and control

Navigation (chapter 8) says where the spacecraft is. *Guidance* decides
where it should go and computes the commands to get there: which way
to point, when to light the engine, when to shut it off. *Control* --
the digital autopilots -- turns those commands into jet firings and
engine-gimbal movements many times a second. This chapter covers the
pieces every burn uses: the servicer that measures acceleration, the
targeting program P30, the burn programs P40 and P41, the autopilots,
and the attitude manoeuvre routine; it ends with a walkthrough of a CSM
SPS burn from five seconds before ignition to the residual trim.

## 9.1 SERVICER: average-G navigation

During thrusting, integrating the equations of motion is not enough:
the engine's acceleration is unknown in advance. So the AGC measures it.
The IMU's accelerometers (PIPAs) count velocity increments as pulses
into counter cells; every two seconds the READACCS task reads and zeroes
them, and the SERVICER job advances the state with the measured
velocity change and the computed gravity. This is the *average-G*
method: gravity is averaged over the interval, between its value at the
old and the new position. SERVICER runs whenever AVEGFLAG is set --
during burns, landing, ascent and entry -- and at its end calls the
current program's guidance through an erasable job id, **AVGEXIT**
(chapter 4, 4.7; `docs/overview/flowcharts/sps-burn.md`).

The equations are in CALCRVG (SERVICER207.agc C p.836; Luminary's
SERVICER.agc L pp.880-881). Listing and C
(`guidance/servicer_common.c:agc_guid_calcrvg`, CSM branch):

```
CALCRVG    VLOAD   VXSC
                   DELV
                   KPIP1
           VXM     VSL1
                   REFSMMAT
           STORE   DELVREF     # DELV IN REF COORDS AT 2(+7)
           VSR1    PUSH
           VAD     PUSH        # (DV-OLDGDT)/2 TO PD SCALED AT 2(+7)M/CS
                   GDT/2
           VAD     VXSC
                   VN
                   2SEC(22)
           VAD     STQ
                   RN
                   31D
           STCALL  RN1         # TEMP STORAGE OF RN SCALED 2(+29)M
                   CALCGRAV
           VAD     VAD
           VAD
                   VN
           STCALL  VN1         # TEMP STORAGE OF VN SCALED 2(+7)M/CS.
```

```c
        agc_vec_t dv = agc_vec_scale(ip, delv, KPIP1_CM); /* PIPA delta-V, SM, m/cs 2^7; VXSC KPIP1 */
        a->delvref = agc_vec_shl(ip, agc_vec_mat(ip, dv, refsmmat), 1); /* VXM VSL1 */
        ...
    agc_vec_t half_dv = agc_vec_shr(a->delvref, 1);                               /* DELVREF/2, m/cs 2^7; VSR1 PUSH */
    agc_vec_t avg_acc = agc_vec_add(ip, half_dv, a->gdt_2);                       /* DELVREF/2 + GDT/2, m/cs 2^7; VAD PUSH GDT/2 */
    agc_vec_t vdt     = agc_vec_scale(ip, agc_vec_add(ip, avg_acc, sv->v), dt22); /* position change, m 2^29; VAD VN, VXSC */

    a->rn1          = agc_vec_add(ip, vdt, sv->r);                 /* VAD RN; STORE RN1 */
    agc_status_t st = agc_guid_calcgrav(ip, veh, moon, a->rn1, a); /* status of CALCGRAV */
    ...
    agc_vec_t v1 = agc_vec_add(ip, a->gdt1_2, sv->v); /* VN + GDT1/2, m/cs 2^7; VAD VN */

    v1     = agc_vec_add(ip, v1, avg_acc); /* VAD (pop)   */
    a->vn1 = agc_vec_add(ip, v1, half_dv); /* VAD (pop)   */
```

In equations, with measured delta-v Δv (rotated from the platform frame
to the reference frame by REFSMMAT) and g0, g1 the gravity at the old and
new positions:

* r1 = r0 + (v0 + Δv/2 + g0 Δt/2) Δt
* v1 = v0 + Δv + (g0 + g1) Δt/2

The push-down list carries Δv/2 and the averaged term; the C names
them `half_dv` and `avg_acc`, and the three un-addressed `VAD`s at the
end pop them in the order the listing pushed them. The prolog also
records a difference between the ropes: "Comanche scales DELV by KPIP1
before the VXM; Luminary does the VXM first".

## 9.2 Targeting: P30

A burn is planned before it is flown. P30 (EXTERNAL DELTA-V, both
vehicles; P30-P37.agc C pp.635-648, `guidance/external_delta_v.c`) takes
a burn computed on the ground: the crew loads the time of ignition
(N33) and the velocity change in local-vertical coordinates (N81). The
program integrates the state to TIG and rotates the delta-v into the
reference frame (S30.1, `external_delta_v_vectors_csm`, C pp.639-640):

```c
    rz->unrm  = agc_vec_unit(ip, agc_vec_cross(ip, rz->rtig, rz->vtig), NULL, NULL);
    agc_mat_t m; /* LOMAT rows, half unit (2^1) */

    agc_rendz_lomat(ip, rz, &m);

    rz->delvsin                   = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1); /* DELVSLV = DELVLVC: one vector, rz->delvlvc */
    agc_guid_state()->burn.vgdisp = agc_vec_abval(ip, rz->delvsin);                       /* MAG DELV */
```

then displays the resulting apogee, perigee and burn magnitude for the
crew to check (N42). (Note the comment: DELVSLV and DELVLVC are one
vector under two names, chapter 5, 5.3.) The rendezvous targeting
programs (P32-P35, P72-P75) and the CSM's Lambert-aimpoint P31 end the
same way: with a TIG and a velocity to be gained left for the burn
program (`docs/modules/prog-rendz.md`, "Targeting").

## 9.3 The burn programs

P40 (CSM SPS burn, `csm/burns/sps/`; LM DPS burn, `lm/burns/dps_burn.c`)
and P41 (RCS burn, both vehicles) fly the planned burn. P42 is the LM's
ascent-engine burn. Their structure, which the walkthrough in 9.6
follows:

* **Before TIG:** check the IMU (R02), compute the initial thrust
  direction and the velocity to be gained VG (S40.1,
  `guidance/burn_steering.c:agc_guid_initial_velocity_to_gain`), the
  preferred attitude (S40.2,3), and manoeuvre to it (R60, 9.5). On the
  CSM, test and trim the engine gimbals (V50 N25: "gimbal test?").
  Integrate the state to TIG - 30 s, then hand over to a chain of
  waitlist tasks timed from TIG.
* **At TIG - 30 s** start SERVICER; **at TIG - 5 s** ask the crew to
  enable the engine (V99) and size the burn (S40.13 TIMEBURN); **at TIG**
  ignite.
* **During the burn**, every SERVICER pass runs the steering, which
  updates VG and commands the attitude rate that turns the thrust
  toward VG -- **cross-product steering**:

  ```c
   * XPRODUCT (Comanche S40.8 p.720): cross-product steering,
   *   OMEGAC = KPRIMEDT * SMNB(REFSMMAT (UNIT(VG) x UNIT(c BDT - DELVREF))).
   * With the velocity-to-be-gained along the thrust (c BDT - DELVREF
   * parallel to VG) the commanded rate is zero.
  ```

  (`guidance/burn_steering.c:agc_guid_steering_cross_product`). When
  the time to go drops below 4 s, the steering stops and a waitlist task
  is set for the exact cut-off.
* **After cut-off**, show the residuals and let the crew trim them with
  the RCS.

## 9.4 Digital autopilots

A **DAP** (digital autopilot) is a control loop that runs in an
interrupt at a fixed rate, reads the gimbal angles, compares the
attitude and rate with what guidance wants, and fires jets or moves the
engine. Each vehicle has its own, and only one owns the T5 interrupt at
a time (`docs/modules/dap-csm.md`, "Program flow and decisions": FLAGWRD6
bits 15-14 say which: 01 RCS, 10 TVC).

* **CSM RCS DAP** (`csm/dap/rcs/`; RCS-CSM_DIGITAL_AUTOPILOT.agc and
  neighbours). Every 100 ms: filter the rates; 20 ms later, decide each
  axis in a *phase plane* of attitude error against rate error, with a
  deadband and switching lines, and compute a jet on-time; 20 ms later,
  select jets per quad and start TIME6 to time them
  (`csm/dap/rcs/t5rupt.c`, PURPOSE). The phase-plane step is
  `csm/dap/rcs/attitude.c:csm_rcs_jets` (JETS .. DOJET,
  AUTOMATIC_MANEUVERS.agc C pp.1030, 1035): "for each axis (Z, Y, X)
  locate (rate error, attitude error) in the phase plane and compute the
  jet on-time TAU".
* **CSM TVC DAP** (`csm/dap/tvc/`, TVCDAPS.agc). During an SPS burn the
  engine bell is steered instead of the jets: the pitch and yaw loops
  alternate as T5 tasks, take the rate command OMEGAC from the
  steering, integrate the error, pass it through a sixth-order digital
  filter, add the trim, limit the command to 6 degrees and send it to the
  engine-gimbal servos through channel 14; a roll loop holds roll with
  the RCS (`docs/modules/dap-csm.md`).
* **LM RCS DAP** (`lm/dap/`, the "P-axis" and "Q,R-axes" autopilots).
  Every 100 ms (T5), with jet firings timed by T6RUPT to 1/1600 s; it
  uses a jet-torque model (1/ACCS) recomputed as mass changes and, in
  descent, the gimbal trim system (GTS), which drives the descent
  engine's trim gimbals in place of Q,R-axis jets when it can
  (`docs/modules/dap-lm.md`, "Program flow and decisions").

The DAPs are the most "basic-code" part of the program -- written in
machine instructions, not the interpreter, for speed -- so their C uses
the word-level helpers of `csm/dap/mysubs.c` and `lm/dap/arith.c`
("Basic (machine) code on ones'-complement words").

## 9.5 Attitude manoeuvres: R60 and KALCMANU

Before a burn, a star sighting or a tracking pass, the spacecraft must
turn to a new attitude. R60 (`imu/attitude_maneuver.c`, R60_62.agc C
pp.390-398) shows the target gimbal angles on V06 N18 and, if the
crew's switches allow it, calls KALCMANU (`imu/maneuver.c`), which
computes a single rotation axis and angle from the present attitude to
the target and feeds the DAP a moving target attitude every second.
The C file's NOTES are a clear summary:

```c
 *   A maneuver: GOMANUR saves the user's return (ATTCADR) and priority
 *   and runs KALCMAN3 in the user's job.  KALCMAN3 finds the rotation
 *   axis COF and angle AM taking the present attitude to CPHI, CTHETA,
 *   CPSI, and DEL, the rotation the attitude makes in one second at the
 *   selected rate.  Every second (UPDTCALL -> NEWDELHI) the desired
 *   attitude matrix MIS is advanced by DEL and the DAP is given new
 *   desired CDU angles and the per-0.1 s increments; in the last second
 *   MANUSTOP sets the final angles and ENDMANU / GOODMANU restarts the
 *   user's job at its priority.
```

It was in KALCMANU's matrix code that the major-word sign rule of
chapter 2 (SIGNMPAC, flight bug 15) made the difference between a
0.01-degree correction and a 40-degree swing.

## 9.6 Walkthrough: P40 from TIG - 5 s to the residuals

The CSM's SPS burn. By TIG - 35 s the set-up is done: P40 has computed
VG, turned to the burn attitude, trimmed the gimbals, integrated the
state to TIG - 30 s and set restart point 4.21 with a LONGCALL of the
TIGBLNK task. TIGBLNK (TIG - 35) blanked the DSKY and scheduled TIGAVEG
(TIG - 30), which started SERVICER and scheduled the next link
(chapter 4 showed TIGBLNK beside its listing).

**TIG - 5 s.** `csm/burns/sps/burn.c:agc_task_tig_5_csm` (TIG-5, C
p.694) schedules TIG-0, asks for the V99 flash by setting NVWORD1, and
starts S40.13 to size the burn:

```c
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIG_0); /* status of agc_waitlist_call */
    ...
    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_ENABLE_REQUEST; /* WILL CAUSE V99 FLASH   */
    agc_phase_change2((uint16_t) 040074, (uint16_t) 000033, NULL);

    return (agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN));
```

The once-a-second clock job (CLOCKJOB) sees NVWORD1 and shows V06 N40
with V99 pasted over it (chapter 6, 6.7): "engine enable?", with the
time to ignition counting down in R1. S40.13 TIMEBURN
(`guidance:agc_guid_timeburn_csm`) decides from VG and the engine's
thrust whether the burn is *impulsive* (too short to steer: IMPULSW).

**V99 PRO.** The crew's answer runs `csm/burns/sps/clock.c:agc_job_engine_enable_proceed`,
which sets ASTNFLAG ("astronaut said go"); if TIG has already come
(IGNFLAG), it schedules IGNITION at once:

```c
        agc_flag_set(AGC_FLAG_ASTNFLAG);

        if (agc_flag_test(AGC_FLAG_IGNFLAG))
        { /* ENDV99PI               */
            agc_status_t st = agc_waitlist_call(1, AGC_TASK_IGNITION); /* status of agc_waitlist_call */
```

**TIG.** `agc_task_tig_0_csm` (TIG-0, C p.695) sets IGNFLAG and ignites
only if the crew has answered; otherwise it waits for the PRO above.
Either order works -- the two flags meet:

```c
    agc_flag_set(AGC_FLAG_IGNFLAG);

    if (!agc_flag_test(AGC_FLAG_ASTNFLAG))
    {
        return (AGC_STATUS_OK); /* WAIT FOR V99P          */
    }
    ...
    agc_phase_change((uint16_t) 000614);               /* 4.61 = IGNITION        */
    return (agc_task_ignition_csm());
```

**Ignition.** IGNITION (`csm/burns/sps/ignition.c:agc_task_ignition_csm`,
C p.695) beside its listing:

```
IGNITION   CAE     CDUX        # SAVE FOR ROLL DAP REFERENCE OGAD
           TS      OGAD        #   V99PJOB (CLOCKJOB) SETS UP IGNITION
           EXTEND              #   TASK (4.61 PROTECTION)
           DCA     TIME2       #   FOR RESTARTS
           DXCH    TEVENT
           CS      FLAGWRD5    # SET ENGONFLG
           MASK    BIT7
           ADS     FLAGWRD5
SPSON      CAF     BIT13       # TURN ON SPS ENGINE
           EXTEND
           WOR     DSALMOUT
IMPULCHK   CAF     BIT9        # CHECK FOR IMPULSIVE BURN
           MASK    FLAGWRD2
           CCS     A
           TCF     IMPLBURN    # IMPULSIVE
           CS      FLAGWRD6    # NON-IMPULSIVE, SET STRULLSW FOR STEERULL
           MASK    BIT13
           ADS     FLAGWRD6
```

```c
    agc_csm_dap_com()->ogad = agc_erase_cdux(); /* ROLL DAP REF.    */
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_flag_set(AGC_FLAG_ENGONFLG);
    agc_chan_or(011, AGC_CSM_SPSON);            /* SPSON                  */

    if (agc_flag_test(AGC_FLAG_IMPULSW))
    { /* IMPULCHK               */
        agc_flag_clear(AGC_FLAG_STRULLSW); /* IMPLBURN               */
        agc_erase_set_tig(agc_erase_tgo() + (agc_dp_t) agc_time_now_cs());
        ...
        agc_status_t st = agc_waitlist_call((uint16_t) agc_dp_low(agc_erase_tgo()), AGC_TASK_ENGINOFF); /* status of agc_waitlist_call */
        ...
    }

    agc_flag_set(AGC_FLAG_STRULLSW); /* NON-IMPULSIVE          */

    return (agc_csm_preptvc());
```

The engine is a bit on channel 11 (DSALMOUT bit 13). An impulsive burn
gets its cut-off task now; a steered burn sets STRULLSW so that steering
starts after the ullage jets stop. PREPTVC then hands T5 from the RCS
DAP to the TVC DAP (DOTVCON 0.4 s later), and DOSTRULL 1.6 s after that
turns steering on (`csm/burns/sps/ignition.c:agc_task_dotvcon`, `agc_task_dostrull`).

**Every 2 s.** READACCS -> SERVICER (9.1) -> AVGEXIT = STEERING
(`csm/burns/sps/steer.c:agc_job_steering_csm`). S40.8 updates the
velocity to be gained by what the accelerometers measured and what
gravity added (`csm/burns/sps/steer.c:agc_csm_cross_product_steering`, C p.719):

```c
    g->burn.vg     = agc_vec_add(ip, agc_vec_sub(ip, g->burn.bdt, g->avg.delvref), agc_erase_vgtig()); /* BVSU VAD VGPREV        */
    g->burn.vgdisp = agc_vec_abval(ip, g->burn.vg);
```

then, if thrust is seen, estimates the time to go (TGOCALC) and either
commands the cross-product rate OMEGAC for the TVC DAP or, with less
than 4 s to go, stops steering and sets the cut-off task (`steer.c:
short_burn_cutoff`). If the measured delta-v falls below a threshold the
engine has failed: LOTHRUST shows V97 ("engine fail") over N40, the
mirror of V99 (`csm/burns/sps/steer.c:lothrust`). The DSKY meanwhile shows V06 N40:
time to cut-off, VG, delta-v so far.

**Cut-off.** ENGINOFF (`csm/burns/sps/ignition.c:agc_task_enginoff_csm`, C pp.697-698)
records restart points and runs DOSPSOFF: engine bit off, ENGONFLG down,
mass bookkeeping; 2.5 s later DOTVCRCS gives T5 back to the RCS DAP and
starts POSTBURN (`docs/overview/flowcharts/sps-burn.md`, "Cut-off and
after").

**Residuals.** POSTBURN (`agc_job_postburn_csm`, C p.686) flashes V16 N40
with the final values. PRO goes to P40RCS, which makes SERVICER compute
the remaining velocity to be gained in *body* axes (AVGEXIT = CALCN85)
and then TIGNOW flashes V16 N85 (`agc_job_tignow_csm`, C p.687): VG
along the spacecraft's X, Y and Z axes, which the crew nulls with the
RCS hand controller while watching the numbers fall. PRO or V34 then
ends the program through POST41 and GOTOPOOH (`agc_job_post41`).

The P41 RCS burn is the same program with the engine replaced by the
crew's translation thrusters (ENG2FLAG); the LM's P40 is the same
skeleton with BURNBABY (`lm/burns/ignition/burnbaby.c`) as the shared
countdown, the descent engine's throttle, the DV monitor DVMON that
watches for thrust, and the LM DAP's gimbal trim where the CSM has its
TVC loop -- which chapter 10 follows into the landing.

Next: [10. Lunar descent and ascent](10-lunar-descent-and-ascent.md).
