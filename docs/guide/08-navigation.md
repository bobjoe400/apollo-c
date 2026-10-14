# 8. Navigation

Navigation is knowing where you are: the position and velocity of your
own vehicle and of the other one, at a known time, in a known frame.
The AGC keeps them as state vectors, carries them forward in time by
integrating the equations of motion, and corrects them with
measurements -- star sightings, landmark sightings, radar ranges --
weighted by a running estimate of their errors. This chapter describes
each piece and then follows one radar mark through the LM's
rendezvous navigation (P20).

## 8.1 Frames and state vectors

Positions and velocities are kept in a *basic reference coordinate
system*, an inertial frame centred on the Earth or on the Moon,
whichever is the sphere of influence (MOONFLAG, chapter 5). The IMU's
stable member defines a second, platform frame; the 3x3 matrix
REFSMMAT turns reference vectors into stable-member vectors and is the
reason the IMU must be aligned (P51, P52: 8.6). Spacecraft-body axes are
a third frame, linked to the platform by the gimbal angles the CDUs
read.

The computer keeps two **permanent state vectors**, one for the CSM and
one for the LM, in both computers (RRECTCSM/VRECTCSM/TETCSM and
RRECTLEM/..., chapter 5's `agc_sv_get`). During powered flight the
vehicle's own state is RN, VN at PIPTIME, maintained by SERVICER
(chapter 9); in coasting flight it is the permanent one, integrated on
demand. Positions are at 2^29 m (2^27 m near the Moon), velocities at
2^7 m/cs, times at 2^28 cs (chapter 2).

## 8.2 Conic routines

Between perturbations, a spacecraft follows a conic: a Kepler orbit
about one body. CONIC_SUBROUTINES.agc (C pp.1262-1305) solves the
classic problems on conics with a universal variable, so the same code
handles circles, ellipses and hyperbolas. In C they are `src/nav/conic/`:

| Routine | Problem | C |
|---|---|---|
| KEPLER | state after a given time on the conic | `nav/conic/kepler.c:agc_conic_kepler` |
| LAMBERT | velocity that connects two positions in a given time | `nav/conic/lambert.c:agc_conic_lambert` |
| TIMETHET | time to travel through a given angle | `nav/conic/timetheta.c:agc_conic_timethet` |
| TIMERAD | time to reach a given radius | `nav/conic/timeradius.c:agc_conic_timerad` |
| APSIDES | apogee and perigee radii | `nav/conic/apsides.c` |

The listing introduces each with a full program description. KEPLER's
(C p.1263) begins:

```
#   THIS SUBROUTINE, GIVEN AN INITIAL STATE VECTOR AND THE DESIRED TRANSFER TIME THROUGH WHICH THE STATE IS TO
#   BE UPDATED ALONG A CONIC TRAJECTORY, COMPUTES THE NEW, UPDATED STATE VECTOR.  THE TRAJECTORY MAY BE ANY CONIC
#   SECTION -- CIRCULAR, ELLIPTIC, PARABOLIC, HYPERBOLIC, OR RECTILINEAR WITH RESPECT TO THE EARTH OR THE MOON. THE
...
```

and the C prolog summarises the method:

```c
 *   KEPLER solves sqrt(mu) TAU = f(x) for x by a secant iteration bounded
 *   by XMIN/XMAX (the bounds move in after every step; a step that would
 *   cross a bound is cut to 9/10 of the way to it), converging when
 *   |TAU - T| <= EPSILONT = |TAU| 2^-22.
```

Two things to notice. The iteration limit is the AGC's own (ITERCTR =
20); Power of Ten wants every loop bounded, and the AGC already was
except for the period-reduction step, which the C bounds and marks as a
DEVIATION (`nav/conic/kepler.c`, ASSUMPTIONS). And the conic code is the place
where the word-level arithmetic of chapter 2 mattered most: the whole
lane runs on `agc_mpac_*` and matches the emulator at tolerance 0
(the yaAGC comparison in `tests/oracle/conic.c`, chapter 12). The same listing page also says
why timing was never part of this translation's job: "THE AGC
COMPUTATION TIME IS APPROXIMATELY .061 SECONDS FOR INITIALIZATION, .065
SECONDS FOR THE FINAL COMPUTATIONS, PLUS .083 SECONDS FOR EACH
ITERATION" -- a Kepler solution could take two seconds; in C it takes
none (chapter 4).

## 8.3 The integrator

Real trajectories are perturbed: by the Earth's oblateness, by the
Moon and the Sun as third bodies, by the Moon's own irregular gravity.
ORBITAL_INTEGRATION.agc (C pp.1334-1354, L pp.1227-1248;
`nav/integration/orbital_integration.c`) integrates them with
**Encke's method**: it follows a reference conic and integrates only the
small *deviation* from it, which keeps the numbers small enough for
fixed point. The C file's ALGORITHM states one step:

```c
 *   One time step: TIMESTEP (origin / rectification test); INTGRATE
 *   YV = TDELTAV, ZV = TNUV; three Nystrom stages at H = 0, DT/2, DT:
 *   ACCOMP computes the disturbing acceleration FV, DIFEQ+0/+1/+2
 *   accumulate, DIFEQCOM predicts the deviation, FBR3 advances the conic
 *   by DT/2 (KEPLER); ENDSTATE stores TDELTAV, TNUV.
```

* **Step size.** DT is 0.3 sqrt(R^3/mu) -- about a twentieth of an
  orbit -- truncated to a multiple of 128 cs and limited to 4000 s
  (`nav/integration/init.c:norfinal`, NORFINAL C p.1323).
* **Rectification.** When the deviation grows too large -- 3/4 of its
  scale, or 1% of the radius -- the integrator starts a new conic
  through the actual state and zeroes the deviation
  (`nav/integration/orbital_integration.c:rectest`, RECTEST C p.1344):

  ```c
      if (!rect)
      {
          rect = (agc_dp_sub(ip, d, K_3_4) >= 0); /* DSU BPL  */
      }
  ```

* **Disturbing accelerations.** ACCOMP (`accomp`, C p.1337) sums the
  primary body's term relative to the conic, the third bodies when
  outside the near-body region (MIDFLAG), and the gravity harmonics
  (OBLATE: the Earth's J2-J4, or the Moon's QUALITY terms).
* **Sphere of influence.** With MIDFLAG set, each step tests whether the
  vehicle has crossed into the other body's sphere and, if so, changes
  origin (`agc_integ_chkswtch`, `agc_integ_origchng`) and MOONFLAG -- and
  with it the scale of every position.
* **Overflow.** If a stage overflows, GOBAQUE backs the conic up and
  rectifies; with no deviation left, POODOO 00430.

The integration is a shared resource -- one set of erasables -- so a
job takes it with INTSTALL and sleeps if it is busy (chapter 4, 4.5).
Programs call it through entrances that set the flags they want:
CSMPREC/LEMPREC (precision, to a time), CSMCONIC/LEMCONIC (conic only),
INTEGRV (the permanent state, with or without W) (`nav/integration/init.c`).
In P00 the computer integrates both permanent states forward every ten
minutes on its own (STATEINT, `nav/integration/jobs.c:agc_task_stateint`), so they are
never far behind the clock.

## 8.4 The W matrix and measurement incorporation

The AGC navigates with a recursive estimator: each measurement moves
the state by an amount weighted by how uncertain the state and the
measurement are. The uncertainty is the **W matrix**, the "error
transition matrix" -- "W W^T = covariance", as the prolog of
`nav/auxiliary/measurement.c` puts it -- 6x6 for
position and velocity or 9x9 with three bias terms (radar or landmark
biases; DMENFLG). It is integrated together with the state when DIM0FLAG
asks for it, initialised from pad-loaded variances, and invalidated if
it overflows (alarm 00421).

The incorporation routines are MEASUREMENT_INCORPORATION.agc (C
pp.1252-1261, `nav/auxiliary/measurement.c`). The listing describes
INCORP1 (C p.1252):

```
#  INCORP1--PERFORMS THE SIX DIMENSIONAL STATE VECTOR DEVIATION FOR POSITI
# ON AND VELOCITY OR THE NINE DIMENSIONAL DEVIATION OF POSITION,VELOCITY,A
# ND RADAR OR LANDMARK BIAS.THE OUTPUT OF THE BVECTOR ROUTINE ALONG WITH T
# HE ERROR TRANSITION MATRIX(W) ARE USED AS INPUT TO THE ROUTINE.THE DEVIA
# TION IS OBTAINED BY COMPUTING AN ESTIMATED TRACKING MEASUREMENT FROM THE
# CURRENT STATE VECTOR AND COMPARING IT WITH AN ACTUAL TRACKING MEASUREMEN
# T AND APPLYING A STATISTICAL WEIGHTING VECTOR.
```

The C file's prolog writes the algorithm out; in words: for a scalar
measurement q with
sensitivity vector b (the BVECTOR: dq/dx), residual delta-q and variance
alpha^2,

* z = W^T b,
* a = z . z + alpha^2,
* omega = W z,
* delta-x = omega (delta-q / a),

and INCORP2 updates the state by delta-x and W by W - gamma omega z^T,
with gamma = 1 / (a + sqrt(a alpha^2)). The C functions carry the
listing's names: `agc_incorp_state_deviation` (INCORP1),
`agc_incorp_update_w` and `agc_incorp_update_state` (INCORP2's two halves,
names from `docs/NAMING.md`). In the C of INCORP1 the first step is
exactly z = W^T b, computed block by block with MXV as the listing does
(label Z123, C p.1253):

```c
        for (int a = 0; a < 3; a++)
        { /* TIX,2 Z123      */
            agc_vec_t z = agc_vec_zero(); /* ZI[a], scale of W B */

            for (int b = 0; b < AGC_INCORP_BLKS; b++)
            {
                agc_mat_t m = incorp_part(w, b, a);                 /* scale of W */
                agc_vec_t t = agc_mat_vec(ip, &m, inc->bvector[b]); /* scale W B */

                z = agc_vec_add(ip, t, z); /* VAD* (first pass: 0 + t) */
            }
```

and the second, a = z.z + alpha^2, keeps the triple-precision sums the
listing keeps ("VSQ RTB TPMODE keeps the full DOTSUB triple of each
square"). The rest of the routine is fixed-point housekeeping: gamma is
computed with a normalisation count so that it fits, and z is rescaled
by its largest component -- the listing's way of doing a square-root
filter in 28-bit fractions.

## 8.5 Walkthrough: one rendezvous-radar mark (LM P20)

In lunar orbit before rendezvous, the LM's P20 points the rendezvous
radar at the CSM and, while the radar is locked, takes **marks**:
range, range rate, and the radar's two gimbal angles
(shaft and trunnion). Routine R22 incorporates each mark into the state
of one vehicle -- the CSM's or the LM's, by the crew's choice (VEHUPFLG).
The C is in `lm/navigation/` (P20 and R22) with the shared rendezvous
core in `programs/navigation/`; the incorporation is
`lm/navigation/measurement_update.c`, whose prolog quotes the listing's
own description (LSR22.3, P20-P25.agc L pp.574-587):

> "This routine computes the B-vectors and delta Q for each of the
> quantities measured by the rendezvous radar (range, range rate, shaft
> and trunnion angles). The routine calls the INCORP1 and INCORP2
> routines which compute the deviations and correct the state vector. If
> the computed state vector deviations exceed the maximum permitted, the
> routine returns to R22LEM to display the deviations."

Step by step:

1. **Read the radar.** The radar data arrive through RADARUPT
   (`lm/radar/read.c:agc_rendz_radar_read`, chapter 4); R22 checks that
   the radar line of sight is within 3 degrees of where the state vectors
   say the CSM is (alarm 00525 otherwise: the radar has locked on
   something else, and P20 goes back to designating)
   (`docs/modules/prog-rendz.md`, "Program flow and decisions").
2. **Bring both states to the mark time.** `mu_integrate` (LSR22.3, L
   pp.575-576) takes the integrator (INTSTALL; the job may sleep here),
   integrates the vehicle *not* being corrected to MKTIME, then the one
   being corrected, with its W matrix (DIM0FLAG) and STATEFLG set so the
   results become the permanent states:

   ```c
       rz_flag_put(AGC_FLAG_VINTFLAG, !csm); /* the other vehicle first */
       ...
       rz_flag_put(AGC_FLAG_DIM0FLAG, w); /* NOTWCSM  */
       ...
       rz_flag_put(AGC_FLAG_STATEFLG, true);
       return (mu_intgrcal(ip));
   ```

   If W was not valid (no marks yet, or after an overflow), WLINIT
   first builds a diagonal W from pad-loaded variances
   (`mu_wlinit`, WLINIT L p.584: "W = 0 except the diagonal: position
   WRENDPOS ... velocity WRENDVEL ... shaft and trunnion bias").
3. **One measurement at a time.** `mu_run` (L pp.576-582) takes the
   four quantities in order. For each it computes b, delta-q and the
   variance; for range (`mu_range`, RANGEBQ L p.576) b is the unit line
   of sight, delta-q the measured range minus the range the state
   vectors predict, and the variance the larger of a pad-loaded floor
   and a fraction of the range squared:

   ```c
       mu.inc.bvector[0] = mu_sign(mu.ulc, false);
       ...
       agc_dp_t r        = agc_dp_sub(ip, lmnav_state()->rm, rlc); /* m 2^29 */
       mu.inc.deltaq = mu_slr_star(ip, r, 2 - mu.x2);            /* B-29/B-27 */
       ...
       mu.inc.variance = vmin;                     /* RVARMIN unless larger (QOK) */
   ```

4. **Weigh it.** INCORP1 gives delta-x (`mu_incorp1`, LGCUPDTE L p.586).
   If the implied change of position or velocity exceeds the pad-loaded
   limits RMAX/VMAX, the routine stops and shows the crew the size of
   the change on N49, to accept (PRO) or reject:

   ```c
           if (big)
           {
               return (lmnav_residuals_display()); /* R22LEM96 */
           }
   ```

5. **Apply it.** Otherwise INCORP2 updates the state and W, a restart
   point is recorded in group 2 (so a restart does not apply the same
   measurement twice), and the next quantity follows:

   ```c
           st = rz_incorp2(ip, &mu.inc, agc_flag_test(AGC_FLAG_LMOONFLG)); /* ASTOK: INCORP2, MOONTHIS = LMOONFLG */
           ...
           rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE_CONTINUE);
           mu.step = mu_next(mu.step);
   ```

The loop is bounded (`MU_STEPS`, "cannot happen: there are four") as
Power of Ten asks, and every wait -- the integration, the N49 question --
is a separate job, so `lm/navigation/measurement_update.c` has three entry jobs
(`agc_job_rendezvous_tracking_data_incorporate`, `_measure`,
`_measure_continue`) where the listing has one stretch of code with
two places to wait. On the lunar surface (SURFFLAG) the same routine
takes only range and range rate, with a 6x6 W, and always corrects the
CSM (LSR22.4).

The CSM side is the same algorithm with different measurements: sextant
marks on the LM and VHF ranging in P20 (`csm/navigation/tracking_data.c`),
landmark marks in P22, star-horizon sightings in P23 for translunar
navigation (`csm/navigation/cislunar_navigation.c`).

## 8.6 Aligning the platform (P52, in brief)

Attitude navigation is the IMU's job: its stable member holds an
inertial orientation, but the gyros drift, so every few hours the crew
realigns it with P52 (`programs/alignment/imu_realign.c`, shared;
`csm/alignment/`, `lm/alignment/`; P51-P53.agc C pp.739-762, L
pp.927-952). The program is mostly a dialogue (chapter 6 showed its first
question): choose an orientation (preferred, nominal, landing site, or
keep REFSMMAT), coarse-align the gimbals if they are more than a degree
off, sight two stars with the sextant or telescope, compare the measured
angle between them with the catalogue's (R54: "star angle difference",
N05), compute the small rotation that best fits both (AXISGEN) and the
gyro torquing angles (CALCGTA, chapter 3), and pulse the gyros (R55,
IMUPULSE). The whole flow is drawn in
`docs/overview/flowcharts/imu-realign.md`.

Next: [9. Guidance and control](09-guidance-and-control.md).
