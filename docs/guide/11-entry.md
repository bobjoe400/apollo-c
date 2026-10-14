# 11. Entry

The last thing the CMC does is bring the Command Module through the
atmosphere to a splashdown point, and it steers by *rolling*. The CM
flies with a fixed lift-to-drag ratio (LAD, "max L/D",
`include/agc/csm/entry.h`); rolling the vehicle points that lift up,
down or sideways, and the vertical part of it (L/D, "desired L/D
(vertical plane)") decides how far the capsule flies. Entry guidance computes, every two seconds, the
vertical L/D it wants, and turns that into a roll command for the entry
autopilot. This chapter follows P61 to P67 (P61-P67.agc C pp.789-818,
REENTRY_CONTROL.agc C pp.844-875, CM_ENTRY_DIGITAL_AUTOPILOT.agc C
pp.1063-1092; `src/csm/entry/`). The whole flow is drawn in
`docs/overview/flowcharts/earth-entry.md`.

## 11.1 Preparation: P61 and P62

**P61** (`csm/entry/programs.c:agc_job_entry_preparation`) checks the
IMU, starts SERVICER if it is not already running, and tests that the
platform is oriented for entry: the IMU's Y axis must be within 30
degrees of the orbit normal UNIT(VN x RN); the reverse is alarm 01427
("IMU reversed"), anything else 01426 (`agc_job_entry_imu_axis_test`).
Then it shows the **splash target** the crew loaded: V06 N61, the
latitude and longitude of the splash point, LAT(SPL) and LNG(SPL), and
HEADSUP (fly heads up or down). On PRO it computes the entry conditions
from the state vector: the predicted maximum g and the velocity and
flight-path angle at entry interface (N60), the range to go and time to
entry (N63), and the initial data for the crew's backup entry monitor
(S61.2, `agc_entry_ems_data`).

**P62** (`agc_job_cm_sm_separation...`) asks the crew to separate the CM
from the service module (V50 N25, "perform"), switches on the entry
DAP (CM/DAPON), and sets NODOFLAG -- from now on V37 is refused with
alarm 1520 (chapter 7, 7.7). The DAP turns the CM to the entry attitude
and P62 waits until it is near trim before going to P63.

## 11.2 The guidance pass

From P63 on, SERVICER (chapter 9, 9.1) runs every 2 s with AVGEXIT =
CM/POSE (`csm/entry/cm_body_attitude.c:agc_job_cm_pose`), which computes the body
attitude and jumps through POSEXIT into the guidance. REENTRY_CONTROL
is one long interpretive routine that branches from phase to phase with
GOTOs and keeps its place in an erasable, GOTOADDR. The C splits it at
its own section boundaries into step functions in six files
(`csm/entry/entry_guidance.c`, `csm/entry/entry_targeting.c`, `csm/entry/entry_initial_roll.c`,
`csm/entry/entry_huntest.c`, `csm/entry/entry_upcontrol.c`, `csm/entry/entry_final_phase.c`) and runs
the chain with a bounded loop (`csm/entry/entry_guidance.c:entry_sequencer`):

```c
    for (n = 0; n < AGC_ENTRY_SEQ_MAX_STEPS; n++)
    {
        if (s == ST_ENDEXIT || s == ST_SERVNOUT || s == ST_ERROR)
        {
            return (s);
        }

        if (s < ST_HUNTEST)
        {
            s = entry_initial_roll_step(p, s);
        }
        else if (s < ST_GOTOUPSY)
        {
            s = entry_huntest_step(p, s);
        }
        ...
```

Each step returns the label the AGC branches to next; the enum order
follows the listing's sections. This is the C answer to a routine that
is one 30-page web of GOTOs: no `goto`, no recursion, and each step a
function short enough to read beside its listing page.

Every pass starts with **TARGETNG** (`csm/entry/entry_targeting.c:agc_entry_targeting`):
the target vector RT, rotated with the Earth to the predicted time of
arrival; THETAH, the range angle to it; D, the measured drag
acceleration; and the measured L/D. Then GOTOADDR selects the phase.

## 11.3 The phases

The programs P63 to P67 are not chosen by the crew: the guidance changes
the major mode as the flight passes each phase boundary
(`csm/entry/programs.c:entry_post_0_05g`, `entry_upcontrol`, `entry_ballistic`,
`entry_final_phase`).

* **P63 entry initialisation** waits for 0.05 g of drag -- the edge of
  the sensible atmosphere -- holding the initial roll attitude.
* **P64, post 0.05 g** (INITROLL, then HUNTEST,
  `csm/entry/entry_initial_roll.c`, `csm/entry/entry_huntest.c`). At 0.05 g the program
  shows N74 (roll command, velocity, drag). For a long entry, HUNTEST
  predicts the range the capsule would fly with a reference
  skip-out ("upcontrol") trajectory and compares it with THETAH,
  adjusting the reference L/D by a secant step until the difference is
  under 25 nautical miles; the iteration is bounded per pass, a
  DEVIATION (`docs/overview/flowcharts/earth-entry.md`).
* **P65, upcontrol** (`csm/entry/entry_upcontrol.c`): fly the reference trajectory
  that climbs out of the dense atmosphere to extend the range.
* **P66, ballistic** (KEP2): above the atmosphere, hold the roll
  attitude; the program waits until drag builds again.
* **P67, final phase** (PREFINAL, PREDICT3, `csm/entry/entry_final_phase.c`):
  predict the range to go, steer out the range error DNRNGERR with L/D, and stop steering below VQUIT, when the
  CM is too slow for lift to matter. P67 then shows range to go,
  latitude and longitude (V16 N67) and, at the end, switches the DAP off
  and goes to P00 (`agc_job_entry_final_phase_end`).

Along the way the commanded L/D is limited: a g-limiter reduces
downward lift when deceleration approaches its maximum (GLIMITER), and
a lateral corridor around the target decides when to reverse the
direction of the roll to bring the cross-range error back (LIMITL/D,
LATSW, `step_limitld`).

## 11.4 From L/D to roll

Every path through the phases ends at label L355 (C p.873), which turns
the desired vertical L/D into a roll angle. Side by side
(`csm/entry/entry_final_phase.c:step_roll_command`):

```
L355       DLOAD   DDV         # ROLLC = ACOS( (L/D1) / LAD)
                   L/D1
                   LAD         # MPAC SET TO +-1 IF OVERFLOW***
           SR1     ACOS
           SIGN    CLEAR
                   K2ROLL
                   NOSWITCH
           STORE   ROLLC
```

```c
    /* MPAC set to +-1 if overflow; the AGC has no BOV here, so OVFIND is
     * left as DDV set it.                                               */
    m         = agc_dp_div(ip, es->l_d1, es->lad);
    m         = agc_dp_acos(ip, agc_dp_shr(m, 1));
    es->rollc = agc_dp_sign(m, es->k2roll);
    agc_flag_clear(AGC_FLAG_NOSWITCH);

    return (ST_ENDEXIT);
```

If the CM's full lift is LAD and the guidance wants a vertical
component L/D1, the roll angle is arccos(L/D1 / LAD); K2ROLL says
whether to roll left or right, the lateral logic having chosen. The
`SR1` before ACOS is chapter 2's scaling at work: interpretive ACOS
takes its argument halved ("arg = sin(theta)/2 ... scaled at 2",
`include/agc/core/interp.h`), and returns revolutions. DDV saturates on
overflow, which is why the listing's comment can say "MPAC SET TO +-1
IF OVERFLOW": an L/D1 larger than LAD asks for full lift up or down.
ENDEXIT then shows the phase's display and the pass ends.

## 11.5 The entry autopilot

The entry DAP (`csm/entry/dap.c`) flies ROLLC with the CM's own RCS
jets. Its file PURPOSE summarises it:

```c
 *   The CM entry digital autopilot. READGYMB (.1 s waitlist task): gimbal
 *   angle differences, BODYRATE (P, Q, R x TCDU/180), then via T5RUPT
 *   ATTRATES: ALFA, BETA, ROLL integration and the atmospheric (rate
 *   damping) or extra-atmospheric (attitude hold) pitch/yaw jets on
 *   channel 5. Every 2 s (SETJTAG from READACCS) the predictive roll
 *   system CM/RCS: the phase-plane solution TON1/TOFF/TON2 executed by
 *   JETCALL waitlist tasks on channel 6.
```

Pitch and yaw are only rate-damped in the atmosphere, while roll, the steering axis, gets a
phase-plane solution every 2 s: when to fire, how long to coast, when
to fire again, each step a waitlist task that sets the roll jets on
channel 6 (`dap_jetlist`). If the waitlist is full, BAILOUT 1203 is
recorded and the pass commands no jets.

## 11.6 Walkthrough summary

| Step | Program | C | What the crew sees |
|---|---|---|---|
| IMU check, splash target | P61 | `agc_job_entry_preparation` .. `_compute` | V06 N61, N60, N63 |
| CM/SM separation, DAP on | P62 | `agc_job_cm_sm_separation*`, `csm/entry/dap.c:agc_entry_dap_ic` | V50 N25, V06 N61, N22 |
| wait for 0.05 g | P63 | `agc_job_entry_initialization`, `agc_entry_startent` | N64 |
| first drag; range iteration | P64 | `entry_initial_roll_step`, `entry_huntest_step` | N74 |
| skip-out reference | P65 | `entry_upcontrol_step` | V16 N69 |
| above the atmosphere | P66 | `entry_ballistic_step` | N22 |
| final steering, to VQUIT | P67 | `entry_final_phase_step` | V16 N67 |
| DAP off, P00 | -- | `agc_job_entry_final_phase_end` | V37 flash |

In the NASSP mission the C flew this sequence to splashdown and matched
the stock emulator's run; the bug found on the way was an unmapped
erasable (UNITW, "splashdown 2.6 deg short", flight bug 13, chapter 5,
5.8), not the guidance itself.

Next: [12. Reading further](12-reading-further.md).
