# Flowcharts

Mermaid flowcharts drawn from the C control flow of the final tree. Each
node names the C function (`file:function`) that implements the step, so
a reader can check the chart against the code; the AGC label and listing
page of every function are in its prolog and in [../xref.md](../xref.md).
The scheduler and restart model they rest on is described in
[../README.md](../README.md).

Scheduler paths:

* [fresh-start.md](fresh-start.md) -- power-on, GOJAM hardware restart,
  V36 fresh start (DOFSTART).
* [software-restart.md](software-restart.md) -- BAILOUT / POODOO /
  ENEMA, the phase tables and how each restart group is re-run.
* [job-preemption.md](job-preemption.md) -- NOVAC / FINDVAC, NEWJOB,
  CHANJOB, ENDOFJOB, JOBSLEEP, CHANG1.
* [waitlist-tick.md](waitlist-tick.md) -- WAITLIST insertion, TIME3 and
  T3RUPT, LONGCALL, ENDTASK.
* [program-change.md](program-change.md) -- V37: refusals, SERVICER and
  integration hand-off, P00 or the new program through a software restart.

Program sequences (program numbers in text only, NAMING.md):

* [sps-burn.md](sps-burn.md) -- CSM P40 SPS burn (and P41 RCS burn).
* [imu-realign.md](imu-realign.md) -- P52 IMU realign (CSM, LM notes).
* [lunar-landing.md](lunar-landing.md) -- LM P63 braking through
  P64-P67, touchdown (P68), aborts.
* [earth-entry.md](earth-entry.md) -- CSM P61-P67 entry.
* [powered-ascent.md](powered-ascent.md) -- LM P12 ascent and the ascent
  guidance (also P70/P71).

Conventions: continuations of a flashing display are written V33/PROCEED,
V34/TERMINATE and ENTER; "n.m" is restart group n, phase m; channel bits
are "ch NN bit b". A chart shows the decisions the code makes; arithmetic
and scaling are in the lane notes (`docs/modules/<lane>.md`, "Program flow
and decisions") and the function prologs.
