# Assignment: oracle-cases   (tier: translation)

Definition-of-done item 6: every numeric module has committed yaAGC
differential cases. core-numeric (287) and nav-conic (35) exist; see
tools/oracle/README.md and tests/oracle/*.c for the format and the
VAC-snippet technique for interpretive entries. Each lane note lists its
proposed cases under "Oracle cases". Produce, per module, a spec in
tools/oracle/cases/<module>.spec, the yaAGC dump <module>.txt, and a test
tests/oracle/<module>.c that runs the C translation on the same inputs and
compares within a documented tolerance (bit-exact unless a -0 / stale-
register deviation is recorded in the module note):
  nav-integ (TIMESTEP one step earth/moon, RECTIFY, ACCOMP with oblateness),
  nav-aux (LAT-LONG, RP-TO-R, CALCTFF, LSPOS CM and LM, INCORP2),
  imu (CALCGTA, CALCGA, AXISGEN, IMUPULSE compensation, KALCMANU step),
  guidance (CALCRVG average-G one step, S40.1/S40.2 steering, DELTIME),
  landing (P63 guidance equations one pass, THROTTLE, FINDCDUW),
  entry (HUNTEST/UPCONTROL/PREDICT3 one pass, CM body attitude),
  dap-lm (TJETLAW one case set, 1/ACCS, Kalman one pass),
  dap-csm (jet selection table, TVC pitch filter one sample, ROLLDAP),
  targeting (CSI/CDH one case, TPI, INITVEL), return_to_earth (one
  conic search), dsky (DSPDECVN/HMS conversions, SFRUTAB).
Use `tools/agc_label.py` to find entries; for routines that need the
executive or interrupts, dump the pure-math subroutine the AGC calls
instead and say so. `make test` runs them all. docs/modules/oracle-cases.md
with per-case tolerance and any mismatch you could not explain (those go
to the review wave as findings).
