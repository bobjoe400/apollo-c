# review-dap-lm -- review of the LM DAP lane (src/lm/dap)

## Progress

* DONE (earlier run, commit 4f35a43): accs_aos_* oracle cases (13), AOSV -0
  modelled in 1/ACCONT (F10/F11 now bit-exact), INVERT overflow path,
  DV |dividend| = |divisor| case.
* DONE: docked 1/ACCS traced in the rope (R1); comment fixed; three docked
  oracle cases added (accs_docked_thrust/_aps/_fix), 1478 checks pass.
* DONE: BIGAOS 2 DB confirmed in the rope (R2).
* DONE: sim B2 (zero erasable): improper DV modelled at register level
  (port of yaAGC SimulateDV), TS/XCH stores wrap instead of saturate in
  1/ACCS and 1/ACCONT, data-dependent asserts replaced by word-range
  checks; unit test 8 (126 checks).
* DONE fidelity: accs.c (all 1/ACCS sections), accont.c (all 1/ACCONT).
* Next: rcs/*.c, gts.c, dapif.c, state.c fidelity; style; naming.

## Scratch build

Other review lanes edit the tree concurrently; SCRATCH/review-dap-lm/build.sh
builds a debug lib (falls back to HEAD for files that do not compile) and
t.sh runs one test against it.
