# Assignment: sys-misc   (tier: translation)

Scope, into src/io/ (shared), src/csm/, src/lm/:
T4RUPT_PROGRAM.agc (the 120 ms T4RUPT: DSKY relay output DSPOUT, IMU/
optics/radar moding monitors, the OPTICS CDU drive (CM), the RR (LM),
the 1/4-second cycle counters) -> t4rupt.c; EXTENDED_VERBS.agc (V40-V99
handlers; both vehicles differ: shared ones once, vehicle ones in
extended_verbs_csm.c / extended_verbs_lm.c) ; DOWNLINK_LISTS.agc (the
downlist tables as static const arrays of erasable ids; both vehicles) and
DOWN_TELEMETRY_PROGRAM.agc (DNPHASE1/2, the 50-word frame, the DNTM1/2
channel output) -> telemetry.c; UPDATE_PROGRAM.agc (P27 uplink update:
V70-V73) -> p27.c; SERVICE_ROUTINES.agc (the small shared routines:
UPFLAG/DOWNFLAG, DELAYJOB, GENTRAN, B5OFF, SETCDU? etc.) -> service.c;
AGC_BLOCK_TWO_SELF_CHECK.agc (self-check and show-banksum; the memory-
test parts become a sim-side bank checksum over the rope image) ->
self_check.c; SYSTEM_TEST_STANDARD_LEAD_INS.agc, IMU_PERFORMANCE_TEST_2.agc
and IMU_PERFORMANCE_TESTS_4.agc (Luminary; P07 IMU tests) -> src/lm/imu_tests.c;
INTER-BANK_COMMUNICATION.agc (BANKCALL/SWCALL/POSTJUMP: these are bank
bookkeeping and mostly vanish; document that in a short comment file);
RTB_OP_CODES.agc / RT8_OP_CODES.agc (the RTB routines: LOADTIME, CDULOGIC,
SGNAGREE, the 1/PIPA interplay, FORMDEG etc. -- these ARE used by
interpretive code and must be translated as functions) -> rtb_ops.c.
Tests: tests/io/service_verbs_telemetry.c (compiles): the downlist for the CM coast
list has the documented word count; LOADTIME returns the current time;
SGNAGREE on a sign-disagreeing DP pair fixes it up.
