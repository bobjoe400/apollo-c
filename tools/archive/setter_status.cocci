// setter_status.cocci -- rule 7 (docs/CODING_STANDARD.md, "no silent
// failures"): a setter whose store sits under a positive assertion,
// `if (AGC_ASSERT(c)) { store; }`, skips the store silently when c fails.
// It becomes a fallible check: the setter returns agc_status_t (BAD_ARG
// when c fails, else OK after the store) and every caller handles it
// (by hand, from the compiler's "void value not ignored" being absent:
// tools/check_rules.py and review).  The regex is the record of the
// finish lane (docs/history/waves/finish.md).
//
//   spatch --sp-file tools/archive/setter_status.cocci --in-place \
//          -I include --include-headers <files>

@def@
identifier f =~ "^(agc_erase_set_(cduxd|cduyd|cduzd|delcdux|delcduy|delcduz|omegapd|omegaqd|omegard|delperor|delqeror|delreror|aosq|aosr|stiksens|neg_ratedb)|agc_imu_set_gcompsw|agc_imu_set_pipa_interval|rz_set_csm_residuals)$";
parameter list ps;
expression C;
statement list S;
typedef agc_status_t;
@@
- void
+ agc_status_t
  f(ps)
  {
  ...
- if (AGC_ASSERT(C))
- {
-   S
- }
+ if (!AGC_ASSERT(C))
+ {
+   return (AGC_STATUS_BAD_ARG);
+ }
+ S
+ return (AGC_STATUS_OK);
  }

@proto@
identifier f =~ "^(agc_erase_set_(cduxd|cduyd|cduzd|delcdux|delcduy|delcduz|omegapd|omegaqd|omegard|delperor|delqeror|delreror|aosq|aosr|stiksens|neg_ratedb)|agc_imu_set_gcompsw|agc_imu_set_pipa_interval|rz_set_csm_residuals)$";
parameter list ps;
typedef agc_status_t;
@@
- void
+ agc_status_t
  f(ps);
