// status_void.cocci -- rule 7 (docs/CODING_STANDARD.md): callers of the
// APIs that status-handling made void (they cannot fail when called with
// valid arguments; bad arguments are caught by AGC_ASSERT inside).
//
//   spatch --sp-file tools/status_void.cocci --in-place --dir src
//
// 1. `(void) f(args);`  ->  `f(args);`
// 2. `return f(args);`  ->  `f(args); return AGC_STATUS_OK;`
//    (ALARM family: `return AGC_STATUS_ALARM;`, the status the caller's
//    caller saw before)
// Assignments such as `st = f(args);` are fixed by hand: the compiler
// reports each one ("void value not ignored").

@voidcast@
identifier f =~ "^(agc_flag_set|agc_flag_clear|agc_flag_invert|agc_flagword_set|agc_time_set_cs|agc_chan_write|agc_chan_or|agc_chan_and|agc_phase_change|agc_phase_change_var|agc_phase_change2|agc_phase_kill|agc_phase_newphase|agc_exec_prio_change|agc_rupt_set_t5_task|agc_rupt_set_t6_task|agc_rupt_set_time|agc_alarm|agc_prioalarm|agc_exec_varalarm|agc_curtains)$";
expression list es;
@@
- (void) f(es);
+ f(es);

@retok@
identifier f =~ "^(agc_flag_set|agc_flag_clear|agc_flag_invert|agc_flagword_set|agc_time_set_cs|agc_chan_write|agc_chan_or|agc_chan_and|agc_phase_change|agc_phase_change_var|agc_phase_change2|agc_phase_kill|agc_phase_newphase|agc_exec_prio_change|agc_rupt_set_t5_task|agc_rupt_set_t6_task|agc_rupt_set_time)$";
expression list es;
@@
- return f(es);
+ f(es);
+ return AGC_STATUS_OK;

@retalarm@
identifier f =~ "^(agc_alarm|agc_prioalarm|agc_exec_varalarm|agc_curtains)$";
expression list es;
@@
- return f(es);
+ f(es);
+ return AGC_STATUS_ALARM;
