# Assignment: status-handling   (tier: translation -- reads AGC failure paths)

Runs after all translation lanes finish and BEFORE integration. You own the
whole tree. Goal: zero `(void)agc_...` casts in src/ (rule 7 as restated in
docs/CODING_STANDARD.md), done properly, not by deleting the cast.

Phase A -- make infallible APIs void (core/exec headers, lead-owned, you
may edit for this task):
1. From `grep -rhn '(void) *agc_' src | grep -o 'agc_[a-z_]*' | sort | uniq -c`,
   take every callee whose failure is only "bad argument" when called with
   constants: agc_flag_set/clear/invert, agc_flagword_set, agc_chan_write/
   or/and, agc_phase_change, agc_phase_change_var, agc_phase_kill,
   agc_exec_prio_change, agc_rupt_set_t and similar. Change them to return
   void; keep the AGC_ASSERT on arguments inside. Add
   `void agc_interp_clear_ovf(agc_interp_t *ip)` and use it where code
   void-casts agc_interp_test_ovf() just to clear OVFIND.
2. Update all callers with a Coccinelle patch (tools/archive/status_void.cocci):
   `(void)f(args);` -> `f(args);` for those callees only. Fix tests.

Phase B -- genuinely fallible calls (waitlist_call/vardelay/longcall,
exec_novac/findvac/job_wake, alarm/prioalarm, phase_newphase, chan_write
with computed values, nav/conic/integration entry points, accessors that
can return NOT_FOUND/UNIMPLEMENTED):
3. For each remaining `(void)agc_` site, open the AGC source at that
   label (tools/agc_label.py) and translate the failure path the AGC has
   there. Patterns: the AGC's WAITLIST aborts internally with 1203 and
   FINDVAC bails out with 1201/1202 -> in C the caller raises
   `agc_bailout(AGC_ALARM_...)` and returns; a job body returns the status
   to the executive; an interrupt handler records the alarm and continues.
   Where the AGC truly ignores a result (rare), write a one-line comment
   quoting the AGC text, and call through a tiny named helper rather than
   casting (e.g. `agc_alarm_ignore_dup(code)`), so the intent is a name,
   not a cast.
4. Keep functions <= 60 lines; splitting is allowed.
5. `make`, `make test`, `make check` (0 hard violations), and the oracle
   tests must pass as before. Write docs/history/waves/status-handling.md with
   the per-callee decisions and a Lessons section.
