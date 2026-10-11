# WORKLOG.md -- lead's running log (newest at bottom)

2026-10-09  Cloned Apollo-11 (Comanche055 86 files / 65k lines, Luminary099
            91 files / 65k lines), NASA Power-of-Ten PDF, virtualagc.
            Wrote docs (PLAN, CODING_STANDARD, TRANSLATION_GUIDE, AGENT_BRIEF,
            ARCHITECTURE, MODULE_STATUS), include/agc contract headers,
            Makefile, tools/check_rules.py. Committed skeleton.
2026-10-09  Wave 1 launched: core-numeric, exec-sched, state-flags, oracle,
            nav-conic, nav-integ, nav-aux, io-pinball (8 agents, parallel).
            Wave 2 waits for state-flags + exec-sched (flag/channel/job ids).
            Wave 3 (integrate) after wave 2.

## Decisions
- Numeric model: fixed-point int16/int32/int64 mirroring SP/DP/TP; no doubles
  in flight code. Interpreter is a library, not an emulator.
- No function pointers: job/task dispatch by enum + switch (rule 9).
- Oracle: Virtual AGC yaAGC runs the real ropes; differential tests compare.
2026-10-09  exec-sched agent hit an API safeguard false positive on launch;
            relaunched with reworded prompt. User directed model tiering
            (Opus for translation, Sonnet for mechanical work); recorded in
            PLAN.md; applies from wave 2.
2026-10-09  Stopped the 7 Fable translation agents (~5 min in, still reading;
            negligible loss). Moved assignments to docs/agents/*.md; relaunched
            on tiers: core-numeric, exec-sched, nav-conic, nav-integ, nav-aux,
            io-pinball on Opus; state-flags and new io-nouns (noun tables split
            out of pinball) on Sonnet. oracle (Fable) left running: ropes
            already assembled.
2026-10-09  Wave 2 launched early (user: no reason to hold at 8). Pinned the
            naming convention for not-yet-existing flags/channels/accessors in
            docs/agents/_common.md so lanes can code ahead of state-flags.
            Running now: 17 agents (8 wave 1 + 9 wave 2: imu, guid-powered,
            guid-landing, guid-entry, dap-lm, dap-csm, prog-rendz, prog-align,
            sys-misc; all Opus). Wave 3 (integrate) after these report.
2026-10-09  oracle DONE. yaYUL/yaAGC built (tools/oracle/BUILD.md). The
            Apollo-11 repo sources do NOT assemble with current yaYUL; ropes
            built from virtualagc's Comanche055/Luminary099 copies, verified
            0 mismatches vs official binsource. agc_probe proven on SPSIN and
            interpretive SQRT/SIN via the VAC-snippet technique. Binaries and
            11.8 MB symtabs are gitignored (rebuild per BUILD.md).
2026-10-09  Usage limit killed 14 Opus lanes mid-work; io-nouns and
            state-flags finished clean first. Committed checkpoint (16.6k
            lines). All lanes have notes with Progress sections. Relaunching
            with resume semantics in two batches: A = core-numeric, exec-sched,
            nav-conic, nav-integ, nav-aux, io-pinball, imu; B = the rest after A.
2026-10-09  nav-aux DONE (0 stubs). Integration items: TEPHEM has two sources
            (shared accessor vs module pad-load struct) until core adds the
            pad-load accessors listed in its note; nav-integ owns INCORP2's
            restart/RECTIFY/INTWAKE; agc_dp_sign must be AGC SIGN semantics.
            Launching batch B lane guid-powered now (one at a time as A frees).
2026-10-09  exec-sched DONE (V37 partial). 197 checks pass. Integration items:
            deferred software restart (ENEMA runs at job/interrupt end, not
            inline, to avoid recursion); 44 provisional job ids + 51 task ids in
            src/exec/exec_pending.h (0x0E00 range) must move into agc_jobs.h and
            dispatch.c; duplicate AGC_JOB_GOTOPOOH in src/csm/entry_pending.h
            and src/programs/rendz_ext.h; AGC_JOB_GOPROG placeholder to remove;
            restart-table delays need core accessors; up-calls SERVIDLE,
            SETCOARS, SETMAXDB/SETMINDB; interrupt handlers T4RUPT/UPRUPT/
            DOWNRUPT/RADAR pending from sys-misc/io lanes.
            Launching batch B lane guid-landing.
2026-10-09  imu DONE (13 stubs: P06/POSTAND + 11 Comanche prelaunch entries).
            65 checks pass. Integration items: provisional ids in
            src/imu/imu_pending.h; MODECADR moved to exec storage (-0 stored
            as -2); prog-align's align_pending.h and guid-landing's
            landing_pending.h declare agc_imu_* with mismatched signatures ->
            adopt agc_imu.h; CALCSMSC duplicated in imu and prog-align (drop
            one); BUG exec-sched executive.c activity_light() passes 0177775 to
            agc_chan_and (mask > 077777 assertion); sim must drain GYROCMD /
            CDU?CMD via agc_imu_hw_set_*. Launching batch B lane guid-entry.
2026-10-09  core-numeric DONE (0 stubs), bit-exact vs yaAGC on 20,720
            interpreter runs and all 65,534 SPSIN/SPCOS inputs; 287 oracle
            cases committed with tests/oracle_core_numeric.c. Translation guide
            corrected (wrap-not-saturate; DMP/DMPR mapping). Found a yaAGC DV
            bug (SpToDecent sign-extension) -> patch note for oracle owner.
            Known deviation: stale DVSIGN on V/SC & UNIT overflow not modelled.
            Launching batch B lane dap-lm.
2026-10-09  nav-integ DONE (0 stubs). 177 checks; LEO J2-J4 10 min within 46 m,
            lunar J2/J3/J22/C31 1 h within 19 m, third-body 6 h within 42 m.
            Integration items: ids STATINT1/STATINT1_STALL/STATEUP_STALL/
            TASK_STATEINT (integ_pending.h); layering violation ATOPCSM/ATOPLEM
            call agc_align_svdwn() in src/programs -> move SVDWN to nav;
            TRKMKCNT accessor needed; agc_state.h comment wrong: RN/VN always
            2^29 m / 2^7 m/cs, not moon-scaled. Launching batch B lane dap-csm.
2026-10-09  io-pinball DONE (1 stub: V27 fixed-memory display). 182 checks
            pass with scratch stand-ins. Integration items: exec needs
            agc_exec_job_wake_as(loc,new_loc); state needs read/write_ecadr,
            optmodes/radmodes, set_samptime/option1/sfail; 15 jobs + 2 tasks
            (PIN_JOB_* in pinball_internal.h); align_pending.h and
            guid_pending_csm.h must switch to agc_dsky_display(). Launching
            batch B lane prog-rendz.
2026-10-09  nav-conic DONE (1 stub: LEMCONIC SURFFLAG landing-site path).
            40/40 unit, 299/299 oracle checks (35 yaAGC cases). Known gap:
            -0 / mixed-sign words shift exactly-circular KEPLER by ~2^-14.
            Integration items: prog-rendz rendz_ext.h declares its own conic
            types and a different agc_conic_apsides signature -> switch to
            agc_nav.h (already told prog-rendz to code against nav-conic names).
            Launching batch B lane prog-align.
2026-10-10  Second usage-limit cutoff killed 7 wave-2 lanes mid-work.
            Checkpoint committed. Wave 1 complete. Per user: keep full
            parallelism. Relaunching all 8 remaining lanes at once:
            guid-powered, guid-landing, guid-entry, dap-lm, dap-csm,
            prog-rendz, prog-align, sys-misc.
2026-10-10  guid-entry DONE (1 stub: P37 return-to-earth, ~1500 lines of
            interpretive code; only XT1LIM translated -> sibling agent
            `p37` queued for wave 3). 23 checks pass; range prediction
            1304.0 NM matches a double-precision model within 1 NM. Requests
            in src/csm/entry_pending.h: agc_svc_set_extvbact (sys-misc),
            agc_guid_preread1_csm (guid-powered), agc_csm_prelaunch_p11_earthr
            (imu), agc_csm_dap_stickchk (dap-csm), agc_dsky_gopin (io-pinball),
            temp ids 0x0C00-0x0C83.
2026-10-10  guid-landing DONE (0 stubs). 17 checks; throttle 319 pulses
            matches FC-FP; on-trajectory gives zero correction. Pending in
            src/lm/landing_pending.h: THRUST/ALTM/CDUT,SCMD accessors,
            TEMX/Y/Z, agc_align_refmf, agc_navaux_set_latlong, rendz flip/
            clradmod, M22/M32, ids 0x0F01-0x0F25 / 0x0F81-0x0F86; exec needs a
            TBASE2 setter; test has LANDING_TEST_FAKE_THRUST to drop.
            User rule: no P/R/V/N or page numbers in identifiers ->
            docs/NAMING.md; rename agent launched for done lanes; running
            lanes notified.
2026-10-10  NAMING RESET. First rename pass (Sonnet, mechanical) produced
            stutter and pasted label fragments (entry_entry_upcontrol,
            AGC_TASK_..._TTASK, _E/_P/_T suffixes). User paused everything.
            All 8 agents stopped; tree checkpointed as-is (partial renames +
            lane progress). NAMING.md gained 8 derivation rules (read the
            AGC source for every label; no opaque suffixes; no stutter;
            2-4 words; map before apply). Single Opus `naming` agent runs
            phase 1 (inventory + map only); lead reviews docs/RENAMES.md,
            then phase 2 applies tree-wide; then all 7 lanes resume
            (guid-powered, dap-lm, dap-csm, prog-rendz, prog-align, sys-misc,
            p37) with instructions to re-read NAMING.md and RENAMES.md.
2026-10-10  Installed coccinelle 1.1.1, universal-ctags 5.9, clangd 14
            (user ran apt). Renames now applied via Coccinelle semantic patch
            (identifiers only; comments keep AGC labels); inventory via ctags.
2026-10-10  Naming phase 1 reviewed and approved: 783-entry inventory
            (ctags + regex), 551 renames, 66 keeps, 3 file moves; every row
            cites the AGC comment. Key findings: V97/V99 T/P/E = crew
            Terminate/Proceed/Enter responses; several first-pass names had
            wrong meanings (R10/R11 abort monitor, N61 event-timer reset,
            CANV37 continues V37). Phase 2 (Coccinelle apply) running.
            Baseline: 156 files compile, 21 fail (paused lanes vs changed
            headers). After phase 2: resume the 7 lanes.
2026-10-10  Naming phase 2 DONE: 446 identifiers via Coccinelle (21 rules);
            spatch 1.1.1 honours only the last --dir, so run per dir; it
            reflows lines, so layout preserved by token-compare. Compile
            failures 21 -> 5 (all pre-existing, paused lanes). Comments
            byte-identical. 42 guid-landing rows restored and reviewed.
            OPEN for integration: guid_pending.h vs exec_pending.h define
            ~40 of the same job/task ids with different values; merge into
            agc_jobs.h. Restructure (subsystem subfolders) launched.
2026-10-10  Restructure DONE: 139 files into subsystem subfolders
            (io/{dsky,service}, nav/{conic,integration,aux}, programs/
            {rendezvous,alignment}, csm/{entry,burns,optics,dap/rcs,dap/tvc},
            lm/{landing,burns,dap/rcs,dap/common}); 13 shared private
            headers promoted to include/agc/; compile parity (172/177, same
            5 failing). NOTE: the moves landed in commit 7aafe81, whose
            message is the assignment doc change (git add -A swept them in).
            Follow-up running: strip folder-redundant filename prefixes;
            rename nav/aux -> nav/auxiliary (Windows reserved name).
2026-10-10  Filename pass DONE: 83 files lost folder-redundant prefixes;
            nav/aux -> nav/auxiliary; compile parity (172/177). Pause over.
            Resuming all 7 lanes (guid-powered, dap-lm, dap-csm, prog-rendz,
            prog-align, sys-misc, p37) with docs/agents/_resume_after_naming.md.
2026-10-10  Folder rules tightened (user): no common/ folders (lm/dap/common
            flattened); >=3 files sharing a prefix -> subfolder named by the
            prefix (core/interp, dsky/pinball, dsky/nouns done by lead;
            csm/burns/sps delegated to guid-powered); agc_ dropped from
            src/core .c names (public headers keep it). All moved files
            compile. Rules recorded in ARCHITECTURE.md "Folder rules".
2026-10-10  Header tree foldered (user): include/agc/<subsystem>/<name>.h,
            agc_ prefix dropped from header file names; single-lane
            internal/pending headers moved back beside their code; cross-lane
            shims stay public with _internal/_pending suffix for integration
            to fold. All include lines rewritten (public, header-to-header,
            private relative). Compile: 169+5 pass; remaining failures are
            lane-owned undeclared symbols (prog-align, sys-misc, guid-powered
            mid-edit) plus aborts.c orphan fixed by lead. 7 lanes notified.
2026-10-10  dap-csm DONE (0 stubs), 791 checks. Needs: TVCPITCH/TVCYAW
            accessors (state-flags), ids from src/csm/dap/pending.h into
            jobs.h + dispatch. NAMING FOLLOW-UP (integration/review):
            digit-suffixed label names remain in DAP code (agc_task_noroll1,
            agc_task_tvcinit1, csm_tvc_init4, ...) -- NAMING.md rule 2 says
            spell them out; defer to integration so cross-lane callers move
            together.
2026-10-10  guid-powered DONE (1 stub: DELRSPL). 13 checks. Moved sps_burn*,
            servicer*, ignition* into burns/{sps,servicer,ignition}/.
            Integration: ids duplicated between src/exec/pending.h and
            guidance pending with different values (CLOKTASK, TIG_5/0/30/35,
            IGNITION, ULLGTASK, ULLAGOFF, COMFAIL...) -> one value each.
            REVIEW NOTE: AGC ZOOM aliases P40ZOOMA (clears only restart group
            3), so COMFAIL2 KILLTASK / 3.3 restart never touch the throttle-up
            tasks; translated as flown -- reviewer to confirm.
2026-10-10  User flagged `(void)agc_waitlist_call(...)`. Audit: 777 void-casts
            (253 flag set/clear, 51 phase_change, 40 test_ovf-as-clear, 24
            waitlist_call, 29 alarm, ...). Rule 7 restated: no void-cast of
            agc_ calls; infallible APIs return void; fallible calls handled
            as the AGC does. Checker enforces (baseline 780 hard). New lane
            `status-handling` (Opus) runs after translation, before
            integration. Running lanes told to fix their own files.
2026-10-10  dap-lm DONE (0 stubs), 120 checks, rule-7 clean. Bug fixed:
            ROT-TOUV U/V swapped in qrjets.c. Integration: dispatch
            JET_ACCEL_UPDATE/RESET; T4RUPT calls agc_lm_dap_rcsmon() every
            480 ms; declare agc_erase_m11..m32 publicly; define hand-controller
            counter accessors. REVIEW: docked 1/ACCS CCS COEFCTR literal
            reading lands on TORQCONS (check vs page images / yaAGC); BIGAOS
            stores 2*DB vs comment 1*DB (code followed).
2026-10-10  Third usage-limit cutoff killed prog-rendz, prog-align, sys-misc,
            p37 mid-work. Checkpoint committed; relaunching all four.
2026-10-10  p37 DONE (0 stubs), 21 checks (coplanar min-fuel return converges
            to -6.500 deg, DV 3.6404 m/cs). Integration: 15 job ids
            0x0C30..0x0C3E into jobs.h/dispatch; prototypes to agc/csm/entry.h;
            accessors t3tot4, rted1, return_to_earth_range; prog-rendz must
            provide agc_rendz_ttogo_display_start (VN1645).
2026-10-10  prog-align DONE (0 stubs), 83 checks, rule-7 clean. OPEN for
            core/review: agc_vec_unit loses precision on short vectors at
            B27 (1 km LOS gives |u|=1.002); verify against INTERPRETER.agc
            UNIT (R63 flies at few-km ranges). Integration: ids in
            include/agc/programs/alignment_pending.h.
2026-10-10  sys-misc DONE (1 stub: LM P07 REDO drift filter jumps into an
            erasable-memory program not in the rope). 271 checks. Needs from
            prog-rendz: agc_rr_designate_gimbal, OPTIONX/DSPTEM1/RR-AZ/RR-ELEV
            accessors; RADSAMP should use R04/R77 accessors. Ids 0x0380-0x0397
            in src/io/verbs/pending.h. Note: library built without
            AGC_HOST_DEBUG so library asserts don't report in tests
            (integration: consider a debug lib target for tests).
2026-10-10  User: apply the NASA SEL-94-003 C Style Guide with an automated
            formatter (clang-format). Lead only skimmed the guide; user
            objected. Guide stored in docs/reference/; `style` lane (Opus)
            reads all 100 pages and derives docs/STYLE.md (every rule with
            its enforcement: clang-format / check_rules / review / n-a),
            .clang-format, tools/check_style.py, prolog templates, make
            targets. Tree-wide `make format` runs after prog-rendz finishes,
            before status-handling. clang-format needs install (sudo).
2026-10-10  prog-rendz DONE (0 stubs). TRANSLATION FIRST PASS COMPLETE:
            all 19 translation lanes done (core-numeric, exec-sched,
            state-flags, nav-conic, nav-integ, nav-aux, io-pinball, io-nouns,
            imu, guid-powered, guid-landing, guid-entry, dap-lm, dap-csm,
            prog-rendz, prog-align, sys-misc, p37, oracle). Integration
            items from prog-rendz: ids 0x1100-0x1217 / tasks 0x1101-0x111C;
            RADMODES must be one storage word (sys-misc has a copy);
            LAT/LONG/ALT, RNRAD, CDUTCMD, KILLTASK accessors; V57/V54/V89
            dispatch by vehicle; P23/R22LEM restart priority deviation;
            P24 not in either rope (remove from NAMING.md).
            Next: style lane output -> make format tree-wide -> status-
            handling -> integrate.
2026-10-10  Restructure pass 2 DONE: programs/rendezvous (39 files) ->
            programs/{navigation,targeting,displays}, csm/navigation,
            lm/navigation, lm/radar; csm/burns/return_to_earth/; tests/
            mirrors src/. All 241 .c files compile (-c). check_rules hard
            count 712 (all rule-7 void-casts; status-handling lane next).
            Known upward dependency to fix at integration:
            programs/navigation/noun_data.c includes lm/radar/radar.h.
2026-10-10  style DONE: docs/STYLE.md (173 rules w/ enforcement), .clang-format
            (69 options verified on 14.0.6), tools/check_style.py (30 rules;
            29,455 findings, mostly `return x;` without parens and
            uncommented declarations), templates, make format/format-check/
            style. Rule 4 now ignores brace-only lines (Allman). `make
            format` applied: 297 files, 0 compile failures, idempotent.
            check_rules: 713 hard (rule 7). Launching status-handling lane.
            Style follow-ups for integration/review: include/agc/core/assert.h
            shadows <assert.h> name (guide 3.4) -> consider rename;
            check_style findings to be worked down in review wave.
2026-10-10  User: paragraphing (blank lines between declarations, asserts,
            blocks, return) missing; clang-format cannot insert blank lines.
            Decision: replace clang-format with Uncrustify 0.72 (installed);
            `style2` lane ports .clang-format + guide 5.5 paragraphing to
            uncrustify.cfg on a scratch copy; tree-wide run after
            status-handling. Also found 3,233 `(void) AGC_ASSERT(...)`
            sites (rule 7 + rule 5 dodge, uppercase slipped past the
            checker): checker extended; status-handling lane scope widened.
2026-10-10  style2 DONE: uncrustify.cfg (164 options), .clang-format deleted,
            `make format` runs Uncrustify twice (align-then-wrap), token-
            identical preprocessed output on 241/241 files, idempotent by
            pass 3. Lead decisions: guard runs keep a blank between guards;
            >80-col trailing comments (920 lines) left for the review wave
            to shorten (check_style reports them); long-block check counts
            blank lines. Tree-wide `make format` runs after status-handling.
2026-10-10  status-handling DONE: 0 hard violations (was 713 + 3,233 after
            widening); 105 APIs made void; assertions given recovery by
            return type (tools/assert_recover.cocci, status_void.cocci);
            bugs fixed: agc_vec_zero/agc_vec_make mutual recursion, two
            GODSPRET calls with NULL continuation (V16N85 never shown).
            Open: CLEANDSP can sleep a job without continuation; core
            sqrt/asin/acos cannot raise 1301/1302 (callers must). Lead set
            mod_paren_on_return=add; reformatted (238 files). make lib OK.
            Launching integrate.
2026-10-10  Usage-limit cutoff killed integrate mid-work (was implementing
            the core alarm latch / exec service point). Checkpoint committed;
            relaunching with resume semantics.
2026-10-10  Collapse wave tooling: GCC -fcallgraph-info verified (per-TU VCG
            with static flags and call-site positions), cflow 1.7 and
            python3-clang-14 installed and verified. collapse.md rewritten:
            computed candidates, libclang extents, scripted inlining,
            guard de-duplication, review gate before applying.
2026-10-10  integrate DONE and lead-verified: make OK, make test (21 binaries,
            real debug lib) OK, make check OK, oracle 299+287 pass, 0 hard,
            format clean, 0 scaffolding files, 119,760 src lines. Registry:
            869 ids (tools/registry.tsv -> gen_registry.py). Added
            agc_power_on(). Deferred lane stubs: DELRSPL, P06, CSM prelaunch
            IMU test, LM systems test, V27, N08/N70/N71 orphan components.
            Launching collapse (phase 1, review gate), sim, oracle-cases in
            parallel with disjoint write scopes.
2026-10-10  Sim scope (user): minimal harness, and reuse Virtual AGC's
            peripherals instead of writing a DSKY/device model. build/agc_sim
            speaks yaAGC's peripheral socket protocol (port 19697) so
            yaDSKY2/yaTelemetry connect to our C as if to yaAGC; scenarios
            via tools/sim_client.py; our part is the clock/interrupt driver.
            virtualagc clone has yaDSKY2, yaTelemetry, yaACA, piPeripheral.
