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
2026-10-10  collapse phase 1 reviewed: 4,140 functions, 25,201 edges; 1,004
            single-caller helpers, 176 candidates under the loose label rule.
            Lead decisions: keep fan_* splits; multi-site helpers out of
            scope; protection only for helpers whose header names an AGC
            label DISTINCT from the caller's (page-only inheritance does not
            protect). Phase 2 (apply) running.
2026-10-10  User paused the sim lane (GUI peripherals interfering with their
            desktop). Sim agent stopped mid-way through tools/sim_client.py.
            collapse phase 2 and oracle-cases continue (command-line only).
            Sim resumes headless-first on user go.
2026-10-10  collapse DONE: 153 helpers merged (150 by tools/inline_one.py),
            176 kept with reasons; entry_step_a/b re-split into five
            per-section dispatchers of REENTRY_CONTROL. make/test unchanged;
            2 hard violations remain in src/sim (paused lane). Behaviour
            change only on assertion-failure paths (caller's failure value).
2026-10-10  User flagged forwarding wrappers (static chan_write -> agc_chan_write
            with duplicated guards). Collapse pass 3: delete every function
            whose body is only guards + one forwarding call, regardless of
            caller count; move any genuinely new assertion into the callee
            once. Running.
2026-10-10  User resumed the sim lane. Relaunched with resume semantics;
            headless scenarios first, GUI verification last.
2026-10-10  User flagged duplicate identical guards in one function
            (set_lampword). check_rules now flags duplicate assertions as
            rule-5 hard violations (20 found); collapse pass 3 removes them
            along with forwarding wrappers.
2026-10-10  collapse pass 3 DONE: 113 forwarding wrappers found, 26 removed
            (81 kept are public API / job entries; review wave may judge
            per case). Lead's duplicate-guard rule was too broad and cost 4
            real postconditions; checker narrowed to back-to-back repeats
            and agent restoring them. Finding: ll_landjunk overwrote the
            lat/long status unchecked. 3,964 functions after collapse (from
            4,140).
2026-10-10  collapse lane COMPLETE: 7 postconditions restored; ll_landjunk
            asserts the LAT-LONG status (LANDJUNK p.791 has no failure exit).
            Oracle finding to carry into review: tests/oracle/dsky fails
            64/1544 checks on pre-collapse source too -> genuine DSKY
            conversion mismatch vs yaAGC (io-pinball review).
2026-10-10  oracle-cases DONE: nav-integ 247, nav-aux 484, imu 211, guidance
            203, landing 154, entry 521, dap-lm 473, dap-csm 246, targeting
            116, return_to_earth 144, dsky 1544 checks. Findings F1-F15
            (F7 CM J2 constants 1/4 of assembled; F8 S40.2,3 raw RTIG not
            UNIT; F9 sign-disagreeing DAS high word used alone; F13 CDH
            DIFFALT 62 m; F14 RTE INVC115 convergence; F15 OPDEGOUT -0
            treated as CCSHOLE). Oracle tool defects: channel 7 writes,
            symbol parsing. Launching review wave: 16 per-lane agents.
2026-10-10  REVIEW WAVE launched: 16 per-lane review agents (core-numeric,
            exec-sched, state-flags, nav-conic, nav-integ, nav-aux, io-dsky,
            imu, guid-powered, guid-landing, guid-entry(+p37), dap-lm,
            dap-csm, prog-rendz, prog-align, sys-misc), each with its oracle
            findings and open questions routed. sim lane running in parallel
            (src/sim only). Document wave after review.
2026-10-10  Usage-limit cutoff killed all 16 review agents + sim. Checkpoint
            committed. Lane-specific review items now in
            docs/agents/review_items.md. Relaunching all 17 with resume.
2026-10-10  review-guid-powered survived the cutoff; the relaunched duplicate
            was stopped (it had F7/F8/F9-TDISPSET/DELRSPL done already).
            Rule: ListAgents before relaunch.
2026-10-10  review-state-flags DONE. BUG: DELVSLV and DELVLVC are the same
            six erasable words in both ropes; C kept two copies so S30.1 and
            downlink used a zero delta-V -> unified. All 148 CSM / 174 LM
            flags verified against rope symbol tables. POST-WAVE TASK (lead):
            rename include/agc/core/assert.h -> agc_assert.h (237 includes;
            commands in review-state-flags.md) once no agent is editing.
2026-10-10  Session limit approaching: tree committed; all 16 running agents
            (15 reviews + sim) told to bring their Progress sections current
            and leave touched files compiling before continuing. On cutoff:
            commit, ListAgents, relaunch only the dead ones with resume.
2026-10-11  Cutoff killed all 16 remaining agents again. User: cap at 2-3
            concurrent lanes. Policy recorded in PLAN.md with a queue order.
            Relaunching 3: review-io-dsky, review-guid-entry, review-nav-aux.
2026-10-11  review-nav-aux DONE (143 + 484 checks; 0 style findings). Tree
            build broken by review-imu's half-added gyrocompassing ids in
            src/csm/prelaunch_align.c -> review-imu moved to the front of
            the queue. Requests to core-numeric: TP-input ASIN/ACOS,
            word-level MPAC (residual 1-4 LSB diffs).
2026-10-11  review-io-dsky DONE: dsky oracle 1627 bit-exact (64 KNOWN fixed);
            sim bug B1 (QUIKDSP skipped when DSKYFLAG off) fixed; 4 ordering
            bugs from new assertions. Cross-lane: `agc_chan_and(~BIT)` passed
            bit 16 and tripped the mask assert at 5 CSM sites -> lead made
            agc_chan_and mask to 15 bits (WAND semantics). Open: loadable
            noun components without write path (list in review-io-dsky.md).
            Launching review-prog-rendz (queue #4).
2026-10-11  review-guid-entry DONE. FINDING E1: 92 DMPR-for-DMP sites in
            entry/P11; residuals that had been attributed to yaAGC vanished
            (ROLLC 2159 LSB -> <=6). Tree-wide DMP/DMPR + shift-rounding
            audit added to review_items.md for all remaining lanes. P11
            fixes (ATERJOB, BIASAK TP add, RESCALES). Requests: exec rename
            AGC_JOB_REFAZE10; guid-powered export AUGEKUGL; POLYNUM+1 on
            P11 pad load. Launching review-guid-landing (#5).
2026-10-11  review-prog-rendz DONE: 1 DMP site; F13 is NOT a multiply error:
            R1A word-form effect in GETX WLOOP and GETECC (circular orbit:
            AGC ECC 0, C 23164 -> apsides 9 km apart) -> nav-conic must carry
            R1A as the AGC word pair; core-numeric: SQRT keeping MPAC+2;
            exec: agc_phase_set_prdt. Launching review-nav-conic (#8 moved
            up: it now owns P1/P6 and the -0 modelling).
2026-10-11  review-imu DONE: build restored (gen_registry re-run); 12 of 13
            stubs translated (P06, POSTAND, P01/P02/ESTIMS/V78/IMU test, P03
            in prelaunch_verify.c); only the erasable-memory drift part
            remains stubbed. 36 DMP sites; F5/F6 fixed; bugs: KALCMANU TM
            storage, PIPFREE inhibit leak, LASTBIAS PIPUSE1->PIPUSE, NBDONLY.
            Recurring request now from 4 lanes: core word-level MPAC API
            (ones'-complement word pairs) -> review-core-numeric must do it.
            Launching review-dap-lm (#6).
2026-10-11  User paused review-nav-conic and review-dap-lm (checkpoint-and-
            stop requested; resume from their notes later). review-guid-
            landing continues alone.
2026-10-11  review-nav-conic paused clean (no code changes): R1A/COGA/X/KEPC2
            are sign-disagreeing word pairs; plan in note. Dependency: core
            word-level API should exist first -> next session order:
            review-core-numeric, then nav-conic, then dap-lm. guid-landing
            still running.
2026-10-11  review-guid-landing DONE: 0 DMP errors but 15 TP-accumulator
            sites (DOT/DMP then SL, DSQ then SQRT) fixed; RODCOMP exact
            (148 LSB /AFC/ error gone); ascent exact through TGO, residuals
            after are word-pair storage (core R1 request). Bug: RDOT noun
            points at unwritten rdot; should be HDOTDISP (exec/nouns.c:155)
            -> review-exec-sched item. State: make/test green; make check
            fails only in paused dap-lm (rule 4 accont.c) and dap-csm (rule 7
            tvc/daps.c). NO AGENTS RUNNING. Next session: review-core-numeric
            (word-level API), then nav-conic, dap-lm, dap-csm, nav-integ,
            exec-sched, prog-align, sys-misc, guid-powered, sim, DMP follow-
            up, assert.h rename, document.
2026-10-11  User: formatted code unreadable -- comments inside expressions
            (`x = /* scale */ call(...)`, `/* UP */` inside arg lists) and
            paren-aligned continuation staircases. Standard updated; lane
            format2 (Opus, alone): tools/fix_comments.py (libclang, moves
            in-expression comments above the statement), uncrustify.cfg
            fixed 8-space continuation, tree-wide run, long-statement
            inventory for the review wave. All other work waits.
2026-10-11  format2 DONE: tools/format.sh = fix_comments.py + uncrustify x2 +
            fix_comments.py; code_width 0; outer-call wraps kept (50),
            ~620 in-expression comments merged into trailing comments, 1253
            paragraph blank lines; paragraph-scoped column alignment done by
            the script (Uncrustify 0.72 aligns across blank lines). Token-
            identical on 270/270. docs/overview/long_statements.tsv: 777
            statements >120 cols for the review wave to split with
            temporaries. Launching dispatch (4 flat switches) + review-core-
            numeric.
2026-10-11  User: 2 concurrent lanes max from now on. review-nav-integ asked
            to checkpoint and stop; dispatch (Sonnet, quick) and review-core-
            numeric continue.
2026-10-11  dispatch DONE: 4 flat switches, 89 chunk functions gone,
            dispatch.c 5200 -> 2800 lines. 50 hard rule-7 violations remain,
            reintroduced by cut-off review runs in dap-csm (tvc), prog-align
            (word_forms.c), csm/optics/geometry.c, dap-lm (accont.c): each
            lane's resumed review clears them. Launching review-dap-lm.
2026-10-11  review-dap-lm DONE: 30 sat->wrap stores (R11), oracle 1780/0 at
            tolerance 0, style 714->0, 18 unused accessors removed. Routed:
            R12 exec restart must call agc_lm_dap_restart(); sim must count
            TIME6. Launching format3 (brace initializers).
2026-10-11  review-core-numeric DONE: word-level MPAC API; 2021 oracle checks
            0 failures; reproduces WLOOP/ECC=0. Guide section 1 updated;
            adoption items routed. Resuming review-nav-conic.
2026-10-11  format3 DONE (brace initializers). Core files formatted by lead
            after the core review finished. Launching review-dap-csm (#7).
2026-10-11  User found `(void) csm_dap_words_das(...)` (31 sites, dap-csm's
            new word-level helper). check_rules now flags ANY (void)-cast
            call outside a stdlib allowlist (and those need a comment).
            58 hard now: dap-csm (running) + prog-align word_forms.c +
            csm/optics/geometry.c (queued lanes).
2026-10-11  User: making functions void to dodge the cast left silent
            failures (`if (!AGC_ASSERT) return;` in 499 void functions, 805
            guards) and sentinel returns (945 value functions, 1,605 guards).
            Rule 7 "No silent failures" added; checker flags both (now ~2,460
            hard). Lane status-handling-2 queued to run alone after the two
            current lanes: delete guaranteed guards, convert fallible ones to
            agc_status_t + out-param with callers checking.
2026-10-11  review-nav-conic DONE: whole lane word-level; oracle exact at
            tolerance 0 (524 checks, 55 cases); CDH DIFFALT exact. Decision:
            rule-5 density is a PROJECT average (hard), per-file is a
            warning; no assertions that cannot fail. prog-rendz follow-up:
            tighten targeting tolerances. Waiting for dap-csm, then
            status-handling-2 alone.
2026-10-11  User: decide guard redundancy by dataflow (value checked at entry
            and unchanged along the path needs no re-check). status-handling-2
            now classifies with Frama-C EVA (frama-c-base, needs install),
            libclang fallback; adds `make prove`. Rule-5 project average is
            1.51 after real-guard removal; target 2.0 reconsidered after the
            pass (no padding assertions).
2026-10-11  frama-c-base installed (user). status-handling-2 launches alone
            when review-dap-csm reports.
2026-10-11  review-dap-csm DONE: QUARTER 8192->4096 (rope BIT13), T6START
            scan fix, word pairs in TVC/RCS, oracle 533/0 at tol 0, 63 void
            casts + 106 guaranteed guards removed. Lane density 0.43.
            Launching status-handling-2 ALONE.
2026-10-11  User: we followed the letter of rule 5, not its spirit. Rule 5
            rewritten as where-assertions-belong; density is now report-only
            (no hard/warn thresholds); brief and review template updated.
            status-handling-2 told not to add assertions to chase 2.0.
2026-10-11  User: js_acy is a single-caller function kept only because of
            its AGC labels. Rope check: ACYCHECK/BDZCHECK 0 references,
            TAUCHCK 1, NOACY 3. Refined rule: a label protects a helper only
            if entered from >= 2 places in the rope. Added as step 7 of
            status-handling-2.
2026-10-11  EVA runs were whole-program (3 x 3.8 GB, 40-min timeouts, 11/15 GB
            used). Lead killed frama-c processes; agent told: EVA is a
            classifier not a gate -- per-TU, low precision, 120 s/run,
            <= 20 min total, libclang dataflow for the rest.
2026-10-11  User: heavy analysis is fine, opacity is not. Revised: EVA runs
            thorough, max 2 concurrent (RAM), detached, writing
            tools/prove/progress.log + summary.txt per run; agent proceeds
            with dataflow-classified work meanwhile and folds in EVA
            verdicts at the end.
2026-10-11  User: why trust EVA before all lanes are reviewed? Adjustment:
            structural deletions everywhere; EVA-only deletions only in
            reviewed lanes; `eva-valid-deferred` class for the 5 unreviewed
            lanes, re-applied via `make prove-classify` after each review.
            Review template updated.
2026-10-11  User: collapse before proving. Re-sequenced status-handling-2:
            structural + collapse + returns first, then one EVA run on the
            finished tree. Lead killed the pre-edit-snapshot EVA runs.
2026-10-11  User's end goal stated: run the Virtual AGC simulator GUI on our
            C backend (agc_sim as a drop-in for yaAGC). PLAN item 7 and the
            sim assignment rewritten around that; sim lane promoted to run
            alongside the remaining reviews after the alone pass.
2026-10-11  User principle: build only the AGC software; everything else is
            an existing proven tool (yaAGC, VirtualAGC GUI/peripherals,
            NASSP). Own world model dropped; NASSP is the only closed-loop
            route (after item 7). Sim lane: no device models.
2026-10-11  Cutoff killed status-handling-2 mid-pass (classifier done: 1,309 of
            2,285 guards GUARANTEED; deletion partly applied, hard count
            2,317 -> 1,015; make lib OK). Relaunched with resume.
2026-10-11  status-handling-2 ran its end-of-pass EVA (34 entries OK, sim
            TIMEOUT at 60 min: the unbounded clock loop with unknown argv
            has no cheap fixpoint). User: EVA is cleanup, not a
            working-code step -- it belongs after all reviews and the
            GUI-verified sim. Re-sequenced: no further EVA runs until the
            tree is frozen; the run's alarms (two INVALID verdicts, the
            uninitialised-struct reads in programs/targeting, two signed
            overflows) are recorded as bug leads per lane in
            review_items.md. Lead also stopped three duplicate waiters the
            agent had stacked on the one EVA process (rule for agents:
            one waiter per event).
2026-10-11  status-handling-2 finished and committed (a02850c): rule 7 at 0
            hard, 216 helpers inlined, 162 returns, tree green (34/34
            tests, 7 scenarios). Launched sim (resume, Opus; may fix
            scenario-blocking flight bugs B2/B4 with reference + test)
            and verify-collapse (Opus; independent check of the
            1b72892..a02850c diff: side-effect/mixed deletions, deletions
            in code no test reaches, manual.tsv sample, status chains,
            inlinings, returns, void conversions). Two-lane cap holds.
            review-exec-sched follows whichever finishes first.
2026-10-11  User: sim lane goes to the end, after all reviews (remaining
            work is acceptance: drop-in command line, GUI runs; doing it
            while reviews change flight code means doing it twice). Sim
            agent stopped at a checkpoint; review-exec-sched takes its
            slot next to verify-collapse.
2026-10-11  User: avoid two agents on the same code / verifying before it
            settles. verify-collapse scoped to the eleven already-reviewed
            lanes (nobody reads them again); each remaining review checks
            the status-handling-2 diff in its own files (review.md item).
            exec-sched starts when the sim agent has stopped.
2026-10-11  sim agent stopped at a clean checkpoint (7/7 scenarios on the
            current tree; B2 and B3 closed by the guard pass, B4 WAIT2
            still stands -> routed to exec-sched). Launched
            review-exec-sched (resume, Opus) alongside verify-collapse.
2026-10-11  verify-collapse done (Opus): 11 reviewed lanes, 1b72892..a02850c.
            41 side-effect/mixed deletions, 1,026 deletions in unreached
            code, 249 manual.tsv rows, 10 status chains, 52 inlinings, 49
            return rewrites, 155 void/dropped-param functions checked:
            agree on all, no src edits. Minor findings and residual doubt
            (BAD_ARG replacing callee status at 62 sites; two tests still
            count assertion reports; boilerplate manual.tsv reasons)
            routed in review_items.md.
2026-10-11  Launched review-prog-align (resume, Opus) in verify-collapse's
            slot, alongside review-exec-sched.
2026-10-11  review-exec-sched done and committed (155178d): WAIT2 both
            branches (B4), LM restart calls the DAP restart (R12), RDOT
            noun, JOBWAKE(NONE) NOT_FOUND, GRP2PC restart points
            (DEVIATION: segment rerun from top), BAD_ARG filters repaired
            (ENEMA/GOPOODOO under INHINT, GOPROG3 BAILOUT), REFAZE10
            renamed, style 1917 -> 0. Requests routed in review_items.md.
            Launched review-sys-misc (resume, Opus) alongside
            review-prog-align.
2026-10-11  User: usage limit approaching, pause one lane. review-sys-misc
            (the newer of the two) told to checkpoint and stop;
            review-prog-align keeps running. One lane running.
2026-10-11  review-sys-misc checkpoint committed: self_check.c to the rope
            (ROPECHK one bank per idle pass, SCOUNT, the three error
            paths; both ropes' bank sums pass; the sim now runs it when
            idle), N08 ALMCADR closed, restart resets wired (DNTMGOTO,
            LM RADMODES, AGSWORD downlist, DUMPCNT on V36 only). Stopped
            in down_telemetry.c. Requests routed in review_items.md.
            prog-align's in-progress files left uncommitted.
2026-10-11  User: pause the remaining review too (session limits).
            review-prog-align told to checkpoint and stop. Nothing
            running after its report; resume order in STATE.md.
2026-10-11  review-prog-align checkpoint committed: six DMP/DOT-shift sites
            fixed against the listing (OANB, SURFSTAR, R59 x3, SR52, P57
            gravity error), MSU field-rotation bug in aotmark.c, SXTNB and
            LM S-band pitch exact at 0 via agc_mpac_*, word_forms.c on the
            word-level API, 50 oracle cases added (oracle_align 600
            checks, 1 KNOWN PA-W4), 62 BAD_ARG filters now propagate.
            Paused before sxtmark/sighting_mark guard review, SXTMARK
            compare, P51-P53/P57 pass, style. Nothing running.
2026-10-11  User: resume. Relaunched review-prog-align and review-sys-misc
            (Opus) from their checkpoints; each told to format only its
            own files. Two lanes running.
2026-10-11  review-prog-align DONE and committed: ~70 guaranteed guards
            deleted, stall-outcome callers propagate, PICAPAR loads the
            catalogue instead of a zero vector, SXTMARK/P51-P53/P57/R59
            confirmed with three bugs fixed (LM P51 THETAD never zeroed;
            MFREF/REFMF/ADVTRACK body chosen from LUNAFLAG instead of the
            time in the accumulator; P52 zero-time test on the high word),
            prelaunch header blocks -> section comments, style 2,275 -> 0.
            Requests routed. review-sys-misc still running.
2026-10-11  Launched review-guid-powered (resume, Opus) in prog-align's
            slot, alongside review-sys-misc.
2026-10-12  review-sys-misc DONE and committed: S10-S18 fixed with tests
            (LM R03 empty-mass constants 4x off, V73 clock increment never
            applied, V74 dump sent zeros, V71/V72 address masking, V43
            ones' complement, two dropped WAITLIST/INTSTALL errors, one
            DMP as DMPR); LM P07 translated from a stub (416 words
            yaAGC-exact; scenario p07_systems_test_lm); never-fail
            assertions and ctx removed, no BAD_ARG filters left, style 0,
            test_sysmisc checks statuses. Requests routed. Seven files
            carrying review-guid-powered's in-progress *NBSM* void
            change left uncommitted for that lane's commit.
2026-10-12  Launched review-nav-integ (resume, Opus) in sys-misc's slot,
            alongside review-guid-powered. These are the last two reviews.
2026-10-12  review-guid-powered DONE and committed: *NBSM*/AX*SR*T word
            forms (align KNOWN PA-W4 closed, oracle_align 570/0 at 0),
            S40.2,3 bell axes and FLESHPOT on words (s40_2 tol 16 -> 2),
            10 AGC-labelled helpers restored from inlining, NULLCLOK
            RELINT bug fixed, ~170 dead guards removed, 78 wrong prolog
            page citations corrected, style 1,276 -> 0. Requests routed.
            Three stray grep-output files (hichp, p, qp) in the repo root
            deleted. review-nav-integ still running.
2026-10-12  Launched follow-up-done-lanes (Opus) in guid-powered's slot,
            alongside review-nav-integ: all routed items owned by closed
            lanes (status propagation, EVA leads, API void conversions,
            DMP/word-level adoption in the eight early-reviewed lanes,
            test hygiene). Keeps out of src/nav/integration.
2026-10-12  review-nav-integ DONE and committed (its files only; the tree is
            mid-edit by follow-up-done-lanes): 9 DMP-as-DMPR sites (COMTERM,
            QUALITY2/3, TIMESTEP DOT) and NORFINAL double rounding fixed;
            GAMCOMP/COMTERM/QUALITY/OBLATE tail at word level (exact on
            3,600 probe cases); 45 oracle tolerances tightened; callee
            statuses carried through ctx->fail, STATEUP/SETCOAST/INT/W no
            longer ignore POODOO; GRP2PC rerun: INTSTALL takes the area
            over (DEVIATION, test); style 859 -> 0. All sixteen reviews
            are now closed. Requests routed to follow-up-done-lanes.
2026-10-12  Launched sim (acceptance pass, Opus) in nav-integ's slot,
            alongside follow-up-done-lanes. Sim records flight bugs only.
2026-10-12  Usage limit cut off follow-up-done-lanes and sim. Checkpoint
            494eb9a: tree builds, make test 34/34, 13 scenarios, check_rules
            0 hard. follow-up had done 1.1-5.5 except 3.2 (entry HUNTEST1,
            landing TBUP/TGO, nav-aux NODI/ASIN word-level); sim had done
            steps 1-4 (rope image, scalers, scenario e, launcher
            drop-in). Both relaunched from their Progress sections.
2026-10-12  follow-up-done-lanes DONE and committed (9b0936b): word-level
            adoption in nav-aux/landing/entry fixed 1-38 LSB errors in
            LAT, LALOTORV, TFF/TFFX, VSUN, AT, TTOGO, PRATE/YRATE, UNFC/2,
            VL, A1, RDOT; oracle tolerances to 0 across nav_aux, landing
            ascent, entry; AOTMARK flow confirmed. Open residual: entry
            DOWNCNTL on values (F3). Every routed review item is closed.
            Only sim (GUI step) running; then lead assert.h rename, ONE
            EVA run, document wave.
2026-10-12  sim acceptance DONE and committed (625fc5f): stock VirtualAGC
            launcher + yaDSKY2 + yaTelemetry on build/agc_sim, both
            vehicles, fresh start/V35/V16N36/P52/P30+P40; 13 headless
            scenarios; LM P63 skipped (closed loop needs a vehicle model:
            NASSP, stretch). Open flight bug B6. Lead: assert.h ->
            assertion.h (guide 3.4), make test now runs the scenarios.
            Launched fix-b6 and review-sim (Opus).
2026-10-12  User's editor showed "too few arguments" on agc_telem_fresh_start:
            the C/C++ extension had resolved the header from the stale EVA
            snapshot build/prove/tree. Snapshot deleted; `make
            compile_commands` (tools/compile_commands.py) + .vscode config
            (compileCommands, build/ excluded) added so editors use
            include/ with the Makefile's flags.
2026-10-12  User found a helper prolog pasted inside lm_dap_needler and
            then SN1's ARGUMENTS/RETURNS box inside a body: 135 such
            boxes (inline_one.py artefact; 77 had been routed to the
            document lane) rewritten as block comments by
            tools/unpaste_prologs.py; check_rules now rejects an indented
            box; rule in CODING_STANDARD. The 22 vehicle assertions at the
            top of LM-only routines (src/lm/dap, src/lm/landing) deleted
            by tools/cocci/vehicle_guard.cocci (guaranteed by construction:
            LM dispatch only); exec_up_servidle's stays (reached from the
            shared POODOO path, caller checks the status).
2026-10-12  review-sim DONE (three real clock faults: T6 chain, T5/T3 order,
            trap bits; protocol fixes; style 0; tests/sim/harness.c) and
            fix-b6 DONE (CLOCPLAY two-part display; scenario e runs the
            whole burn, 0 xfail; LM had the same defect). Committed. No
            open flight bugs. Tree frozen: next the single EVA run.
2026-10-12  Started the single EVA run on the frozen tree (jobs 1, no
            effective timeout, sim entry unbounded as the real main).
            Launched the document wave (Opus) alongside: docs only.
2026-10-12  User asked whether the loose ends were cleaned up: they were
            only recorded. Launched loose-ends (Opus) for the five open
            items alongside document; EVA runs on its snapshot so small
            code edits are safe (apply matches guards by condition).
2026-10-12  User: EVA and docs run only on a 100% working, final codebase.
            Lead had started both with five loose ends open and then
            launched a code lane under them: EVA killed, document agent
            stopped at a checkpoint (its output kept for re-check on
            resume). Order fixed: loose-ends -> frozen tree -> EVA ->
            apply -> document wave.
2026-10-12  User: the document wave's read found inconsistencies, so that
            check belongs before EVA. Its exec_cp_cleanup_lm "empty body"
            was a misread (body intact); the two comment findings stand.
            Lead wrote tools/prolog_check.py (libclang: RETURNS vs actual
            returns with propagation, ARGUMENTS vs parameters, flags and
            calls named in a prolog vs the body, empty bodies): 118
            findings (lm/dap 46, csm/burns 14, nav/aux 11, ...), `make
            prolog-report`. Launched prolog-consistency (Opus) on them,
            excluding loose-ends' directories; those follow after it.
2026-10-12  User: functions that always return OK must be void (rule 7);
            EVA does not report that. Lead wrote tools/always_ok.py
            (`make always-ok-report`): 94 functions (77 public, 24 job/task
            entries). User: a lane does the sweep, not the lead -> queued
            always-ok for the next free slot; added to the freeze gate.
2026-10-12  prolog-consistency DONE and committed (95ddc59): 90 prologs
            rewritten, 0 body fixes, checker tightened. Side findings
            routed (LM GET+MGA double unitize -> loose-ends; leadin.c
            silent skip and returns-extra -> always-ok). Launched
            always-ok (Opus) alongside loose-ends.
2026-10-12  loose-ends DONE and committed: entry L/D and L/D1 at 0 (cause was
            V1's stored words + GETETA on values, not D); waitlist 16250 cs
            bound kept as a documented DEVIATION; pad sps_impulse + a second
            burn scenario (engine on 0.75 s, residuals 0); 64-jobs bound
            justified (CHECKG1 PIPA poll; AGC < 17 jobs/cs); LM GET+MGA
            unitizes twice as Luminary does; prolog-report 0 tree-wide.
            always-ok now has the whole tree.
2026-10-12  always-ok DONE and committed (3a0f68d): 136 always-OK functions
            void, ~250 call sites, 34 dispatch arms, three dropped
            statuses exposed; prolog-report and always-ok-report 0
            tree-wide. 21 positive-assertion setter guards: classifier
            blocked the lane's deletion; user: leave them for EVA. `make
            check` fails on exactly those 21 (silent-skip rule) until the
            apply step; the bare check_rules.py invocation scans a
            different set (recorded). Tree FROZEN; EVA started
            (jobs 1, no effective timeout, sim entry unbounded).
2026-10-12  Lead note: `python3 tools/check_rules.py` with no paths scans
            nothing (prints 0 hard, density 0/1); the real gate is `make
            check-rules` (src include). The lead's bare invocations in
            this session were vacuous; every lane ran `make check`, and
            `make check-rules` now reports exactly the 21 silent-skip
            sites left for EVA and nothing else.
2026-10-12  EVA pass: 36 entries in 47 min, 33 OK, 3 crashed (sim, io_dsky,
            guidance_guidance) with a Frama-C 25.0-beta internal assertion
            at locsam.c:545 (the picapar_search_t initializer; reproduced
            single-file with -main agc_align_picapar; `= {0}` crashes too,
            plain declaration + assignments does not). Workaround applied
            (one statement, behaviour unchanged); the three entries rerun.
            INVALID 2 at the two known fallible guards (SPECTEST CCSHOLE,
            start_job with no stored point) reached by tests on purpose.
2026-10-12  tools/prove/eta.py (progress meter: coverage vs reachable set,
            position, loop heads, memory) showed the unbounded sim entry
            swapping at 14.4 GB RSS, 78 % coverage, outer loop not yet
            re-entered: killed. User, on whether to build EVA harnesses
            like coverage: stop here, finish the project. EVA rerun
            stopped; verdicts merged from the 33 completed entries
            (VALID 100, INVALID 2, UNKNOWN 146, DEAD 644). Launched
            finish (Opus) to apply them and close the gates.
2026-10-12  User: final test = the whole Apollo 11 mission (CSM + LM) on our
            backend via VirtualAGC and NASSP, all from this Linux/WSL
            host. Virtual AGC alone has no spacecraft; its LM Simulator
            (Tcl/Tk) is the one closed-loop tool: tcl/tk installed by the
            user, lm-simulator lane launched (10a). NASSP is Windows-only
            and embeds yaAGC's engine; route is WSL interop to the host
            (10b study first, 10c build/run if feasible). PLAN item 10
            rewritten; assignments written.
2026-10-12  Host (Windows, via WSL interop/mnt): user installed Orbiter 2016 at
            C:\Orbiter2016; lead downloaded NASSP V8.0 Beta Orbiter2016-2651
            (287 MB, GitHub release) to C:\Users\cnama\Downloads\nassp and
            unpacked it over C:\Orbiter2016 (1,601 files: Modules/
            ProjectApollo, Scenarios/Project Apollo - NASSP, Doc). Missing:
            the D3D9 graphics client (forum download, manual). NASSP's
            default branch is Orbiter2016 (so 2016, not 2024).
2026-10-12  NASSP beta 2651's Saturn5.dll imports __writeLogError from
            Orbiter.exe: an open-source Orbiter export, absent from the
            stock 2016 binary (user had asked "why 2016 instead of 2024":
            2024 was right). Installed Orbiter 2024 x86 (orbitersim/orbiter
            release "2024", 373 MB) as C:\Orbiter2024, NASSP unpacked over
            it (3,912 files), Orbiter.exe exports the symbol, D3D9Client
            bundled and activated in Orbiter_NG.cfg. C:\Orbiter2016 left
            as is.
2026-10-12  finish DONE and committed (d53337b): 100 EVA-VALID guards
            deleted (manual.tsv rows), 20 setter guards fallible, 51 more
            void, check_rules bare invocation fixed, sim entry dropped
            from eva_run. CODE FINAL. Document wave relaunched on it.
            Host: NASSP beta crashes in Saturn5.dll on Orbiter 2024
            release after scene init (first frame); Orbiter nightly x86
            (single exe, Orbiter.cfg ACTIVE_MODULES block) installed as
            C:\Orbiter-nightly with NASSP, user to try.
2026-10-12  Forum/guide (user: "couldn't this have been solved in the
            forums"): NASSP 8 betas target Orbiter 2016 Beta R90, not the
            2016 release nor 2024; NASSP 9 (rcflyinghokie/NASSP, CMake,
            x64, Orbiter 2024) is the 2024 line. Installed C:\NASSP9 =
            Orbiter 2024 x64 + NASSP-V9.0-Orbiter2024-437 (D3D9Client +
            XRSound active, D3DCompiler_47 x64, Modules/Server junctions).
            Integration target switched to NASSP 9.
2026-10-12  User: stock NASSP 9 (C:\NASSP9, Orbiter 2024 x64 +
            NASSP-V9.0-Orbiter2024-437) loads "Apollo 11 - Launch". This
            is the full-mission baseline for PLAN 10c. nassp-study
            launches in the next free slot (lm-simulator and document
            hold both).
2026-10-12  User: host toolchain installed (VS 2022 Build Tools 17.14 /
            MSVC 14.44, MSBuild, CMake, Git; verified via interop).
            document paused at a checkpoint (a, b, d done: overview,
            10 flowcharts, xref 3,067 labels) to free the slot; launched
            nassp-integration (Opus) alongside lm-simulator. Document
            wave's five code/comment findings filed in review_items.md.
2026-10-12  lm-simulator DONE (b0a29df): IMU loops close as yaAGC; L1-L3
            flight bugs, 00211 alarm, PROG-after-POODOO, S-L1/S-L2 harness
            findings; make lm-sim. That commit also swept in
            nassp-integration's in-progress src/core/ecadr.c (address map,
            112 checks) and src/sim changes (lead's mistake; harmless).
            src/nassp (host-only shim) excluded from the Linux build.
            Launched closed-loop-fixes (Opus) alongside nassp-integration.
2026-10-12  NASSP milestone 1 PASS (Insertion + first MCC CSM state-vector
            uplink, our AGC in NASSP 9 vs stock, identical unattended Lua
            script, 20x warp cap): V82 after load 104.1/102.0 vs
            104.3/101.9 (Linux replay exact), after uplink 109.7/107.3
            vs 110.2/107.0 (MCC per-session vectors), N43 identical, P27
            sequence identical. Findings on the way, all in the scenario-
            restore address map (src/core/ecadr.c), none in flight code:
            full 42-word state-vector blocks (integration exited from
            RCV=0), LAT/LONG/ALT, LAT(SPL)/LNG(SPL) (R30 DELRSPL; HA/HP
            5 nmi high), mixed-sign words kept for unchanged elements.
            Integration bug #1 was cost: full mirror export per cs ->
            per second (6 ms CPU per sim second, like yaAGC); watchdog
            sidecar logs. NASSP's yaAGC now also builds on Linux for
            offline image replays. Next: TLI (scenario 03) + pad-load map.
2026-10-12  NASSP milestone 2 PASS (TLI, scenario 03, unattended): V82 at
            load 111.1/104.0 vs stock 111.2/104.0 (replay exact), P47
            through the S-IVB burn on both, post-TLI vector consistent to
            the 0.11 s sampling offset. Pad-load map extended (IMU comp,
            pad orientation, CSM ephemeris, SPS/TVC pads, entry pads,
            lunar potential), tests/core/ecadr.c 125 checks. Milestone 3
            (MCC-2 P30/P40 SPS) via NASSP's own Apollo 11 checklist
            workbook compiled to Lua tables, executed identically on both.
