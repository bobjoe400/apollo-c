# nav-aux -- lane history

Process diary of the nav-aux lane, moved verbatim out of [docs/modules/nav-aux.md](../../modules/nav-aux.md), which keeps the reader-facing explanation. Note dates: 2026-10-09 to 2026-10-14 (first and last commit touching the note).

## Progress

| Routine / table | Status |
|---|---|
| header agc_navaux.h | DONE include/agc/nav/auxiliary.h |
| util.c helpers | DONE src/nav/auxiliary/util.c |
| planetary.c (NEWANGLE EARTHMX EARTHL MOONMX R-TO-RP RP-TO-R, pad load) | DONE agc_planet_* src/nav/auxiliary/planetary.c |
| latlong.c (LAT-LONG LALOTORV GETERAD ARCTAN SETGAMMA SETRE) | DONE agc_latlong_* src/nav/auxiliary/latlong.c |
| free_fall.c (TFFCONIC TFFRP/RA CALCTFF CALCTPER T(X)) | DONE agc_tff_* src/nav/auxiliary/free_fall.c |
| measurement.c (INCORP1 NEWZCOMP INCORP2) | DONE agc_incorp_state_deviation agc_incorp_update_w agc_incorp_update_state src/nav/auxiliary/measurement.c |
| ephemerides.c (CM LSPOS LUNPOS LUNVEL SOLPOS, LM LSPOS, agc_lsposition) | DONE agc_ephem_* agc_lsposition src/nav/auxiliary/ephemerides.c |
| star_tables.c | DONE agc_star_vector src/nav/auxiliary/star_tables.c (37 stars, rope-verified) |
| tests/nav/auxiliary.c | DONE (compiles; links once core-numeric lands) |

All files compile with the project flags (-Werror); tools/check_rules.py:
0 hard violations, 0 warnings for the nav-aux files.  The whole lane plus
the test was also linked against a throwaway double-precision model of
agc_interp.h (scratch only, not committed): 143 checks, 0 failures.

## Requested core additions

* Shared erasable accessors for the pad load / uplinked ephemeris, now
  module-static behind agc_navaux_set_padload / agc_ephem_set_csm_data:
  agc_erase_azo(), agc_erase_axo(), agc_erase__ayo() (-AYO),
  agc_erase_504lm(), agc_erase_timemo(), agc_erase_vecoem(i),
  agc_erase_reso(), agc_erase_veso(), agc_erase_omegaes().
  (agc_erase_tephem() and agc_erase_rls() now exist in agc_state.h;
  agc_lsposition uses agc_erase_tephem.)
* agc_tp_shl / agc_tp_norm in agc_interp.h (core-numeric lists
  agc_tp_shift, agc_tp_norm in its design) would let util.c drop
  its raw-representation TP helpers; agc_vec_sq_tp would remove the INCORP1
  VSQ deviation.

## Requested jobs-tasks

None (INCORP2's PHASCHNG/GRP2PC restart protection and POSTJUMP INTWAKE
belong to the caller / nav-integ; see Deviations).

## Oracle cases (to request; tools/oracle cases/navaux_*.spec)

Interpretive routines: entry at the label with the interpreter set up
(LOC, BANKSET, push-down pointer), stop at the return (DANZIG / caller).
| case | rope | entry | inputs (scale) | outputs |
|---|---|---|---|---|
| geterad | both | GETERAD | ALPHAV+4 = 1/2 sin L (2^1) | ERADM:dp (2^29 m) |
| arctan | both | ARCTAN | SINTH 18D, COSTH 16D (2^1) | THETA MPAC:dp (rev) |
| latlong | both | LAT-LONG | ALPHAV (2^29 m), MPAC time (2^28 cs), TEPHEM, AZO, AXO, -AYO, flags LUNAFLAG ERADFLAG | LAT LONG ALT:dp |
| lalotorv | both | LALOTORV | LAT LONG ALT, time, pad | ALPHAV:dp x3 |
| r-to-rp / rp-to-r | both | R-TO-RP, RP-TO-R | MPAC vector, 6D time, X1 (0 earth / 2 moon), 504LM, TEPHEM | MPAC vector |
| newangle | CM | NEWANGLE | time, X0, XDOT, X1 | MPAC:dp |
| tffconic/tffrpra | CM | TFFCONIC | RONE VONE (2^29/2^7), MPAC 1/RTMU | TFFNP TFFALFA TFF1/ALF RPER RAPO:dp |
| calctff | CM | CALCTFF | after TFFCONIC; MPAC RTERM | MPAC TFF:dp, TFFQ1 TFFX TFFTEM |
| t(x) | CM | T(X) | MPAC X | MPAC:dp |
| incorp1 | CM | INCORP1 | W (162 DP), BVECTOR, DELTAQ, VARIANCE:tp, DMENFLG | ZI OMEGA DELTAX GAMMA NORMGAM |
| incorp2 | CM | INCORP2 FAZA1-FAZB2 | W ZI OMEGA GAMMA NORMGAM (rupts off) | W |
| lspos-cm | CM | LSPOS | MPAC time, TEPHEM, TIMEMO, VECOEM, RESO, VESO, OMEGAES | MPAC (moon), 2D (sun) |
| lspos-lm | LM | LSPOS | MPAC time, TEPHEM | VMOON VSUN:dp x3 |

## Open questions

* VGAMCALC and TFF/TRIG are in P61-P67.agc (CM entry), not in
  TIME_OF_FREE_FALL.agc; left to guid-entry.
* The pad load is duplicated (agc_navaux_padload_t vs agc_erase_tephem);
  resolve when core adds the accessors above.

