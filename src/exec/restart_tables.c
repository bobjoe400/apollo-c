/**********************************************************************
 * FILE NAME: restart_tables.c
 *
 * PURPOSE:
 *   The RESTART TABLES of both vehicles as static const data, and the
 *   reader for the erasables they name.
 *
 * AGC SOURCE:
 *   Comanche055 RESTART_TABLES.agc pp. 211-219
 *   Luminary099 RESTART_TABLES.agc pp. 238-243
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TGO           agc_dp_t  I    cs             agc_erase_tgo()
 *   SAVET-30      agc_dp_t  I    cs             LM agc_erase_savet_30()
 *   P40TMP        agc_dp_t  I    cs             CSM agc_erase_p40tmp()
 *   ZOOMTIME      int16_t   I    cs             LM pad load
 *   S61DT, CM/GYMDT int16_t I    cs             CSM entry delta times
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   NOT_FOUND for a phase past the end of a table; BAD_ARG for an
 *   erasable the reader does not hold.
 *
 * NOTES:
 *   Every restart group has an even table (two 3-word entries per even
 *   phase, both are restarted) and an odd table (one 3-word entry per odd
 *   phase >= 3). Phase numbers are octal: 4.11SPOT is phase 011 = 9. An
 *   entry is a job (PRDTTAB = priority, + FINDVAC / - NOVAC, then the job's
 *   2CADR), a task (2CADR stored negatively; PRDTTAB = delta time in cs,
 *   -0 for "immediately", or -GENADR of a word holding the delta time) or a
 *   LONGCALL (GENADR negative, BBCON positive; PRDTTAB = GENADR of the DP
 *   delta time). The AGC tells them apart by signs; here by `kind` and
 *   `dtsrc`. Priorities are written as priority numbers (PRIOnn >> 9);
 *   OCT 52777 in Luminary is -OCT 25000, a NOVAC of priority 25.
 *
 *   Aliased tables are kept as aliases: Comanche 1.2SPOT = 2.2SPOT =
 *   3.2SPOT = 4.2SPOT, Luminary 2.2SPOT = 3.2SPOT = 6.2SPOT = 1.2SPOT.
 *   The AGC's "DEC 0" placeholder entries are kind NONE.
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define RST_J          AGC_RESTART_JOB      /* job entry */
#define RST_T          AGC_RESTART_TASK     /* task entry */
#define RST_L          AGC_RESTART_LONGCALL /* LONGCALL entry */
#define RST_N          AGC_RESTART_NONE     /* DEC 0 placeholder */
#define RST_D          RST_DT_DIRECT        /* PRDTTAB holds the delta time */
#define RST_I          RST_DT_IMMEDIATE     /* PRDTTAB -0: immediately */
#define RST_X          RST_DT_INDIRECT      /* PRDTTAB -GENADR: delta time in erasable */
#define RST_G          RST_DT_LONG          /* LONGCALL: DP delta time in erasable */
#define RST_E0         RST_E_NONE           /* no erasable */
#define RST_JN         AGC_JOB_NONE         /* no job */
#define RST_TN         AGC_TASK_NONE        /* no task */
#define RST_600SECS_CS 60000                /* 600SECS 2DEC 60000: 600 s in cs        */
#define RST_SP_SCALE   16384                /* one SP word, 2^14                      */

typedef struct rst_group_tab
{
    const rst_entry_t *even;   /* 2 entries per even phase from G.2       */
    int                n_even; /* entries in the even table        */
    const rst_entry_t *odd;    /* 1 entry per odd phase from G.3          */
    int                n_odd;  /* entries in the odd table         */
} rst_group_tab_t;

/* ================= Comanche055 (CSM), pp. 212-219 ===================== */
static const rst_entry_t csm_even4[] = {
    /* 4.2SPOT = 1.2 = 2.2 = 3.2 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_PRECHECK}, /* 4.2 */
    {RST_J, 030, RST_D, RST_E0, AGC_JOB_THRUST_MONITOR_RESET, RST_TN},
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_PRECHECK}, /* 4.4 */
    {RST_T, 2996, RST_D, RST_E0, RST_JN, AGC_TASK_TTG_0},
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_PRECHECK}, /* 4.6 */
    {RST_T, 2496, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_5},
};
static const rst_entry_t csm_even5[] = {
    {RST_J, 032, RST_D, RST_E0, AGC_JOB_NORMLIZE, RST_TN}, /* 5.2 */
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC},
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_SERVICER, RST_TN}, /* 5.4 */
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC},
};
static const rst_entry_t csm_even6[] = {
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_GIMBAL_TRIM_RESTART}, /* 6.2 */
    {RST_T, 100, RST_D, RST_E0, RST_JN, AGC_TASK_CLOKTASK},
};
static const rst_entry_t csm_odd1[]  = {
    {RST_T, 120, RST_D, RST_E0, RST_JN, AGC_TASK_SETJTAG}, /* 1.3 */
    {RST_J, 010, RST_D, RST_E0, AGC_JOB_REDO40_9, RST_TN}, /* 1.5 */
    {RST_J, 010, RST_D, RST_E0, AGC_JOB_RELINUS, RST_TN},  /* 1.7 */
     {RST_J, 010, RST_D, RST_E0, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_TRACK,
     RST_TN},                                              /* 1.11 */
 };
static const rst_entry_t csm_odd2[]  = {
    {RST_L, 0, RST_G, RST_E_600SECS, RST_JN, AGC_TASK_STATEINT}, /* 2.3 */
     {RST_J, 05, RST_D, RST_E0, AGC_JOB_STATEINT_UPDATE, RST_TN},

     /* 2.5 */
     {RST_J, 010, RST_D, RST_E0, AGC_JOB_RENDEZVOUS_TRACKING_DATA,
     RST_TN}, /* 2.7 */
     {RST_J, 014, RST_D, RST_E0, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT,
      RST_TN},

     /* 2.11 */
     {RST_J, 010, RST_D, RST_E0, AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK,
     RST_TN}, /* 2.13 */
 };
static const rst_entry_t csm_odd3[] = {
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_TIMEBURN, RST_TN},     /* 3.3 */
    {RST_N, 0, RST_D, RST_E0, RST_JN, RST_TN},                 /* 3.5 */
    {RST_J, 022, RST_D, RST_E0, AGC_JOB_MATRXJOB, RST_TN},     /* 3.7 */
    {RST_J, 022, RST_D, RST_E0, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_RESTART,
     RST_TN},                                                  /* 3.11 */
    {RST_J, 022, RST_D, RST_E0, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_EPHEMERIS,
     RST_TN},                                                  /* 3.13 */
    {RST_T, 0, RST_X, RST_E_TGO_1, RST_JN, AGC_TASK_ENGINOFF}, /* 3.15 */
};
static const rst_entry_t csm_odd4[] = {
    {RST_T, 40, RST_D, RST_E0, RST_JN, AGC_TASK_DOTVCON},   /* 4.3 */
    {RST_T, 160, RST_D, RST_E0, RST_JN, AGC_TASK_DOSTRULL}, /* 4.5 */
    {RST_T, 500, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_0},    /* 4.7 */
    {RST_T, 250, RST_D, RST_E0, RST_JN,
     AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM},                   /* 4.11 */
    {RST_T, 200, RST_D, RST_E0, RST_JN,
     AGC_TASK_ENGINE_FAIL_REENABLE},                        /* 4.13 */
    {RST_J, 016, RST_D, RST_E0, AGC_JOB_COMPVER, RST_TN},   /* 4.15 */
    {RST_J, 016, RST_D, RST_E0, AGC_JOB_GYROCOMPASS_AZIMUTH_CHANGE, RST_TN},

    /* 4.17 */
    {RST_L, 0, RST_G, RST_E_PREREAD_DELAY, RST_JN, AGC_TASK_TIGBLNK}, /* 4.21 */
    {RST_J, 012, RST_D, RST_E0, AGC_JOB_PREBURN_STATE_VECTOR,
     RST_TN},                                                         /* 4.23 */
    {RST_J, 024, RST_D, RST_E0, AGC_JOB_IMU_REALIGN, RST_TN},         /* 4.25 */
    {RST_T, 250, RST_D, RST_E0, RST_JN, AGC_TASK_DOTVCRCS},           /* 4.27 */
    {RST_J, 013, RST_D, RST_E0, AGC_JOB_CSM_FINE_ALIGN, RST_TN},      /* 4.31 */
    {RST_T, 2100, RST_D, RST_E0, RST_JN,
     AGC_TASK_WAKE_CM_SM_SEPARATION},                                 /* 4.33 */
    {RST_J, 012, RST_D, RST_E0, AGC_JOB_POSTBURN, RST_TN},            /* 4.35 */
    {RST_T, 500, RST_D, RST_E0, RST_JN, AGC_TASK_TIGAVEG},            /* 4.37 */
    {RST_J, 017, RST_D, RST_E0, AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY,
     RST_TN},                                                         /* 4.41 */
    {RST_T, 0, RST_X, RST_E_S61DT, RST_JN,
     AGC_TASK_ENTRY_START_AVERAGE_G},                                 /* 4.43 */
    {RST_J, 013, RST_D, RST_E0, AGC_JOB_ENTRY_IMU_AXIS_TEST, RST_TN}, /* 4.45 */
    {RST_J, 017, RST_D, RST_E0, AGC_JOB_PRE_HUNT, RST_TN},            /* 4.47 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_ATERTASK},             /* 4.51 */
    {RST_T, 0, RST_I, RST_E0, RST_JN,
     AGC_TASK_ENGINE_FAIL_REIGNITE_PREP},                             /* 4.53 */
    {RST_J, 013, RST_D, RST_E0, AGC_JOB_ENTRY_UPCONTROL_DISPLAY,
     RST_TN},                                                         /* 4.55 */
    {RST_T, 0, RST_X, RST_E_PREREAD_DELAY, RST_JN, AGC_TASK_TIGON},   /* 4.57 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_IGNITION},             /* 4.61 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_DOSPSOFF},             /* 4.63 */
    {RST_T, 10, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_5},               /* 4.65 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_ENGINE_FAIL_SHUTDOWN}, /* 4.67 */
    {RST_T, 250, RST_D, RST_E0, RST_JN,
     AGC_TASK_ENGINE_FAIL_RCS_DAP},                                   /* 4.71 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_ENGINE_FAIL_RESUME},   /* 4.73 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_SPSOFF97},             /* 4.75 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_TIG_0},                /* 4.77 */
};
static const rst_entry_t csm_odd5[] = {
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC}, /* 5.3 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_REDO5_5},    /* 5.5 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_RSTGTS1, RST_TN},   /* 5.7 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_ALLOOP1},    /* 5.11 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_WTLISTNT, RST_TN},  /* 5.13 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_RESTEST1, RST_TN},  /* 5.15 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_ENDOFJOB, RST_TN},  /* 5.17 GEOSTRT4 */
    {RST_J, 022, RST_D, RST_E0, AGC_JOB_ALFLT1, RST_TN},    /* 5.21 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_SPECSTS},    /* 5.23 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_RESTEST3, RST_TN},  /* 5.25 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_RESTAIER, RST_TN},  /* 5.27 */
    {RST_N, 0, RST_D, RST_E0, RST_JN, RST_TN},              /* 5.31 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_RESCHNG, RST_TN},   /* 5.33 */
    {RST_N, 0, RST_D, RST_E0, RST_JN, RST_TN},              /* 5.35 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_CHEKAVEG},   /* 5.37 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_PREREAD},    /* 5.41 */
};
static const rst_entry_t csm_odd6[] = {
    {RST_T, 100, RST_D, RST_E0, RST_JN, AGC_TASK_CLOKTASK},       /* 6.3 */
    {RST_J, 030, RST_D, RST_E0, AGC_JOB_TIMEDIDR, RST_TN},        /* 6.5 */
    {RST_N, 0, RST_D, RST_E0, RST_JN, RST_TN},                    /* 6.7 */
    {RST_T, 0, RST_X, RST_E_CM_GYMDT, RST_JN, AGC_TASK_READGYMB}, /* 6.11 */
    {RST_N, 0, RST_D, RST_E0, RST_JN, RST_TN},                    /* 6.13 */
};

/* ================= Luminary099 (LM), pp. 239-243 ===================== */
static const rst_entry_t lm_even1[] = {
    /* 1.2 = 2.2 = 3.2 = 6.2     */
    {RST_J, 021, RST_D, RST_E0, AGC_JOB_ENDOFJOB, RST_TN}, /* 1.2 */
    {RST_T, 100, RST_D, RST_E0, RST_JN, AGC_TASK_TASKOVER},
};
static const rst_entry_t lm_even4[] = {
    {RST_T, 2500, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_5}, /* 4.2 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_REDO4_2},
};
static const rst_entry_t lm_even5[] = {
    {RST_J, 022, RST_D, RST_E0, AGC_JOB_NORMLIZE, RST_TN},  /* 5.2 */
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC},
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC}, /* 5.4 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_SERVICER, RST_TN},
};
static const rst_entry_t lm_odd1[]  = {
    {RST_T, 0, RST_X, RST_E_SAVET_30, RST_JN, AGC_TASK_ULLGTASK}, /* 1.3 */
 };
static const rst_entry_t lm_odd2[]  = {
    {RST_L, 0, RST_G, RST_E_600SECS, RST_JN, AGC_TASK_STATEINT}, /* 2.3 */
     {RST_J, 05, RST_D, RST_E0, AGC_JOB_STATEINT_UPDATE, RST_TN},

     /* 2.5 */
     {RST_T, 1500, RST_D, RST_E0, RST_JN,
     AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK},         /* 2.7 */
     {RST_J, 014, RST_D, RST_E0, AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK,
     RST_TN},                                             /* 2.11 */
    {RST_J, 010, RST_D, RST_E0, AGC_JOB_RELINUS, RST_TN}, /* 2.13 */
     {RST_J, 026, RST_D, RST_E0, AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART,
     RST_TN},                                             /* 2.15 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_REDO2_17}, /* 2.17 */
     {RST_T, 25, RST_D, RST_E0, RST_JN,
     AGC_TASK_ABORT_MONITOR},                             /* 2.21 */
 };
static const rst_entry_t lm_odd3[] = {
    {RST_T, 0, RST_X, RST_E_ZOOMTIME, RST_JN, AGC_TASK_ZOOM}, /* 3.3 */
    {RST_J, 020, RST_D, RST_E0, AGC_JOB_TIMEBURN, RST_TN},    /* 3.5 */
};
static const rst_entry_t lm_odd4[] = {
    {RST_J, 025, RST_D, RST_E0, AGC_JOB_GOABORT, RST_TN},      /* 4.3 */
    {RST_T, 50, RST_D, RST_E0, RST_JN, AGC_TASK_ULLAGOFF},     /* 4.5 */
    {RST_T, 500, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_0},       /* 4.7 */
    {RST_T, 0, RST_X, RST_E_TGO_1, RST_JN, AGC_TASK_ENGOFTSK}, /* 4.11 */
    {RST_J, 012, RST_D, RST_E0, AGC_JOB_POSTBURN, RST_TN},     /* 4.13 */
    {RST_T, 500, RST_D, RST_E0, RST_JN, AGC_TASK_TIG_30},      /* 4.15 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_TIG_5},         /* 4.17 */
    {RST_J, 013, RST_D, RST_E0, AGC_JOB_LM_FINE_ALIGN_PERFORM, RST_TN},

    /* 4.21 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_IGNITION},       /* 4.23 */
    {RST_L, 0, RST_G, RST_E_SAVET_30, RST_JN, AGC_TASK_TIG_35}, /* 4.25 */
    {RST_J, -025, RST_D, RST_E0, AGC_JOB_DPS_ABORT_COMMANDED,
     RST_TN},                                                   /* 4.27 */
    {RST_J, -025, RST_D, RST_E0, AGC_JOB_APS_ABORT_COMMANDED,
     RST_TN},                                                   /* 4.31 */
    {RST_J, -031, RST_D, RST_E0, AGC_JOB_GO_IDLE_FIX, RST_TN},  /* 4.33 */
    {RST_J, -031, RST_D, RST_E0, AGC_JOB_GOPOODOO, RST_TN},     /* 4.35 */
    {RST_J, -025, RST_D, RST_E0, AGC_JOB_COMFAIL, RST_TN},      /* 4.37 */
};
static const rst_entry_t lm_odd5[] = {
    {RST_T, 200, RST_D, RST_E0, RST_JN, AGC_TASK_REREADAC}, /* 5.3 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_REDO5_5},    /* 5.5 */
    {RST_T, 0, RST_I, RST_E0, RST_JN, AGC_TASK_BIBIBIAS},   /* 5.7 */
};
static const rst_entry_t lm_odd6[] = {
    {RST_T, 100, RST_D, RST_E0, RST_JN, AGC_TASK_CLOKTASK}, /* 6.3 */
    {RST_J, 030, RST_D, RST_E0, AGC_JOB_TIMEDIDR, RST_TN},  /* 6.5 */
    {RST_J, 017, RST_D, RST_E0, AGC_JOB_REDO6_7, RST_TN},   /* 6.7 */
};

/* SIZETAB (C p.212 / L p.239), index group-1. */
static const rst_group_tab_t csm_tab[AGC_RESTART_GROUPS] = {
    {csm_even4, (int) (sizeof (csm_even4) / sizeof (csm_even4[0])), csm_odd1,
     (int) (sizeof (csm_odd1) / sizeof (csm_odd1[0]))},
    {csm_even4, (int) (sizeof (csm_even4) / sizeof (csm_even4[0])), csm_odd2,
     (int) (sizeof (csm_odd2) / sizeof (csm_odd2[0]))},
    {csm_even4, (int) (sizeof (csm_even4) / sizeof (csm_even4[0])), csm_odd3,
     (int) (sizeof (csm_odd3) / sizeof (csm_odd3[0]))},
    {csm_even4, (int) (sizeof (csm_even4) / sizeof (csm_even4[0])), csm_odd4,
     (int) (sizeof (csm_odd4) / sizeof (csm_odd4[0]))},
    {csm_even5, (int) (sizeof (csm_even5) / sizeof (csm_even5[0])), csm_odd5,
     (int) (sizeof (csm_odd5) / sizeof (csm_odd5[0]))},
    {csm_even6, (int) (sizeof (csm_even6) / sizeof (csm_even6[0])), csm_odd6,
     (int) (sizeof (csm_odd6) / sizeof (csm_odd6[0]))},
};
static const rst_group_tab_t lm_tab[AGC_RESTART_GROUPS]  = {
     {lm_even1, (int) (sizeof (lm_even1) / sizeof (lm_even1[0])), lm_odd1,
      (int) (sizeof (lm_odd1) / sizeof (lm_odd1[0]))},
     {lm_even1, (int) (sizeof (lm_even1) / sizeof (lm_even1[0])), lm_odd2,
      (int) (sizeof (lm_odd2) / sizeof (lm_odd2[0]))},
     {lm_even1, (int) (sizeof (lm_even1) / sizeof (lm_even1[0])), lm_odd3,
      (int) (sizeof (lm_odd3) / sizeof (lm_odd3[0]))},
     {lm_even4, (int) (sizeof (lm_even4) / sizeof (lm_even4[0])), lm_odd4,
      (int) (sizeof (lm_odd4) / sizeof (lm_odd4[0]))},
     {lm_even5, (int) (sizeof (lm_even5) / sizeof (lm_even5[0])), lm_odd5,
      (int) (sizeof (lm_odd5) / sizeof (lm_odd5[0]))},
     {lm_even1, (int) (sizeof (lm_even1) / sizeof (lm_even1[0])), lm_odd6,
      (int) (sizeof (lm_odd6) / sizeof (lm_odd6[0]))},
 };

/**********************************************************************
 * ITSATBL / ITSEVEN / PHSPART2 (RESTARTS_ROUTINE C pp.1416-1419 /
 * L pp.1305-1308): the entry for phase `phase` (>= 2, octal value) of
 * `group`. For an even phase `part` 0 is the first entry and 1 the
 * second (PHSPART2).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6 (agc_restart_group checked it)
 *   phase -- the table phase, 2..0177; the phase codes the programs
 *            set lie inside their group's tables (EVA, 2026-10-12)
 *   part -- 0, or 1 for the second entry of an even phase
 *   out -- receives the entry
 *
 * RETURNS: none
 **********************************************************************/
void rst_table_lookup(int group, int phase, int part, rst_entry_t *out)
{
    const rst_group_tab_t *t = &csm_tab[group - 1]; /* the group's even and odd tables */
    int                    idx;                     /* entry index in the table */
    const rst_entry_t     *base;                    /* the even or the odd table */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        t = &lm_tab[group - 1];
    }

    if ((phase % 2) == 0)
    {
        idx  = (phase - 2) + part; /* 2 entries per even phase      */
        base = t->even;
    }
    else
    {
        idx  = (phase - 3) / 2;
        base = t->odd;
    }

    *out = base[idx];
}

/**********************************************************************
 * ITSINDIR (RESTARTS_ROUTINE C p.1415 / L p.1304): NDX A / CA 1 -- the
 * single word at GENADR. For SAVET-30 and P40TMP that is the first word
 * of the B(2) pair, which TIG-30 (L p.735) and P47CSM (C p.691) write
 * with a TS: the writers in C store that SP delta time as the whole DP
 * (src/core/restart_delays.c), so the value read here is the low part
 * of the C DP.
 *
 * ARGUMENTS:
 *   src -- the erasable the table names
 *   out -- receives the delta time, cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG for an erasable that
 *          holds no SP delta time (a table error)
 **********************************************************************/
agc_status_t rst_read_sp(rst_erasable_t src, int16_t *out)
{
    agc_status_t st = AGC_STATUS_OK; /* BAD_ARG: no SP delta time */

    switch (src)
    {
        case RST_E_TGO_1:         /* TGO +1: low word of TGO, cs          */
            *out = (int16_t) (agc_erase_tgo() % RST_SP_SCALE);
            break;
        case RST_E_600SECS:       /* high word of 2DEC 60000              */
            *out = (int16_t) (RST_600SECS_CS / RST_SP_SCALE);
            break;
        case RST_E_SAVET_30:      /* LM -GENADR SAVET-30: TIG-30 TS, cs     */
            *out = (int16_t) (agc_erase_savet_30() % RST_SP_SCALE);
            break;
        case RST_E_ZOOMTIME:      /* LM ZOOMTIME (pad load), cs             */
            *out = agc_erase_zoomtime();
            break;
        case RST_E_PREREAD_DELAY: /* CSM -GENADR P40TMP: P47CSM TS, cs  */
            *out = (int16_t) (agc_erase_p40tmp() % RST_SP_SCALE);
            break;
        case RST_E_S61DT:         /* CSM S61DT, cs                             */
            *out = agc_erase_s61dt();
            break;
        case RST_E_CM_GYMDT:      /* CSM CM/GYMDT, cs                       */
            *out = agc_erase_cm_gymdt();
            break;
        case RST_E_NONE:
        default:
            st   = AGC_STATUS_BAD_ARG;
            break;
    }

    return (st);
}

/**********************************************************************
 * ITSLGCL1 (RESTARTS_ROUTINE C p.1417 / L p.1306): EXTEND / NDX A /
 * DCA 0 -- the DP delta time of a LONGCALL restart, cs.
 *
 * ARGUMENTS:
 *   src -- the erasable the table names
 *   out -- receives the delta time, cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG for an erasable that
 *          holds no DP delta time (a table error)
 **********************************************************************/
agc_status_t rst_read_dp(rst_erasable_t src, uint32_t *out)
{
    agc_status_t st = AGC_STATUS_OK; /* BAD_ARG: no DP delta time */

    switch (src)
    {
        case RST_E_600SECS:            /* 600SECS 2DEC 60000 (FRESH_START)      */
            *out = (uint32_t) RST_600SECS_CS;
            break;
        case RST_E_SAVET_30:           /* LM SAVET-30 B(2), cs                   */
            *out = (uint32_t) agc_erase_savet_30();
            break;
        case RST_E_PREREAD_DELAY:      /* CSM P40TMP B(2), cs                */
            *out = (uint32_t) agc_erase_p40tmp();
            break;
        case RST_E_TGO_1:
        case RST_E_ZOOMTIME:
        case RST_E_S61DT:
        case RST_E_CM_GYMDT:
        case RST_E_NONE:
        default:
            st   = AGC_STATUS_BAD_ARG; /* not a DP delta time  */
            break;
    }

    return (st);
}
