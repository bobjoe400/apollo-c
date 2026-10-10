/*
 * align_internal.h -- helpers shared by the alignment and marking files of
 * the prog-align lane (src/programs/alignment/align_*.c, src/csm/, src/lm/).  Not a
 * public interface.
 */
#ifndef ALIGN_INTERNAL_H
#define ALIGN_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/core/state.h"
#include "agc/io/dsky.h"
#include "agc/imu/imu.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/alignment_pending.h"

/* Integration entrances used by the alignment code (INTEGRATION_INIT.).   */
typedef enum align_integ_kind {
    ALIGN_CSMPREC = 0,
    ALIGN_LEMPREC,
    ALIGN_CSMCONIC,
    ALIGN_LEMCONIC
} align_integ_kind_t;

/* CSMPREC/LEMPREC/CSMCONIC/LEMCONIC to tdec1 (cs B28) in the shared
 * integration context; *out = RATT.. X2.  BUSY: the job sleeps in INTSTALL
 * and is resumed as resume_job, which must call this again.              */
agc_status_t align_integ(agc_interp_t *ip, align_integ_kind_t kind,
                         agc_dp_t tdec1, agc_job_id_t resume_job,
                         agc_integ_out_t *out);

/* A flashing display with the three astronaut returns (call CADR +1, +2,
 * +3).  For GOPERF1 / GOPERF4 kinds vn is the checklist / option code.   */
agc_status_t align_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                         uint16_t vn, agc_job_id_t term, agc_job_id_t proceed,
                         agc_job_id_t enter);
/* A non-flashing display (GODSP, GODSPR, GODSPRET, CLEANDSP, KLEENEX).    */
agc_status_t align_show(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                        uint16_t vn);
/* The answer to the last flashing display was V32 (recycle).             */
bool         align_resp_recycle(void);

/* GOTO / TC to another resumption point of the same job (CHANG1-like:
 * the job continues as `next`); the caller returns its status.          */
agc_status_t align_goto(agc_job_id_t next);

/* IMUSTALL / OPTSTALL (AOTSTALL).  *done = true: the event already came,
 * go on now.  *done = false: the job sleeps and continues as wake_as.
 * A bad end does CURTAINS and returns AGC_STATUS_ALARM.                  */
agc_status_t align_stall(agc_imu_device_t dev, agc_job_id_t wake_as, bool *done);
/* In the job woken by a stall: AGC_STATUS_OK for GOODEND, otherwise
 * CURTAINS and AGC_STATUS_ALARM.                                         */
agc_status_t align_stall_outcome(agc_imu_device_t dev);

/* PHASCHNG with a type C code and a job restart point at priority prio. */
agc_status_t align_phase_job(uint16_t code, agc_job_id_t job, agc_prio_t prio);

/* PLANET (planet.c): the line of sight of STARCODE at t, left in
 * agc_align_ram()->planet_los; the job continues as ret.                 */
agc_status_t align_planet(const agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret);

/* PHASCHNG with a fixed (type A) group 4 code.                           */
agc_status_t align_phase4(uint16_t code);

/* NCOARSE (Luminary p. 951): SET1/PDT plus PIPAX/Y/Z = -0.               */
agc_status_t align_ncoarse(void);

/* COARSE (Luminary p. 951; the same sequence is inline in the Comanche
 * callers): IMUCOARS to THETAD, IMUSTALL, IMUFINE, IMUSTALL, then the job
 * continues as ret.  A bad end does CURTAINS.  The caller returns the
 * status.                                                                */
agc_status_t align_coarse(agc_job_id_t ret);

/* LOADTIME: TIME2, TIME1 as a DP, cs B28.                                */
agc_dp_t     align_loadtime(void);
/* DSPTEM1, DSPTEM1 +1 as a DP (cs B28), and the store.                    */
agc_dp_t     align_dsptem1(void);
agc_status_t align_set_dsptem1(agc_dp_t v);
/* Raw 15-bit 1's complement word <-> signed SP.                           */
agc_sp_t     align_word_to_sp(uint16_t w);
uint16_t     align_sp_to_word(agc_sp_t v);

/* The present IMU CDUs X, Y, Z (raw words).                               */
void         align_read_cdus(uint16_t cdu[3]);
/* CDUTRIG + CALCSMSC of the present CDUs: rows XNB.. wrt the present SM.  */
agc_status_t align_nb_present(agc_interp_t *ip, agc_cdutrig_t *t, agc_mat_t *nb);

/* SET1/PDT (CM) / the NCOARSE preamble (LM): 1/PIPADT = TIME1, GCOMP = 0,
 * DRIFTFLG set.                                                          */
agc_status_t align_set1_pdt(void);

/* The VN pseudo-op: verb and noun as decimal pairs packed 0VVVVVVVNNNNNNN
 * (same packing as agc_dsky_vn(), usable in constant expressions).       */
#define ALIGN_VN(verb, noun) ((uint16_t)(((verb) << 7) | (noun)))

/* Priorities used by the mark system and the jobs it creates.            */
#define ALIGN_PRIO13 ((agc_prio_t)013)
#define ALIGN_PRIO15 ((agc_prio_t)015)
#define ALIGN_PRIO24 ((agc_prio_t)024)
#define ALIGN_PRIO32 ((agc_prio_t)032)
#define ALIGN_PRIO5  ((agc_prio_t)05)
#define ALIGN_PRIO4  ((agc_prio_t)04)

/* Restart group 4 phase codes used by P50-P57.                           */
#define ALIGN_PH_04024 ((uint16_t)04024)   /* type A: group 4, phase 2     */
#define ALIGN_PH_00014 ((uint16_t)000014)  /* type A: group 4, phase 1     */
#define ALIGN_PH_05024 ((uint16_t)05024)   /* type C: group 4 + job PRIO13 */
#define ALIGN_PH_00254 ((uint16_t)000254)  /* PROG52 (CM): group 4 phase 25*/

#endif /* ALIGN_INTERNAL_H */
