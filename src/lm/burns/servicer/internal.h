/*
 * internal.h -- stages of the Luminary SERVICER job shared by
 * src/lm/burns/servicer/average_g.c and landing_radar.c (the landing radar
 * update). Each stage is a label of SERVICER.agc that the other file
 * branches to; they all run inside the SERVICER job.
 */
#ifndef SERVICER_LM_INTERNAL_H
#define SERVICER_LM_INTERNAL_H

#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/conic.h"
#include "agc/programs/rendezvous.h"

/* QUIKFAZ5 (p.890): make `resume` the group-5 restart point.             */
void agc_lm_serv_quikfaz5(agc_job_id_t resume);

/* CONTSERV (p.875): clear the measurement discretes, LR lamps, COPYCYC1. */
agc_status_t agc_lm_serv_contserv(agc_job_ctx_t *ctx);

/* COPYCYC1 .. COPYCYC2 (p.876-877), then COPYCYCL.                       */
agc_status_t agc_lm_serv_copycyc1(agc_job_ctx_t *ctx);

/* COPYCYCL (p.861): copy cycle, DV monitor, SERVOUT, AVGEXIT.          */
agc_status_t agc_lm_serv_copycycl(agc_job_ctx_t *ctx);

/* MUNRETRN (p.874): return point of MUNRVG; LR logic, then COPYCYC1.    */
agc_status_t agc_lm_serv_munretrn(agc_job_ctx_t *ctx);

/* UPDATCHK (p.884) / VMEASCHK (p.886) / VALTCHK (p.889): the LR update.   */
agc_status_t agc_lm_serv_updatchk(agc_job_ctx_t *ctx);
agc_status_t agc_lm_serv_vmeaschk(agc_job_ctx_t *ctx);
agc_status_t agc_lm_serv_valtchk(agc_job_ctx_t *ctx);

/* RUPDATED / VUPDATED (p.886, 889): GNURVST of GNUR into R1S or V1S.     */
agc_status_t agc_lm_serv_rupdated(agc_job_ctx_t *ctx);
agc_status_t agc_lm_serv_vupdated(agc_job_ctx_t *ctx);

#endif /* SERVICER_LM_INTERNAL_H */
