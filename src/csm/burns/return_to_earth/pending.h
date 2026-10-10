/*
 * pending.h -- PENDING symbols used by P37 (lane p37).
 *
 * Every declaration here is a request to another lane or to integration
 * (docs/modules/p37.md, "Requested core additions" / "Requested
 * jobs-tasks"). Nothing here is defined by this lane except the job
 * functions, whose prototypes move to agc/csm/entry.h at integration.
 * Delete each block when its owner publishes it.
 */
#ifndef RETURN_TO_EARTH_PENDING_H
#define RETURN_TO_EARTH_PENDING_H

#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/exec/jobs.h"

/* ---- P37 continuation jobs (provisional ids next to the entry lane's
 * 0x0C00 range; AGC_JOB_RETURN_TO_EARTH itself is 0x0C16 in
 * agc/csm/entry_pending.h). Lead: add to agc/exec/jobs.h + dispatch.c. -- */
#define AGC_JOB_RETURN_TO_EARTH_TIG_DISPLAY        ((agc_job_id_t)0x0C30) /* V06N33 TIG input       */
#define AGC_JOB_RETURN_TO_EARTH_ENTRY_DISPLAY      ((agc_job_id_t)0x0C31) /* V06N60 GAMMAEI/DV input */
#define AGC_JOB_RETURN_TO_EARTH_CONIC_SEARCH       ((agc_job_id_t)0x0C32) /* RTE299 .. P37E          */
#define AGC_JOB_RETURN_TO_EARTH_TRANSIT_DISPLAY    ((agc_job_id_t)0x0C33) /* RTEVN V06N39            */
#define AGC_JOB_RETURN_TO_EARTH_ENTRY_CONDITIONS_DISPLAY ((agc_job_id_t)0x0C34) /* RTEVN V06N60    */
#define AGC_JOB_RETURN_TO_EARTH_DELTA_V_DISPLAY    ((agc_job_id_t)0x0C35) /* RTEVN V06N81            */
#define AGC_JOB_RETURN_TO_EARTH_PRECISION          ((agc_job_id_t)0x0C36) /* RTE505, PREC100         */
#define AGC_JOB_RETURN_TO_EARTH_PRECISION_RESUME   ((agc_job_id_t)0x0C37) /* PREC100 after INTSTALL  */
#define AGC_JOB_RETURN_TO_EARTH_PROPULSION_OPTION  ((agc_job_id_t)0x0C38) /* P37N OPTION1/2          */
#define AGC_JOB_RETURN_TO_EARTH_OPTION_DISPLAY     ((agc_job_id_t)0x0C39) /* P37N V04N06             */
#define AGC_JOB_RETURN_TO_EARTH_IGNITION_TIME      ((agc_job_id_t)0x0C3A) /* P37Q/R/T biased TIG     */
#define AGC_JOB_RETURN_TO_EARTH_BIASED_TIG_DISPLAY ((agc_job_id_t)0x0C3B) /* P37T V06N33             */
#define AGC_JOB_RETURN_TO_EARTH_TARGETING          ((agc_job_id_t)0x0C3C) /* RTENCK1, P37U/V         */
#define AGC_JOB_RETURN_TO_EARTH_TARGET_STORE       ((agc_job_id_t)0x0C3D) /* P37W, VN1645 return     */
#define AGC_JOB_RETURN_TO_EARTH_ALARM_DISPLAY      ((agc_job_id_t)0x0C3E) /* P370ALRM V05N09         */

/* Job functions (program.c); move to agc/csm/entry.h.             */
agc_status_t agc_job_return_to_earth_tig_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_entry_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_conic_search(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_transit_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_entry_conditions_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_delta_v_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_precision(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_precision_resume(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_propulsion_option(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_option_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_ignition_time(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_biased_tig_display(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_targeting(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_target_store(agc_job_ctx_t *ctx);
agc_status_t agc_job_return_to_earth_alarm_display(agc_job_ctx_t *ctx);

/* ---- state-flags: shared erasables -------------------------------------- */
/* T3TOT4 (EQUALS RELDELV +2, noun 39 for P34/P35/P37), cs B28.            */
agc_dp_t agc_erase_t3tot4(void);
void     agc_erase_set_t3tot4(agc_dp_t v);
/* RTED1 (I(2)PL, VGAMMA POLY COEF B-3): pad load, X(T2) polynomial D1.     */
agc_dp_t agc_erase_rted1(void);
/* P37RANGE (I(1)PL, EBANK6): entry range override, revs (0 = computed).   */
agc_sp_t agc_erase_return_to_earth_range(void);
/* Same declarations as src/programs/alignment/pending.h and
 * src/csm/burns/guid_pending.h (OPTION1 +index, N06; VHFCNT).             */
agc_sp_t agc_erase_option1(int index);
void     agc_erase_set_option1(int index, agc_sp_t v);
void     agc_erase_set_vhfcnt(agc_sp_t v);

/* ---- prog-rendz: VN1645 (P34-35_P74-75.agc p. 481) as a public entry.
 * Same as rz_ttogo_display (src/programs/displays/display.c): +MGA,
 * COMPTGO, QSAVED = resume_as, DELAYJOB 1 s, V16N45.                      */
agc_status_t agc_rendz_ttogo_display_start(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                           agc_job_id_t resume_as);

#endif /* RETURN_TO_EARTH_PENDING_H */
