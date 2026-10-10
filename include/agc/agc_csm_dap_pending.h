/*
 * csm_dap_pending.h -- shared symbols the CSM DAP needs that are not yet
 * published in include/agc (docs/agents/_common.md naming convention).
 *
 * INTEGRATION: the lead moves the job/task ids into agc_jobs.h (and adds
 * the dispatch cases), the accessors into agc_state.h, and then deletes
 * the corresponding lines here. Every name follows the convention, so no
 * DAP source file needs to change. Provisional ids use the 0x0D00 range;
 * REDORCS, REDOTVC and TVCEXEC repeat exec_pending.h token for token.
 */
#ifndef CSM_DAP_PENDING_H
#define CSM_DAP_PENDING_H

#include <stdint.h>
#include "agc/agc_word.h"
#include "agc/agc_jobs.h"

/* ---- T5 / T6 programs and waitlist tasks (dap-csm) --------------------- */
#define AGC_TASK_REDORCS   ((agc_task_id_t)0x0EAB) /* T5LOC: RCS DAP (dap-csm)    */
#define AGC_TASK_REDOTVC   ((agc_task_id_t)0x0EAC) /* T5LOC: TVC DAP (dap-csm)    */
#define AGC_TASK_TVCEXEC   ((agc_task_id_t)0x0EAE) /* ENEMA, TVC on (dap-csm)     */
#define AGC_TASK_RCSATT    ((agc_task_id_t)0x0D01) /* T5: RCS DAP phases 1,2      */
#define AGC_TASK_JETSLECT  ((agc_task_id_t)0x0D02) /* T5: RCS jet selection       */
#define AGC_TASK_T6START   ((agc_task_id_t)0x0D03) /* T6: RCS jet channel timing  */
#define AGC_TASK_TVCDAPON  ((agc_task_id_t)0x0D04) /* T5: TVC DAP start (DOTVCON) */
#define AGC_TASK_TVCINIT1  ((agc_task_id_t)0x0D05) /* T5: TVC initialisation pt 2 */
#define AGC_TASK_DAPINIT   ((agc_task_id_t)0x0D06) /* T5: TVC CDU rate init       */
#define AGC_TASK_PITCHDAP  ((agc_task_id_t)0x0D07) /* T5: TVC pitch DAP           */
#define AGC_TASK_YAWDAP    ((agc_task_id_t)0x0D08) /* T5: TVC yaw DAP             */
#define AGC_TASK_ENABL2    ((agc_task_id_t)0x0D09) /* T5: TVC restart, step 2     */
#define AGC_TASK_CMDSOUT   ((agc_task_id_t)0x0D0A) /* T5: TVC restart, step 3     */
#define AGC_TASK_NOROLL1   ((agc_task_id_t)0x0D0B) /* T6: TVC roll jets off       */
#define AGC_TASK_ROLLDAP   ((agc_task_id_t)0x0D0C) /* waitlist: TVC roll DAP      */
#define AGC_TASK_HACKWLST  ((agc_task_id_t)0x0D0D) /* waitlist: stroke test burst */
#define AGC_TASK_EXRSTRT   ((agc_task_id_t)0x0D0E) /* waitlist: TVCEXEC restart   */
#define AGC_TASK_RCSUP     ((agc_task_id_t)0x0D0F) /* T5: RCSUP (P40 TVC -> RCS)  */
#define AGC_JOB_AMBGUPDT   ((agc_job_id_t)0x0D20)  /* job: AMGB matrix, PRIO34    */
#define AGC_JOB_STRKTSTI   ((agc_job_id_t)0x0D21)  /* job: V68 stroke test init   */

/* ---- erasable accessors requested from state-flags ---------------------
 * TVCPITCH = CDUSCMD (054) and TVCYAW = CDUTCMD (053): optics CDU error
 * counter drive registers, 85.41 arcsec/bit in TVC mode. Same prototypes
 * as src/csm/burns/guid_pending.h (guid-powered requests the setters too). */
agc_sp_t agc_erase_tvcpitch(void);
void     agc_erase_set_tvcpitch(agc_sp_t v); /* TVCPITCH counter, 85.41"/bit */
agc_sp_t agc_erase_tvcyaw(void);
void     agc_erase_set_tvcyaw(agc_sp_t v);   /* TVCYAW counter, 85.41"/bit   */

#endif /* CSM_DAP_PENDING_H */
