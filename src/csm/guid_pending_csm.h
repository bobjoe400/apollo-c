/*
 * guid_pending_csm.h -- Comanche-only symbols guid-powered needs from other
 * lanes that do not exist yet (docs/modules/guid-powered.md, "Requested core
 * additions"). Delete each block when the owner's header provides it.
 */
#ifndef GUID_PENDING_CSM_H
#define GUID_PENDING_CSM_H

#include "../guidance/guid_pending.h"
#include "agc/agc_flags_csm.h"
#include "agc/agc_csm_dap.h"
#include "agc/agc_csm_entry.h"

/* ---- entry DAP angle registers (guid-entry) ------------------------------ */
agc_sp_t agc_erase_aog(void);               /* AOG outer gimbal, /180 2's comp */
agc_sp_t agc_erase_aig(void);               /* AIG inner gimbal               */
agc_sp_t agc_erase_amg(void);               /* AMG middle gimbal              */
void     agc_erase_set_cm_gymdt(agc_sp_t v); /* CM/GYMDT entry DAP gyro dt    */

/* ---- marks and optics (prog-rendz / imu / sys-misc) ---------------------- */
void     agc_erase_set_vhfcnt(agc_sp_t v);  /* VHFCNT VHF range mark count   */
void     agc_erase_set_mrkbuf2(int index, agc_sp_t v); /* MRKBUF2 +index     */
void     agc_erase_set_optind(agc_sp_t v);  /* OPTIND optics mode (T4RUPT)   */

/* ---- TVC output counters and T5 (dap-csm / exec) ------------------------- */
void     agc_erase_set_tvcpitch(agc_sp_t v); /* TVCPITCH counter, 85.41"/bit */
void     agc_erase_set_tvcyaw(agc_sp_t v);   /* TVCYAW counter               */
agc_sp_t agc_erase_time5(void);              /* TIME5 counter                */
void     agc_erase_set_time5(agc_sp_t v);

/* ---- other lanes' routines called from the CSM burn programs ------------- */
agc_status_t agc_csm_s11_1(agc_interp_t *ip);  /* S11.1 V I, H, HDOT (P11, guid-entry) */
agc_status_t agc_csm_satstkon(const agc_job_ctx_t *ctx); /* SATSTKON (P11 / V46, guid-entry) */
agc_status_t agc_dsky_gopin(const agc_job_ctx_t *ctx);   /* GOPIN (extended verb exit, pinball) */

#endif /* GUID_PENDING_CSM_H */
