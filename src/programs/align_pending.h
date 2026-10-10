/*
 * align_pending.h -- interfaces the alignment code (prog-align) uses that
 * other lanes own and have not published in include/agc yet.
 *
 * INTEGRATION: every declaration below is an ASSUMED interface. When the
 * owning header appears, delete the matching block here and include that
 * header instead. Each block names its owner. Listed again in
 * docs/modules/prog-align.md under "Requested core additions".  Where
 * another lane already uses a name for the same thing (sys-misc
 * sysmisc.h / sysmisc_ext.h), the same name and signature are used here.
 *
 * Job/task identifiers: object-like constants AGC_JOB_<LABEL> /
 * AGC_TASK_<LABEL> with provisional values in the 0x0A00 range; the lead
 * replaces them with real agc_jobs.h members and dispatch.c cases.
 */
#ifndef ALIGN_PENDING_H
#define ALIGN_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"

/* ==== state-flags: shared erasables not yet in agc_state.h =============== */
agc_sp_t  agc_erase_option1(int index);        /* OPTION1 +index (N06), 0..2 */
void      agc_erase_set_option1(int index, agc_sp_t v);
uint16_t  agc_erase_extvbact(void);            /* EXTVBACT (as sys-misc)     */
void      agc_erase_set_extvbact(uint16_t v);
uint16_t  agc_erase_optmodes(void);            /* OPTMODES (CM, as sys-misc) */
void      agc_erase_set_optmodes(uint16_t v);
void      agc_erase_set_optind(agc_sp_t v);    /* OPTIND (CM, as sys-misc)   */
uint16_t  agc_erase_radmodes(void);            /* RADMODES (LM, as sys-misc) */
agc_dp_t  agc_erase_samptime(void);            /* SAMPTIME cs B28            */
void      agc_erase_set_samptime(agc_dp_t v);
agc_dp_t  agc_erase_tland(void);               /* TLAND cs B28 (guid-landing)*/
void      agc_erase_set_tland(agc_dp_t v);
uint16_t  agc_erase_aotaz(int index);          /* AOTAZ -1+index, cdu (pad)  */
uint16_t  agc_erase_aotel(int index);          /* AOTEL -1+index, cdu (pad)  */
agc_sp_t  agc_erase_landmark(void);            /* LANDMARK (CM, prog-rendz)  */
agc_sp_t  agc_erase_cduchkwd(void);            /* CDUCHKWD cs (CM, pad)      */
agc_sp_t  agc_erase_swsample(void);            /* SWSAMPLE (CM, T4RUPT)      */
uint16_t  agc_erase_agsword(void);             /* AGSWORD old downlist code  */
void      agc_erase_set_mrkbuf(int buf, int index, uint16_t v); /* MRKBUF1/2 */
uint16_t  agc_erase_mrkbuf(int buf, int index);
void      agc_erase_set_markdown(int index, uint16_t v); /* MARKDOWN +0..6   */
void      agc_erase_set_mark2dwn(int index, uint16_t v); /* MARK2DWN +0..6   */

/* ==== sys-misc (src/io/sysmisc.h, not public yet) ======================== */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as); /* DELAYJOB */
agc_status_t agc_svc_b5off(void);                                       /* B5OFF    */
agc_status_t agc_telem_select_list(int dnlstcod);                       /* TS DNLSTCOD */

/* ==== exec-sched: VAC areas claimed by the mark system ==================== */
/* MARKOK/MKVAC: first free VACnUSE, marked occupied (CCS VACnUSE loop);
 * AGC_STATUS_FULL when none (the caller BAILOUTs 01207).                   */
agc_status_t agc_exec_vac_claim(int *vac_index);
agc_status_t agc_exec_vac_release(int vac_index);   /* TS VACnUSE          */

/* ==== prog-rendz (P20-P25, Comanche) ===================================== */
agc_status_t agc_rendz_llasrd(void);     /* LLASRD: LAT,LONG,ALT -> N89 regs */
agc_status_t agc_rendz_llasrda(void);    /* LLASRDA: N89 regs -> LAT,LONG,ALT*/
/* R61CSM: CSM tracking attitude; returns after the maneuver/update.        */
agc_status_t agc_rendz_r61csm(const agc_job_ctx_t *ctx, agc_job_id_t ret_job);

/* ==== provisional job and task identifiers ================================ */
#define AGC_JOB_CHKSDATA_PROCEED   ((agc_job_id_t)0x0A01)
#define AGC_JOB_CHKSDATA_ENTER     ((agc_job_id_t)0x0A02)
#define AGC_JOB_R55_1              ((agc_job_id_t)0x0A03)
#define AGC_JOB_R55_2              ((agc_job_id_t)0x0A04)
#define AGC_JOB_R55_PULSED         ((agc_job_id_t)0x0A05)
#define AGC_JOB_R55RET             ((agc_job_id_t)0x0A06)
#define AGC_JOB_SBANDANT_CM        ((agc_job_id_t)0x0A07)
#define AGC_JOB_SBANDANT_CM_RECYCLE ((agc_job_id_t)0x0A08)
#define AGC_JOB_SBANDANT_LM        ((agc_job_id_t)0x0AAB)
#define AGC_JOB_SBANDANT_LM_RECYCLE ((agc_job_id_t)0x0AAC)
#define AGC_JOB_SBAND_B5OFF        ((agc_job_id_t)0x0AAD)
#define AGC_JOB_V89CALL            ((agc_job_id_t)0x0A09)
#define AGC_JOB_V89RECL            ((agc_job_id_t)0x0A0A)
#define AGC_JOB_V89_MANEUVER       ((agc_job_id_t)0x0A0B)
#define AGC_JOB_AGSINIT            ((agc_job_id_t)0x0A0C)
#define AGC_JOB_AGSDISPK           ((agc_job_id_t)0x0A0D)
#define AGC_JOB_AGSDISPK_ENTER     ((agc_job_id_t)0x0A0E)
#define AGC_JOB_AGSVCALC           ((agc_job_id_t)0x0A0F)
#define AGC_JOB_AGS_LEMPREC        ((agc_job_id_t)0x0A10)
#define AGC_JOB_AGS_CSMPREC        ((agc_job_id_t)0x0A11)
#define AGC_JOB_AGS_AFTER_DELAY    ((agc_job_id_t)0x0A12)
#define AGC_JOB_AGS_CKSTALL        ((agc_job_id_t)0x0A13)
#define AGC_JOB_AGSEND             ((agc_job_id_t)0x0A14)
#define AGC_JOB_MKVB51             ((agc_job_id_t)0x0A15)
#define AGC_JOB_MKVB50             ((agc_job_id_t)0x0A16)
#define AGC_JOB_TERMSXT            ((agc_job_id_t)0x0A17)
#define AGC_JOB_ENTANSWR           ((agc_job_id_t)0x0A18)
#define AGC_JOB_MKVB5X             ((agc_job_id_t)0x0A19)
#define AGC_JOB_MARKDISP           ((agc_job_id_t)0x0A1A) /* P23 (prog-rendz) */
#define AGC_JOB_GETDAT             ((agc_job_id_t)0x0A1B)
#define AGC_JOB_DODAT              ((agc_job_id_t)0x0A1C)
#define AGC_JOB_CODE7              ((agc_job_id_t)0x0A1D)
#define AGC_JOB_CODE7_PROCEED      ((agc_job_id_t)0x0A1E)
#define AGC_JOB_GETMKS             ((agc_job_id_t)0x0A1F)
#define AGC_JOB_MARKCHEX           ((agc_job_id_t)0x0A20)
#define AGC_JOB_KILLAOT            ((agc_job_id_t)0x0A21)
#define AGC_JOB_CHANGEVB           ((agc_job_id_t)0x0A22)
#define AGC_JOB_DSPV6N79           ((agc_job_id_t)0x0A23)
#define AGC_JOB_SURFEND            ((agc_job_id_t)0x0A24)
#define AGC_JOB_DSPV6N79_ENTER     ((agc_job_id_t)0x0A25)
#define AGC_JOB_PROG52             ((agc_job_id_t)0x0A26)
#define AGC_JOB_P52B               ((agc_job_id_t)0x0A27)
#define AGC_JOB_P52_OPTION         ((agc_job_id_t)0x0A28)
#define AGC_JOB_P52T               ((agc_job_id_t)0x0A29)
#define AGC_JOB_P52T_PROCEED       ((agc_job_id_t)0x0A2A)
#define AGC_JOB_P52_S52_3          ((agc_job_id_t)0x0A2B)
#define AGC_JOB_P52D               ((agc_job_id_t)0x0A2C)
#define AGC_JOB_COARSTYP           ((agc_job_id_t)0x0A2D)
#define AGC_JOB_CAL53A             ((agc_job_id_t)0x0A2E)
#define AGC_JOB_CAL53A_COARSE_DONE ((agc_job_id_t)0x0A2F)
#define AGC_JOB_CAL53A_FINE_DONE   ((agc_job_id_t)0x0A30)
#define AGC_JOB_GYCRS              ((agc_job_id_t)0x0A31)
#define AGC_JOB_GYCRS_PULSED       ((agc_job_id_t)0x0A32)
#define AGC_JOB_P52C               ((agc_job_id_t)0x0A33)
#define AGC_JOB_P52C_PROCEED       ((agc_job_id_t)0x0A34)
#define AGC_JOB_P52F               ((agc_job_id_t)0x0A35)
#define AGC_JOB_P52LS              ((agc_job_id_t)0x0A36)
#define AGC_JOB_LSDISP             ((agc_job_id_t)0x0A37)
#define AGC_JOB_LSDISP_PROCEED     ((agc_job_id_t)0x0A38)
#define AGC_JOB_P52LS_PREC         ((agc_job_id_t)0x0A39)
#define AGC_JOB_ENDP50S            ((agc_job_id_t)0x0A3A)
#define AGC_JOB_P51                ((agc_job_id_t)0x0A3B)
#define AGC_JOB_P51AA              ((agc_job_id_t)0x0A3C)
#define AGC_JOB_P51_COARSE         ((agc_job_id_t)0x0A3D)
#define AGC_JOB_P51_COARSE_DONE    ((agc_job_id_t)0x0A3E)
#define AGC_JOB_P51_FINE_DONE      ((agc_job_id_t)0x0A3F)
#define AGC_JOB_P51B               ((agc_job_id_t)0x0A40)
#define AGC_JOB_P51C               ((agc_job_id_t)0x0A41)
#define AGC_JOB_P51C2              ((agc_job_id_t)0x0A42)
#define AGC_JOB_P51E               ((agc_job_id_t)0x0A43)
#define AGC_JOB_P51F               ((agc_job_id_t)0x0A44)
#define AGC_JOB_P51G               ((agc_job_id_t)0x0A45)
#define AGC_JOB_R51                ((agc_job_id_t)0x0A46)
#define AGC_JOB_R51DSP             ((agc_job_id_t)0x0A47)
#define AGC_JOB_R51DSP_PROCEED     ((agc_job_id_t)0x0A48)
#define AGC_JOB_R51ST              ((agc_job_id_t)0x0A49)
#define AGC_JOB_R51A               ((agc_job_id_t)0x0A4A)
#define AGC_JOB_R51A_PLANET        ((agc_job_id_t)0x0A4B)
#define AGC_JOB_R51_AXISGEN        ((agc_job_id_t)0x0A4C)
#define AGC_JOB_R51K               ((agc_job_id_t)0x0A4D)
#define AGC_JOB_PLANET_FLASH       ((agc_job_id_t)0x0A4E)
#define AGC_JOB_PLANET_LOADED      ((agc_job_id_t)0x0A4F)
#define AGC_JOB_R52                ((agc_job_id_t)0x0A50)
#define AGC_JOB_R52C               ((agc_job_id_t)0x0A51)
#define AGC_JOB_R52F               ((agc_job_id_t)0x0A52)
#define AGC_JOB_R52_AFTER_DELAY    ((agc_job_id_t)0x0A53)
#define AGC_JOB_R52_R61_DONE       ((agc_job_id_t)0x0A54)
#define AGC_JOB_TERM52             ((agc_job_id_t)0x0A55)
#define AGC_JOB_R53JOB             ((agc_job_id_t)0x0A56)
#define AGC_JOB_R53A               ((agc_job_id_t)0x0A57)
#define AGC_JOB_R53_MARKED         ((agc_job_id_t)0x0A58)
#define AGC_JOB_R53C1              ((agc_job_id_t)0x0A59)
#define AGC_JOB_R53C               ((agc_job_id_t)0x0A5A)
#define AGC_JOB_R53Z               ((agc_job_id_t)0x0A5B)
#define AGC_JOB_R53D               ((agc_job_id_t)0x0A5C)
#define AGC_JOB_R56                ((agc_job_id_t)0x0A5D)
#define AGC_JOB_R56A               ((agc_job_id_t)0x0A5E)
#define AGC_JOB_R56A1              ((agc_job_id_t)0x0A5F)
#define AGC_JOB_R56_MARK           ((agc_job_id_t)0x0A60)
#define AGC_JOB_R56_RECYCLE        ((agc_job_id_t)0x0A61)
#define AGC_JOB_LM_P52T_PROCEED    ((agc_job_id_t)0x0A62)
#define AGC_JOB_LM_P52LS           ((agc_job_id_t)0x0A63)
#define AGC_JOB_N89DISP            ((agc_job_id_t)0x0A64)
#define AGC_JOB_N89DISP_PROCEED    ((agc_job_id_t)0x0A65)


#define AGC_JOB_LM_R51             ((agc_job_id_t)0x0A68)
#define AGC_JOB_LM_R51_LOCSAM      ((agc_job_id_t)0x0A69)
#define AGC_JOB_LM_R51E            ((agc_job_id_t)0x0A6A)
#define AGC_JOB_LM_R51_3           ((agc_job_id_t)0x0A6B)
#define AGC_JOB_LM_R51_MARKED      ((agc_job_id_t)0x0A6C)
#define AGC_JOB_LM_R51_PLANET      ((agc_job_id_t)0x0A6D)
#define AGC_JOB_LM_R51_4           ((agc_job_id_t)0x0A6E)
#define AGC_JOB_LM_R51K            ((agc_job_id_t)0x0A6F)
#define AGC_JOB_LM_R52             ((agc_job_id_t)0x0A70)
#define AGC_JOB_LM_R52A            ((agc_job_id_t)0x0A71)
#define AGC_JOB_LM_R52B            ((agc_job_id_t)0x0A72)
#define AGC_JOB_GETAZEL            ((agc_job_id_t)0x0A73)
#define AGC_JOB_AZEL_KEYED         ((agc_job_id_t)0x0A74)
#define AGC_JOB_LM_R52_PLANET      ((agc_job_id_t)0x0A75)
#define AGC_JOB_LM_R52_R60DONE     ((agc_job_id_t)0x0A76)
#define AGC_JOB_LM_P51             ((agc_job_id_t)0x0A77)
#define AGC_JOB_LM_P51_PERF        ((agc_job_id_t)0x0A78)
#define AGC_JOB_LM_P51_COARSE      ((agc_job_id_t)0x0A79)
#define AGC_JOB_LM_P51_COARSED     ((agc_job_id_t)0x0A7A)
#define AGC_JOB_LM_P51B            ((agc_job_id_t)0x0A7B)
#define AGC_JOB_LM_P51C            ((agc_job_id_t)0x0A7C)
#define AGC_JOB_LM_P51_MARKED      ((agc_job_id_t)0x0A7D)
#define AGC_JOB_LM_P51_PLANET      ((agc_job_id_t)0x0A7E)
#define AGC_JOB_LM_P51E            ((agc_job_id_t)0x0A7F)
#define AGC_JOB_LM_P51G            ((agc_job_id_t)0x0A80)
#define AGC_JOB_R59                ((agc_job_id_t)0x0A81)
#define AGC_JOB_R59A               ((agc_job_id_t)0x0A82)
#define AGC_JOB_R59E               ((agc_job_id_t)0x0A83)
#define AGC_JOB_R59OUT             ((agc_job_id_t)0x0A84)
#define AGC_JOB_R59RET             ((agc_job_id_t)0x0A85)
#define AGC_JOB_P57                ((agc_job_id_t)0x0A86)
#define AGC_JOB_P57OPT             ((agc_job_id_t)0x0A87)
#define AGC_JOB_ALIGNOPT           ((agc_job_id_t)0x0A88)
#define AGC_JOB_P57AA              ((agc_job_id_t)0x0A89)
#define AGC_JOB_P57AA_PROCEED      ((agc_job_id_t)0x0A8A)
#define AGC_JOB_P57D_DONE          ((agc_job_id_t)0x0A8B)
#define AGC_JOB_PACKOPTN           ((agc_job_id_t)0x0A8C)
#define AGC_JOB_DSPOPTN            ((agc_job_id_t)0x0A8D)
#define AGC_JOB_DSPOPTN_PROCEED    ((agc_job_id_t)0x0A8E)
#define AGC_JOB_GVDETER            ((agc_job_id_t)0x0A8F)
#define AGC_JOB_GRAVEL             ((agc_job_id_t)0x0A90)
#define AGC_JOB_GVDETER_LUNG2      ((agc_job_id_t)0x0A91)
#define AGC_JOB_PROGRAV            ((agc_job_id_t)0x0A92)
#define AGC_JOB_PROGRAV_RECYCLE    ((agc_job_id_t)0x0A93)
#define AGC_JOB_LUNG               ((agc_job_id_t)0x0A94)
#define AGC_JOB_LUNG_COARSE_DONE   ((agc_job_id_t)0x0A95)
#define AGC_JOB_LUNG_FINE_DONE     ((agc_job_id_t)0x0A96)
#define AGC_JOB_ADDGRAV            ((agc_job_id_t)0x0A97)
#define AGC_JOB_ATTCHK             ((agc_job_id_t)0x0A98)
#define AGC_JOB_P57JUMP            ((agc_job_id_t)0x0A99)
#define AGC_JOB_SURFLINE           ((agc_job_id_t)0x0A9A)
#define AGC_JOB_SURFLINE_R54       ((agc_job_id_t)0x0A9B)
#define AGC_JOB_5DEGTEST           ((agc_job_id_t)0x0A9C)
#define AGC_JOB_SURFDISP           ((agc_job_id_t)0x0A9D)
#define AGC_JOB_P57POST            ((agc_job_id_t)0x0A9E)
#define AGC_JOB_P57POST_PROC       ((agc_job_id_t)0x0A9F)
#define AGC_JOB_P57POST_LS         ((agc_job_id_t)0x0AA0)
#define AGC_JOB_COATRIM            ((agc_job_id_t)0x0AA1)
#define AGC_JOB_CORSIT             ((agc_job_id_t)0x0AA2)
#define AGC_JOB_CORSIT_COARSE_DONE ((agc_job_id_t)0x0AA3)
#define AGC_JOB_CORSIT_FINE_DONE   ((agc_job_id_t)0x0AA4)
#define AGC_JOB_GYROTRIM           ((agc_job_id_t)0x0AA5)
#define AGC_JOB_JUSTTRIM           ((agc_job_id_t)0x0AA6)
#define AGC_JOB_JUSTTRIM_FINE_DONE ((agc_job_id_t)0x0AA7)
#define AGC_JOB_JUSTTRIM_PULSED    ((agc_job_id_t)0x0AA8)
#define AGC_JOB_2STARS             ((agc_job_id_t)0x0AA9)
#define AGC_JOB_1STAR              ((agc_job_id_t)0x0AAA)

#define AGC_JOB_CHKSDATA_FLASH        ((agc_job_id_t)0x0AAE)

#define AGC_JOB_ALIGN_COARSE_DONE     ((agc_job_id_t)0x0AAF)
#define AGC_JOB_ALIGN_FINE_DONE       ((agc_job_id_t)0x0AB0)

#define AGC_JOB_P52T_FLASH            ((agc_job_id_t)0x0AB1)
#define AGC_JOB_CAL53A_COARSED        ((agc_job_id_t)0x0AB2)
#define AGC_JOB_CAL53A_FINEONLY       ((agc_job_id_t)0x0AB3)
#define AGC_JOB_GYCRS_PULSE           ((agc_job_id_t)0x0AB4)
#define AGC_JOB_GYCRS_TORQUED         ((agc_job_id_t)0x0AB5)

#define AGC_TASK_MARKDIF           ((agc_task_id_t)0x0A01) /* MARKDIF (CM)   */
#define AGC_TASK_ENDMARKS          ((agc_task_id_t)0x0A02) /* ENDMARKS (CM)  */
#define AGC_TASK_AOT_MKRELEAS      ((agc_task_id_t)0x0A03) /* MKRELEAS (LM)  */
#define AGC_TASK_GRABGRAV          ((agc_task_id_t)0x0A04) /* GRABGRAV (LM)  */

#endif /* ALIGN_PENDING_H */
