/*
 * agc_status.h -- status codes returned by every non-void apollo-c function
 * that can fail (Power of Ten rule 7).
 */
#ifndef AGC_STATUS_H
#define AGC_STATUS_H

typedef enum agc_status
{
    AGC_STATUS_OK = 0,
    AGC_STATUS_BAD_ARG,     /* parameter validation failed            */
    AGC_STATUS_OVERFLOW,    /* fixed-point overflow where AGC aborts   */
    AGC_STATUS_NO_CONVERGE, /* iteration bound hit (Kepler, Lambert)   */
    AGC_STATUS_ALARM,       /* an alarm was raised (non-fatal)         */
    AGC_STATUS_BAILOUT,     /* BAILOUT: software restart requested     */
    AGC_STATUS_POODOO,      /* POODOO: program aborted to P00          */
    AGC_STATUS_FULL,        /* table full (jobs, VACs, waitlist)       */
    AGC_STATUS_NOT_FOUND,
    AGC_STATUS_BUSY,
    AGC_STATUS_UNIMPLEMENTED /* translation stub; see docs/MODULE_STATUS */
} agc_status_t;

#endif /* AGC_STATUS_H */
