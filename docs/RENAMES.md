
## dap-lm (renamed by dap-lm itself)

| Old identifier | New identifier | Cross-lane callers to update |
|---|---|---|
| agc_lm_dap_1_accs | agc_lm_dap_jet_accel_update | src/lm/servicer_lm.c:370 |
| agc_job_1_accjob / AGC_JOB_1_ACCJOB | agc_job_jet_accel_update / AGC_JOB_JET_ACCEL_UPDATE | dispatch.c (when registered) |
| agc_job_1_accset / AGC_JOB_1_ACCSET | agc_job_jet_accel_reset / AGC_JOB_JET_ACCEL_RESET | dispatch.c (when registered) |
| lm_dap_1_accont, lm_dap_1sto2s, lm_dap_pjetslec6, lm_dap_purgency4 (lane-private) | lm_dap_switch_curves, lm_dap_ones_to_twos, lm_dap_pjetslec_pair, lm_dap_purgency_law | none |
