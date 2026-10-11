# nav-integ oracle cases (Comanche055): one precision integration through
# the real INTEGRVS ... INTEXIT, and RECTIFY alone.
#
# Integration: entry INTPRET with Q = INTEGRVS (bank 13).  INTEGRVS stores
# QPRET in IRETURN, rectifies (TDELTAV = TNUV = 0), ALOADED -> TESTLOOP:
# NORFINAL picks the step; with TDEC1 - TET below the maximum step the
# whole interval is one Nystrom step (three ACCOMP stages: GAMCOMP primary
# term + OBLATE, earth J2..J4 or lunar J2/J3 + QUALITY1-3; no third bodies
# because |R| < RME / RMM leaves MIDFLAG clear).  The next TESTLOOP finds
# DT/2 < DT/2MIN -> A-PCHK -> RECTOUT (RECTIFY, push list) -> INTEXIT,
# which ends with TC INTWAKE: the stop.  Several steps when TDEC1 is far.
# Inputs: RCV VCV (E 2^29 m / 2^7 m/cs, M 2^27 / 2^5), TET, TDEC1 =
# push-down 32D (VAC1+32), cs 2^28; FLAGWRD0 = 04000 (MOONFLAG) for the
# moon; E3J22R2M = 5232d, E32C31RM = 1465d (lunar pad loads, as in
# tests/nav/integration.c); every other erasable 0 (ephemeris pad, MODREG,
# 504LM, TEPHEM).
# Outputs: push list RATT 0D VATT 6D TAT 12D RATT1 14D VATT1 20D MU(P) 26D,
# RRECT VRECT TET, FV (last ACCOMP: stage 2 of the last step), DT/2.
#
# RECTIFY: entry INTPRET, Q = RECTIFY (bank 11); return address QPRET =
# VAC1+42 = interpretive address of ABTCONIC (12,3751 -> 25751, EXIT / TC
# POODOO): stop POODOO.  Inputs RCV VCV TDELTAV (E 2^22 M 2^18) TNUV
# (E 2^3 M 2^-1) TC XKEP PBODY (0 / 2).
rope Comanche055
entry INTPRET
stop INTWAKE
maxcycles 20000000
ebank 3
get VAC1:dp
get VAC1+2:dp
get VAC1+4:dp
get VAC1+6:dp
get VAC1+8:dp
get VAC1+10:dp
get VAC1+12:dp
get VAC1+14:dp
get VAC1+16:dp
get VAC1+18:dp
get VAC1+20:dp
get VAC1+22:dp
get VAC1+24:dp
get VAC1+26:dp
get RRECT:dp
get RRECT+2:dp
get RRECT+4:dp
get VRECT:dp
get VRECT+2:dp
get VRECT+4:dp
get TET:dp
get FV:dp
get FV+2:dp
get FV+4:dp
get DT/2:dp
get FLAGWRD0
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TET=0,0
set VAC1+32=00000,05670
run integ_earth_1step
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set RCV=00077,02755,77554,71054,00210,26326
set VCV=21330,07470,07452,15714,00100,01470
set TET=00001,00000
set VAC1+32=00001,05670
run integ_earth2_1step
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TET=0,0
set VAC1+32=00006,03240
run integ_earth_multi
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set RCV=00161,17370,00236,34050,00161,17370
set VCV=62154,44226,10326,12425,02001,14746
set TET=0,0
set FLAGWRD0=04000
set E3J22R2M=5232d
set E32C31RM=1465d
set VAC1+32=00000,05670
run integ_moon_1step
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set RCV=00161,17370,00236,34050,00161,17370
set VCV=62154,44226,10326,12425,02001,14746
set TET=0,0
set FLAGWRD0=04000
set E3J22R2M=5232d
set E32C31RM=1465d
set VAC1+32=00006,03240
run integ_moon_multi
# ---- RECTIFY alone (bank 11), return to ABTCONIC -> stop POODOO --------
stop POODOO
q RECTIFY
fbank 11
get RCV:dp
get RCV+2:dp
get RCV+4:dp
get VCV:dp
get VCV+2:dp
get VCV+4:dp
get TDELTAV:dp
get TDELTAV+2:dp
get TDELTAV+4:dp
get TNUV:dp
get TNUV+2:dp
get TNUV+4:dp
get TC:dp
get XKEP:dp
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC1+42=25751
set PBODY=0
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TDELTAV=00004,26040,77774,73757,00001,05410
set TNUV=02000,00000,77146,54631,00314,31463
set TC=00000,05670
set XKEP=00123,04567
run rectify_earth
q RECTIFY
fbank 11
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC1+42=25751
set PBODY=2
set RCV=00161,17370,00236,34050,00161,17370
set VCV=62154,44226,10326,12425,02001,14746
set TDELTAV=00006,11000,77774,73377,00001,10200
set TNUV=03146,14632,76560,64365,00507,25605
set TC=00000,05670
set XKEP=00123,04567
run rectify_moon
q RECTIFY
fbank 11
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC1+42=25751
set PBODY=0
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TDELTAV=77035,67722,00000,00240,00000,00000
set TNUV=02000,00000,77146,54631,00314,31463
run rectify_earth_neg
# ---- TIMESTEP state before RECTOUT ---------------------------------------
# Same INTEGRVS runs with STATEFLG and VINTFLAG set (FLAGWRD3 = 00024):
# A-PCHK -> ANDOUT clears STATEFLG and EXITs to TC PHASCHNG (the stop),
# before ATOPCSM / RECTOUT, so TDELTAV TNUV RCV VCV TC XKEP TET are the
# Encke state after the last Nystrom step (C: the permanent CSM state
# written by ATOPCSM).  TDEC1 = 24064 cs = one maximum step for this
# orbit (NORFINAL), 48128 cs = two steps; moon 3000 cs = one step.
stop PHASCHNG
q INTEGRVS
fbank 13
get TDELTAV:dp
get TDELTAV+2:dp
get TDELTAV+4:dp
get TNUV:dp
get TNUV+2:dp
get TNUV+4:dp
get RCV:dp
get RCV+2:dp
get RCV+4:dp
get VCV:dp
get VCV+2:dp
get VCV+4:dp
get TC:dp
get XKEP:dp
get TET:dp
get FV:dp
get FV+2:dp
get FV+4:dp
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set FLAGWRD3=00024
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TET=0,0
set VAC1+32=00001,17000
run step_earth_max
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set FLAGWRD3=00024
set RCV=00172,10710,00141,32240,00202,15600
set VCV=63672,47725,17014,34525,00067,27047
set TET=0,0
set VAC1+32=00002,36000
run step_earth_2max
q INTEGRVS
fbank 13
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set FLAGWRD3=00024
set FLAGWRD0=04000
set E3J22R2M=5232d
set E32C31RM=1465d
set RCV=00161,17370,00236,34050,00161,17370
set VCV=62154,44226,10326,12425,02001,14746
set TET=0,0
set VAC1+32=00000,05670
run step_moon
