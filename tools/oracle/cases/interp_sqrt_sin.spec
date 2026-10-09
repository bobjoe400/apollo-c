# Interpretive unary ops SQRT, SIN, COS via the real INTPRET entry (INTERPRETER.agc).
#
# How the interpreter is entered: INTPRET does "QXCH LOC", i.e. LOC := Q = the
# address of the first interpretive word, then fetches opcode words with
# "INDEX LOC / CA 0".  Any erasable address works for LOC (this is how the
# flown Erasable Memory Programs ran interpretive code), so the snippet is
# poked into the unused VAC2 work area and Q is pointed at it.  EXIT does
# "INDEX LOC / TC 1", executing the basic word after EXIT from erasable; that
# word is "TC ENDOFJOB" (= the 12-bit address of ENDOFJOB), our stop label.
#
# Scaling (INTERPRETER.agc p.1082): SIN computes (1/2)sin(2*pi*MPAC),
# COS computes (1/2)cos(2*pi*MPAC); SQRT takes sqrt of DP MPAC in [0,1).
rope Luminary099
whatis VAC2USE
whatis VAC1
entry INTPRET
q VAC2USE
stop ENDOFJOB
maxcycles 20000
get MPAC:dp
get MPAC+2
get OVFIND
# interpreter job context: work area for push-down list, NEWJOB=+0 (no job change)
# snippet: SQRT, EXIT, TC ENDOFJOB    (opcode words from yaYUL: SQRT=77766 EXIT=77776)
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77766,77776,@ENDOFJOB
set MPAC=20000,00000,00000
set MODE=0
run sqrt_0.5
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77766,77776,@ENDOFJOB
set MPAC=10000,00000,00000
run sqrt_0.25
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77766,77776,@ENDOFJOB
set MPAC=00000,00001,00000
run sqrt_2e-28
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77766,77776,@ENDOFJOB
set MPAC=37777,37777,00000
run sqrt_max
# snippet: SIN, EXIT, TC ENDOFJOB   (SIN=77756)
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77756,77776,@ENDOFJOB
set MPAC=04000,00000,00000
run sin_1/8rev
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77756,77776,@ENDOFJOB
set MPAC=10000,00000,00000
run sin_1/4rev
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77756,77776,@ENDOFJOB
set MPAC=74000,00000,00000
run sin_74000                     # NB: octal 74000 is -2047/2^14 = -0.12494 rev, NOT -1/8
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77756,77776,@ENDOFJOB
setdp MPAC=-0.125                 # canonical -1/8 rev = 73777,77777
set MPAC+2=0
run sin_-1/8rev
# snippet: COS, EXIT, TC ENDOFJOB   (COS=77746)
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77746,77776,@ENDOFJOB
set MPAC=04000,00000,00000
run cos_1/8rev
set FIXLOC=@VAC1
set PUSHLOC=@VAC1
set VAC2USE=77746,77776,@ENDOFJOB
set MPAC=00000,00000,00000
run cos_0
