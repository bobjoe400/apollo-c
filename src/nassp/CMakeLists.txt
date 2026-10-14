# apollo_c_agc: the C flight software (libagc), the src/sim clock and
# interrupt driver, and the NASSP engine shim, as one static library that
# NASSP's vessel DLLs link instead of compiling yaAGC (src_sys/yaAGC).
# Added to NASSP's build by sim/nassp/patches/0001-apollo-c-agc-backend.patch
# (configure NASSP with -DAPOLLO_C_DIR=<this checkout>); NASSP_YAAGC_DIR is
# NASSP's src_sys/yaAGC, whose agc_engine.h the shim implements.
cmake_minimum_required(VERSION 3.19)

get_filename_component(APOLLO_C_ROOT ${CMAKE_CURRENT_LIST_DIR}/../.. ABSOLUTE)

file(GLOB_RECURSE APOLLO_C_LIB_SRCS ${APOLLO_C_ROOT}/src/*.c)
list(FILTER APOLLO_C_LIB_SRCS EXCLUDE REGEX "/src/(sim|nassp)/")

add_library(apollo_c_agc STATIC
    ${APOLLO_C_LIB_SRCS}
    ${APOLLO_C_ROOT}/src/sim/clock.c
    ${APOLLO_C_ROOT}/src/sim/channels.c
    ${APOLLO_C_ROOT}/src/sim/counters.c
    ${APOLLO_C_ROOT}/src/sim/rope.c
    ${APOLLO_C_ROOT}/src/nassp/engine.c
)
set_target_properties(apollo_c_agc PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
target_include_directories(apollo_c_agc PRIVATE ${APOLLO_C_ROOT}/include ${NASSP_YAAGC_DIR})
# -DAPOLLO_C_ASSERT_REPORTS=ON: failed AGC_ASSERTs print file:line to stderr
# (apollo-c-agc-stderr.log next to Orbiter), as the Linux debug library does.
option(APOLLO_C_ASSERT_REPORTS "report failed AGC_ASSERTs on stderr" OFF)
if(APOLLO_C_ASSERT_REPORTS)
    target_compile_definitions(apollo_c_agc PRIVATE AGC_HOST_DEBUG)
endif()
if(MSVC)
    # C4996: fopen etc. (rope.c reads the rope file NASSP names).
    target_compile_definitions(apollo_c_agc PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
