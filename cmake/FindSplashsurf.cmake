# ==============================================================================
# FindSplashsurf
#
# If found, set the following variables:
#  SPLASHSURF_FOUND - system has Splashsurf
#  SPLASHSURF_EXECUTABLE - the Splashsurf executable
#  SPLASHSURF_VERSION_STRING - the version of Splashsurf found
# ==============================================================================

find_program(SPLASHSURF_EXECUTABLE  splashsurf)

if(SPLASHSURF_EXECUTABLE)
    execute_process(COMMAND "${SPLASHSURF_EXECUTABLE}" -V
                  OUTPUT_VARIABLE SPLASHSURF_OUTPUT_VARIABLE
                  ERROR_QUIET
                  OUTPUT_STRIP_TRAILING_WHITESPACE)

    string(REGEX REPLACE "^splashsurf ([0-9\\.]+)( patchlevel )?" "\\1." SPLASHSURF_VERSION_STRING "${SPLASHSURF_OUTPUT_VARIABLE}")
    string(REGEX REPLACE "\\.$" "" SPLASHSURF_VERSION_STRING "${SPLASHSURF_VERSION_STRING}")
    unset(SPLASHSURF_OUTPUT_VARIABLE)

    include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Splashsurf
                                      REQUIRED_VARS SPLASHSURF_EXECUTABLE
                                      VERSION_VAR SPLASHSURF_VERSION_STRING)

    mark_as_advanced(SPLASHSURF_EXECUTABLE)
endif()
