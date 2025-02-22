set(DEMOS
    demo_CS_CH_buildsystem
)

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
endforeach()
