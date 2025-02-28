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
  # set dependencies, so latest SWIG always builds first
  foreach(module IN LISTS MODULE_LIST)
    add_dependencies(${PROGRAM} "Chrono_csharp_${module}")
  endforeach()
endforeach()
