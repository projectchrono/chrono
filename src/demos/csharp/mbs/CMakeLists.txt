set(DEMOS
    demo_CS_MBS_ballSMC
    demo_CS_MBS_earthquake
    demo_CS_MBS_link_bushing
)

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")
if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND MODULE_LIST "irrlicht")
endif()

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
  # set dependencies, so latest SWIG always builds first
  foreach(module IN LISTS MODULE_LIST)
    add_dependencies(${PROGRAM} "Chrono_csharp_${module}")
  endforeach()
endforeach()
