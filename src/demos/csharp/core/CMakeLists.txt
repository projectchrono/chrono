set(DEMOS
    demo_CS_CH_buildsystem
)

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")
list(LENGTH CHRONO_CSHARP_SOURCES NUM_CSHARP_FILES)

message(STATUS "  C# core demos;    number C# sources: ${NUM_CSHARP_FILES}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
endforeach()
