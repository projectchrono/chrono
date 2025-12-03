set(DEMOS
    demo_CS_CH_buildsystem
)

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")

set(CHRONO_CSHARP_SOURCES)
set(_chrono_csharp_source_desc "")
# Prefer the managed wrapper when available so each demo stays tiny; otherwise embed generated SWIG sources directly (original approach)
if(CHRONO_CSHARP_USING_WRAPPER)
  set(_chrono_csharp_source_desc "using chrono_csharp_wrapper.dll")
else()
  CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")
  list(LENGTH CHRONO_CSHARP_SOURCES NUM_CSHARP_FILES)
  set(_chrono_csharp_source_desc "embedding ${NUM_CSHARP_FILES} SWIG sources")
endif()

message(STATUS "  C# core demos;    ${_chrono_csharp_source_desc}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
  chrono_configure_csharp_demo(${PROGRAM})
endforeach()
