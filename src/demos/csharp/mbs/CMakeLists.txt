if(NOT (CH_ENABLE_MODULE_IRRLICHT OR CH_ENABLE_MODULE_VSG))
  message(STATUS "Warning: the MBS C# projects require Chrono::Irrlicht or Chrono::VSG")
  return()
endif()

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
if(CH_ENABLE_MODULE_VSG)
  list(APPEND MODULE_LIST "vsg")
endif()

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")
list(LENGTH CHRONO_CSHARP_SOURCES NUM_CSHARP_FILES)

message(STATUS "  C# mbs demos;     number C# sources: ${NUM_CSHARP_FILES}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
  
  # Add preprocessor defines based on available visual modules
  if(CH_ENABLE_MODULE_IRRLICHT)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DOTNET_REFERENCES_COPY_LOCAL TRUE)
    target_compile_options(${PROGRAM} PRIVATE "/define:CHRONO_IRRLICHT")
  endif()
  if(CH_ENABLE_MODULE_VSG)
    target_compile_options(${PROGRAM} PRIVATE "/define:CHRONO_VSG")
  endif()
endforeach()
