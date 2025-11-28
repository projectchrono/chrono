if(NOT CH_ENABLE_MODULE_SENSOR)
  return()
endif()

set(DEMOS
  demo_CS_SEN_camera
)

# Demos that combine sensor simulation with vehicle dynamics
if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
    set(DEMOS ${DEMOS}
      #demo_SEN_Gator
    )
endif()

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")
list(APPEND MODULE_LIST "sensor")

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND MODULE_LIST "irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
  list(APPEND MODULE_LIST "vsg")
endif()

if(CH_ENABLE_MODULE_VEHICLE) 
    list(APPEND MODULE_LIST "vehicle")
endif()

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")
list(LENGTH CHRONO_CSHARP_SOURCES NUM_CSHARP_FILES)

message(STATUS "  C# sensor demos; number C# sources: ${NUM_CSHARP_FILES}")
#message("list: ${CHRONO_CSHARP_SOURCES}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})

  # Applying specific reference dependencies for CRGTerrain demos (uses dynamic objects)
  if(HAVE_OPENCRG)
    if(WIN32)
      # Add reference to Microsoft.CSharp
      set_property(TARGET ${PROGRAM} PROPERTY VS_DOTNET_REFERENCES "Microsoft.CSharp")
    else()
      # Assuming use of Mono on Unix/Linux (this is currently untested)
      find_package(Mono REQUIRED)
    endif()
  endif()
  
  # Add preprocessor defines based on available visualisation modules
  if(CH_ENABLE_MODULE_IRRLICHT)
    target_compile_options(${PROGRAM} PRIVATE "/define:CHRONO_IRRLICHT")
  endif()
  if(CH_ENABLE_MODULE_VSG)
    target_compile_options(${PROGRAM} PRIVATE "/define:CHRONO_VSG")
  endif()
endforeach()
