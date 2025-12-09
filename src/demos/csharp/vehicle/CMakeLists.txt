if(NOT CH_ENABLE_MODULE_VEHICLE)
  return()
endif()

set(DEMOS "")

if(CH_ENABLE_MODULE_IRRLICHT OR CH_ENABLE_MODULE_VSG)
  set(DEMOS ${DEMOS}
      demo_CS_VEH_Kraz_OpenLoop
      demo_CS_VEH_HMMWV_MeshTerrain
  )
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
  set(DEMOS ${DEMOS}
      demo_CS_VEH_WheeledJSON
      demo_CS_VEH_UAZBUS
      demo_CS_VEH_HMMWV_Rollover
      demo_CS_VEH_Gator_Accel
      demo_CS_VEH_RigidTerrain_WheeledVehicle
  )

  if(CH_ENABLE_MODULE_POSTPROCESS)
    set(DEMOS ${DEMOS}
        demo_CS_VEH_Kraz_ClosedLoop
    )
  endif()

  if(HAVE_OPENCRG)
    set(DEMOS ${DEMOS}
        demo_CS_VEH_CRGTerrain_IRR
    )
  endif()

endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  set(DEMOS ${DEMOS}
      demo_CS_VEH_paths
  )
endif()

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")
list(APPEND MODULE_LIST "vehicle")
if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND MODULE_LIST "irrlicht")
endif()
if(CH_ENABLE_MODULE_VSG)
  list(APPEND MODULE_LIST "vsg")
endif()
if(CH_ENABLE_MODULE_POSTPROCESS)
  list(APPEND MODULE_LIST "postprocess")
endif()

set(CHRONO_CSHARP_SOURCES)
set(_chrono_csharp_source_desc "")
# Wrapper DLL simplifies references; fallback embeds all needed module sources when the wrapper is disabled so demos still compile as standalone
if(CHRONO_CSHARP_USING_WRAPPER)
  set(_chrono_csharp_source_desc "using chrono_csharp_wrapper.dll")
else()
  CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")
  list(LENGTH CHRONO_CSHARP_SOURCES NUM_CSHARP_FILES)
  set(_chrono_csharp_source_desc "embedding ${NUM_CSHARP_FILES} SWIG sources")
endif()

message(STATUS "  C# vehicle demos; ${_chrono_csharp_source_desc}")

# ----------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})

  # Applying specific reference dependencies for CRGTerrain demos (uses dynamic objects)
  if(HAVE_OPENCRG)
    if(WIN32)
      # Add reference to Microsoft.CSharp
      get_property(_existing_refs TARGET ${PROGRAM} PROPERTY VS_DOTNET_REFERENCES)
      if(NOT _existing_refs)
        set(_existing_refs "")
      endif()
      list(APPEND _existing_refs "Microsoft.CSharp")
      list(REMOVE_DUPLICATES _existing_refs)
      set_property(TARGET ${PROGRAM} PROPERTY VS_DOTNET_REFERENCES "${_existing_refs}")
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

  chrono_configure_csharp_demo(${PROGRAM})
endforeach()
