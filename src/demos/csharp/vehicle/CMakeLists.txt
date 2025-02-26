if(NOT CH_ENABLE_MODULE_VEHICLE)
  return()
endif()

set(DEMOS
    demo_CS_VEH_WheeledJSON
    demo_CS_VEH_Kraz_OpenLoop
    demo_CS_VEH_Kraz_ClosedLoop
    demo_CS_VEH_UAZBUS
    demo_CS_VEH_HMMWV_Rollover
    demo_CS_VEH_Gator_Accel
    demo_CS_VEH_paths
    demo_CS_VEH_RigidTerrain_WheeledVehicle
    demo_CS_VEH_HMMWV_MeshTerrain
)

set(DEMOS_OPENCRG
    demo_CS_VEH_CRGTerrain_IRR
)

if(HAVE_OPENCRG)
  list(APPEND DEMOS ${DEMOS_OPENCRG})
endif()

# ----------------------------------------------------------------------

set(MODULE_LIST)
list(APPEND MODULE_LIST "core")
list(APPEND MODULE_LIST "vehicle")
if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND MODULE_LIST "irrlicht")
endif()
if(CH_ENABLE_MODULE_POSTPROCESS)
  list(APPEND MODULE_LIST "postprocess")
endif()

CollectCSharpFiles(CHRONO_CSHARP_SOURCES "${MODULE_LIST}")

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
  # set dependencies, so latest SWIG always builds first
  foreach(module IN LISTS MODULE_LIST)
    add_dependencies(${PROGRAM} "Chrono_csharp_${module}")
  endforeach()
endforeach()
