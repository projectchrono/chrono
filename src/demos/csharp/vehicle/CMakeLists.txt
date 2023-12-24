set(DEMOS
    demo_VEH_WheeledJSON
    demo_VEH_Kraz_OpenLoop
    demo_VEH_Kraz_ClosedLoop
    #demo_VEH_TrackedJSON
    demo_VEH_UAZBUS
    demo_VEH_HMMWV_Rollover
    demo_VEH_Gator_Accel
    demo_VEH_paths
    demo_VEH_RigidTerrain_WheeledVehicle
    demo_VEH_CRGTerrain_IRR
)


message(STATUS "------------------------")
message(STATUS "C# Chrono::Vehicle demos")
message(STATUS "------------------------")

# Look for a Chrono installation
find_package(Chrono
             COMPONENTS CSharp Vehicle
             OPTIONAL_COMPONENTS Irrlicht Postprocess
             CONFIG)

# Append to the parent's lists of DLLs (and make them visible)
list(APPEND ALL_DLL_NAMES       "${CHRONO_DLL_NAMES}")
list(APPEND ALL_DEPENDENCY_DLLS "${CHRONO_DEPENDENCY_DLLS}")
set(ALL_DLL_NAMES       "${ALL_DLL_NAMES}"       PARENT_SCOPE)
set(ALL_DEPENDENCY_DLLS "${ALL_DEPENDENCY_DLLS}" PARENT_SCOPE)

# Add C# executables
foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
endforeach()

