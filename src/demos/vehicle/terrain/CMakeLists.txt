#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle terrain models.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all demos

set(DEMOS "")

if(CH_ENABLE_MODULE_IRRLICHT OR CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_VEH_RigidTerrain_WheeledVehicle
        demo_VEH_RigidTerrain_MovingPatch
        demo_VEH_SCMTerrain_WheeledVehicle
        demo_VEH_SCMTerrain_RigidTire
        demo_VEH_SCMTerrain_TrackedVehicle
    )
endif()

if(HAVE_OPENCRG) 
   if (CH_ENABLE_MODULE_IRRLICHT)
      set(DEMOS ${DEMOS}
          demo_VEH_CRGTerrain_IRR
      )
   endif()

   if(CH_ENABLE_MODULE_VSG)
      set(DEMOS ${DEMOS}
          demo_VEH_CRGTerrain_VSG
    )
   endif()
endif()

if(CH_ENABLE_MODULE_MULTICORE AND CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_VEH_GranularTerrain_RigidTire
        demo_VEH_GranularTerrain_MovingPatch
    )
endif()

if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_VEH_CRMTerrain_WheeledVehicle
        demo_VEH_CRMTerrain_TrackedVehicle
    )
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL AND CH_ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
        demo_VEH_SCMTerrain_FEATire
    )
endif()

#--------------------------------------------------------------

set(UTILS "../../SetChronoSolver.h")

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")
list(APPEND LIBS "ChronoModels_robot")

if(CH_ENABLE_MODULE_MULTICORE)
    list(APPEND LIBS "Chrono_multicore")
endif()

if(CH_ENABLE_MODULE_FSI)
    list(APPEND LIBS "Chrono_fsisph")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    list(APPEND LIBS "Chrono_mumps")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
    if(CH_ENABLE_MODULE_FSI)
      list(APPEND LIBS "Chrono_fsisph_vsg")
    endif()
endif()

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp ${UTILS})
    source_group("" FILES ${PROGRAM}.cpp ${UTILS})

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
