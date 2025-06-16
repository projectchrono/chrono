#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle test rigs.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_TireTestRig
    demo_VEH_SuspensionTestRig
    demo_VEH_TrackTestRig
)

if(CH_ENABLE_MODULE_FSI)
    set(DEMOS ${DEMOS} demo_VEH_TireTestRig_CRM)
endif()

if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_PARDISO_MKL OR CH_ENABLE_MODULE_MUMPS)
    set(DEMOS ${DEMOS} demo_VEH_TrackTestRig_Band)
endif()

#--------------------------------------------------------------

set(UTILS "../../SetChronoSolver.h")

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_FSI)
    list(APPEND LIBS "Chrono_fsisph")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
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

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
