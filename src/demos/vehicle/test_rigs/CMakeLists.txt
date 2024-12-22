#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle test rigs.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_TireTestRig
)

if(ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_VEH_SuspensionTestRig
       demo_VEH_TrackTestRig
   )
   if(ENABLE_MODULE_PARDISO_MKL OR ENABLE_MODULE_MUMPS)
      set(DEMOS ${DEMOS}
          demo_VEH_TrackTestRig_Band
      )
   endif()
endif()

#--------------------------------------------------------------

set(UTILS "../../SetChronoSolver.h")

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()


if(ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    list(APPEND LIBS "ChronoEngine_mumps")
endif()

if(ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "ChronoEngine_postprocess")
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
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
