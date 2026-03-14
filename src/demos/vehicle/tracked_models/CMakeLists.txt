#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle models.
#=============================================================================

if(NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_M113
    demo_VEH_M113_Band
    demo_VEH_Marder
    demo_VEH_TrackedJSON
    demo_VEH_TrackedJSON_Band
)

#--------------------------------------------------------------

set(UTILS "../../SetChronoSolver.h")

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")
list(APPEND LIBS "Chrono_vehicle_vsg")

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS "Chrono_mumps")
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
