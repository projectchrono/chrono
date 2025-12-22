#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle terrain models.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_FSI_SPH)
   return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS "")

if(CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_VEH_FSI_FloatingBlock
    )
endif()

#--------------------------------------------------------------

set(UTILS "../../SetChronoSolver.h")

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")
list(APPEND LIBS "Chrono_fsisph")

if(CH_ENABLE_MODULE_PARDISO_MKL)
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS "Chrono_mumps")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
    list(APPEND LIBS "Chrono_fsisph_vsg")
endif()

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp ${UTILS})
    source_group("" FILES ${PROGRAM}.cpp ${UTILS})

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
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
