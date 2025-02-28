#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle models.
# These example programs require run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_HMMWV_Rollover
    demo_VEH_Gator_Incline
    demo_VEH_WheeledGeneric
    demo_VEH_WheeledGenericSet
    demo_VEH_Wheeled
    demo_VEH_WheeledJSON
)

if(CH_ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_VEH_HMMWV9_YUP
       demo_VEH_Sedan_Lockable_Diff
   )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
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

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    if(PROGRAM STREQUAL "demo_VEH_Wheeled")
      set(MODELS ../WheeledVehicleModels.h)
    elseif(PROGRAM STREQUAL "demo_VEH_WheeledJSON")
      set(MODELS ../WheeledVehicleJSON.h)
    else()
      set(MODELS "")
    endif()

    add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODELS})
    source_group("" FILES ${PROGRAM}.cpp ${MODELS})

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
