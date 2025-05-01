#=============================================================================
# CMake configuration file for the ISO2631 demos.
# These example programs work with and without Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all ISO2631 demos

set(DEMOS
    demo_VEH_Test_ISO2631_Ride
    demo_VEH_Test_ISO2631_Shock
    demo_VEH_Test_Accel
    demo_VEH_Test_FMTV_Ride
    demo_VEH_Test_TireFEA
)

if(HAVE_OPENCRG)
   if(CH_ENABLE_MODULE_VSG)
      set(DEMOS ${DEMOS}
          demo_VEH_Test_FEDA_Ride
          demo_VEH_Test_Cornering
   )
   endif()
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

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

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
endif()

#--------------------------------------------------------------
# Create the executables

set(MODELS ../WheeledVehicleModels.h)

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

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
