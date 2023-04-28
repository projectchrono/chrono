#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle models.
# These example programs require run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_CityBus
    demo_VEH_Gator
    demo_VEH_HMMWV
    demo_VEH_HMMWV_Accel
    demo_VEH_HMMWV_Rollover
    demo_VEH_HMMWV9
    demo_VEH_UAZBUS
    demo_VEH_U401
    demo_VEH_WheeledGeneric
    demo_VEH_WheeledJSON
)

if(ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_VEH_FEDA_Accel
       demo_VEH_Gator_Accel
       demo_VEH_HMMWV9_YUP
       demo_VEH_Kraz_ClosedLoop
       demo_VEH_Kraz_OpenLoop
       demo_VEH_LMTV_ride
       demo_VEH_MAN_5t
       demo_VEH_MAN_7t
       demo_VEH_MAN_10t
       demo_VEH_MROLE
       demo_VEH_MTV_ride
       demo_VEH_RCCar
       demo_VEH_Sedan
       demo_VEH_Sedan_Lockable_Diff
       demo_VEH_UAZBUS_SAE
   )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)

    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    list(APPEND LIBS "ChronoEngine_mumps")
endif()

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
