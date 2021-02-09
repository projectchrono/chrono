#=============================================================================
# CMake configuration file for the test rig with continuous band track demo.
# This example program requires Irrlicht run-time visualization and the
# PardisoMKL or MUMPS Chrono modules.
#=============================================================================

# Irrlicht required
if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

# MUMPS or MKL required
if (NOT ENABLE_MODULE_PARDISO_MKL AND NOT ENABLE_MODULE_MUMPS)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(DEMOS
    demo_VEH_TrackTestRig_Band
)

#--------------------------------------------------------------
# Compiler and linker flags

include_directories(${CH_IRRLICHTINC})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
set(LIBRARIES 
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoEngine_vehicle_irrlicht
    ChronoModels_vehicle
    )

if (ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    set(LIBRARIES ${LIBRARIES} ChronoEngine_pardisomkl)
endif()

if (ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_mumps)
endif()

#--------------------------------------------------------------
# Add executables

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")
    
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES
                          FOLDER demos 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
