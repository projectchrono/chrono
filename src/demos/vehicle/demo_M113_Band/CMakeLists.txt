#=============================================================================
# CMake configuration file for the M113 with continuous band track demos.
# This example program works with and without Irrlicht run-time visualization
# and requires the MKL or MUMPS Chrono modules.
#=============================================================================

# MUMPS or MKL required. IRRLICHT required
if ((NOT ENABLE_MODULE_MKL AND NOT ENABLE_MODULE_MUMPS) OR NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

#--------------------------------------------------------------
# List all demo programs

set(DEMOS
    demo_VEH_M113_Band
#    demo_VEH_M113_Band_DefSoil
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

if (ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    set(LIBRARIES ${LIBRARIES} ChronoEngine_mkl)
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
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
