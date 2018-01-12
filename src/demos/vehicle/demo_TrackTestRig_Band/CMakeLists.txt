#=============================================================================
# CMake configuration file for the test rig with continuous band track demo.
# This example program works only with Irrlicht run-time visualization and 
# requires the FEA and MKL modules.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_MKL)
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
include_directories(${CH_MKL_INCLUDES})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
set(LIBRARIES 
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoModels_vehicle
    ChronoEngine_fea
    ChronoEngine_mkl)

#--------------------------------------------------------------
# Add executables

foreach(DEMO ${DEMOS})

    message(STATUS "...add ${DEMO}")
    
    add_executable(${DEMO} ${DEMO}.cpp)
    source_group("" FILES ${DEMO}.cpp)
    set_target_properties(${DEMO} PROPERTIES
                          FOLDER demos 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${DEMO} ${LIBRARIES})
    install(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

endforeach()
