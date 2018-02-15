#=============================================================================
# CMake configuration file for the VEHICLE demo - example programs for using
# a tracked vehicle model with continuous-band track, specified entirely
# through JSON specification files.
# These example programs requires Irrlicht run-time visualization and the
# FEA and MKL Chrono modules.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_MKL)
    RETURN()
endif()

#--------------------------------------------------------------
# List all demo programs

set(DEMOS
    demo_VEH_TrackedJSON_Band
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

    add_executable(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
    source_group("" FILES ${DEMO}.cpp)
    set_target_properties(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${DEMO} ${LIBRARIES})
    install(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

endforeach()
