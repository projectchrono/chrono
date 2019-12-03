#=============================================================================
# CMake configuration file for the SUSPENSION_TEST_RIG demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO demo_VEH_TrackTestRig)
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Compiler and linker flags

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "")

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_mkl")
endif()

if (ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    list(APPEND LIBS "ChronoEngine_mumps")
endif()

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
TARGET_LINK_LIBRARIES(${DEMO} 
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle
                      ChronoModels_vehicle
                      ${LIBS})
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

