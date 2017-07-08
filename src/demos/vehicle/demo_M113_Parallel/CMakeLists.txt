#=============================================================================
# CMake configuration file for the M113_PARALLEL demo.
# This example program requires Chrono:Parallel and the OpenGL module for
# run-time visualization
#=============================================================================

#--------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_OPENGL OR NOT ENABLE_MODULE_PARALLEL)
  RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO demo_VEH_M113_Parallel)
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES} ${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO}
                      ChronoEngine
                      ChronoEngine_vehicle
                      ChronoEngine_parallel
                      ChronoEngine_opengl
                      ChronoModels_vehicle)
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
