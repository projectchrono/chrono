#=============================================================================
# CMake configuration file for the M113_Multicore demo.
# This example program requires Chrono:Multicore and the OpenGL module for
# run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_OPENGL OR NOT ENABLE_MODULE_MULTICORE)
  RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(PROGRAM demo_VEH_M113_Multicore)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES} ${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM}
                      ChronoEngine
                      ChronoEngine_vehicle
                      ChronoEngine_multicore
                      ChronoEngine_opengl
                      ChronoModels_vehicle)

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
