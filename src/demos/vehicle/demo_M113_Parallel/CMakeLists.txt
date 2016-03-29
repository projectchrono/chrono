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

SET(MODEL_FILES
    ../m113/M113_Sprocket.h
    ../m113/M113_Sprocket.cpp
    ../m113/M113_Idler.h
    ../m113/M113_Idler.cpp
    ../m113/M113_BrakeSimple.h
    ../m113/M113_RoadWheel.h
    ../m113/M113_RoadWheel.cpp
    ../m113/M113_Suspension.h
    ../m113/M113_Suspension.cpp
    ../m113/M113_TrackAssembly.h
    ../m113/M113_TrackAssembly.cpp
    ../m113/M113_TrackShoe.h
    ../m113/M113_TrackShoe.cpp
    ../m113/M113_SimpleDriveline.h
    ../m113/M113_SimpleDriveline.cpp
	  ../m113/M113_DrivelineBDS.h
    ../m113/M113_DrivelineBDS.cpp
    ../m113/M113_SimplePowertrain.h
    ../m113/M113_SimplePowertrain.cpp
    ../m113/M113_ShaftsPowertrain.h
    ../m113/M113_ShaftsPowertrain.cpp
    ../m113/M113_Vehicle.h
    ../m113/M113_Vehicle.cpp
)

SET(DEMO
    demo_VEH_M113_Parallel
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
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
                      ChronoEngine_opengl)
INSTALL(TARGETS ${DEMO} DESTINATION bin)
