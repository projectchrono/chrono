#=============================================================================
# CMake configuration file for the SUSPENSION_TEST_RIG demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

MESSAGE(STATUS "...add TRACK_TEST_RIG demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../m113/M113_Sprocket.h
    ../m113/M113_Sprocket.cpp
    ../m113/M113_Idler.h
    ../m113/M113_Idler.cpp
    ../m113/M113_RoadWheel.h
    ../m113/M113_RoadWheel.cpp
    ../m113/M113_Suspension.h
    ../m113/M113_Suspension.cpp
    ../m113/M113_TrackAssembly.h
    ../m113/M113_TrackAssembly.cpp
    ../m113/M113_TrackShoe.h
    ../m113/M113_TrackShoe.cpp
)

SET(DEMO_FILES
    demo_TrackTestRig.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Add executable

ADD_EXECUTABLE(demo_TrackTestRig ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_TrackTestRig PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_TrackTestRig 
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle)
INSTALL(TARGETS demo_TrackTestRig DESTINATION bin)

