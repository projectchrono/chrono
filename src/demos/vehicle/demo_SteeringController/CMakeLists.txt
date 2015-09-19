#=============================================================================
# CMake configuration file for the STEERING_CONTROLLER demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
	MESSAGE(STATUS "---demo_SteeringController requires Irrlicht support")
	RETURN()
ENDIF()

MESSAGE(STATUS "...add STEERING_CONTROLLER demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../generic/Generic_PathFollowerDriver.h
    ../generic/Generic_PathFollowerDriver.cpp
)

SET(DEMO_FILES
    demo_SteeringController.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})


#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle)

#--------------------------------------------------------------
# Create the executable

ADD_EXECUTABLE(demo_SteeringController ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_SteeringController PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_SteeringController ${LIBRARIES})
INSTALL(TARGETS demo_SteeringController DESTINATION bin)
