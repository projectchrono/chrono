#=============================================================================
# CMake configuration file for the SUSPENSION_TEST demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
	RETURN()
ENDIF()

MESSAGE(STATUS "...add TIRE TEST RIG demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(TEST_FILES
    demo_TireTestRig.cpp
)

SOURCE_GROUP("" FILES ${TEST_FILES})

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

ADD_EXECUTABLE(demo_TireTestRig ${TEST_FILES})
SET_TARGET_PROPERTIES(demo_TireTestRig PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_TireTestRig ${LIBRARIES})
INSTALL(TARGETS demo_TireTestRig DESTINATION bin)
