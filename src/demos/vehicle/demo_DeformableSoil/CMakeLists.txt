#=============================================================================
# CMake configuration file for the SUSPENSION_TEST_RIG demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

MESSAGE(STATUS "...add SUSPENSION_TEST_RIG demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO_FILES
    demo_DeformableSoil.cpp
)

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
    ChronoEngine_vehicle
    ${CH_IRRLICHTLIB}
)

#--------------------------------------------------------------
# Add executable

ADD_EXECUTABLE(demo_DeformableSoil ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_DeformableSoil PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_DeformableSoil ${LIBRARIES})
INSTALL(TARGETS demo_DeformableSoil DESTINATION bin)

