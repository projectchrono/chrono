#=============================================================================
# CMake configuration file for the ARTICULATED_VEHICLE demo.
# This example program works with and without Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../articulated/Articulated_Vehicle.h
    ../articulated/Articulated_Vehicle.cpp
    ../articulated/Articulated_Chassis.h
    ../articulated/Articulated_Chassis.cpp
    ../articulated/Articulated_Trailer.h
    ../articulated/Articulated_Trailer.cpp
)

SET(DEMO
    demo_VEH_ArticulatedVehicle
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO}
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle
                      ChronoModels_vehicle)
INSTALL(TARGETS ${DEMO} DESTINATION bin)
