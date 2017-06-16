#=============================================================================
# CMake configuration file for the ARTICULATED_VEHICLE demo.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    subsystems/Articulated_Front.h
    subsystems/Articulated_Front.cpp
    subsystems/Articulated_Rear.h
    subsystems/Articulated_Rear.cpp
)

SET(DEMO
    demo_VEH_ArticulatedVehicle
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp ${MODEL_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})
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
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
