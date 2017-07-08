#=============================================================================
# CMake configuration file for the TRACTOR_TRAILER demo.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    subsystems/TT_Tractor.h
    subsystems/TT_Tractor.cpp
    subsystems/TT_Chassis.h
    subsystems/TT_Chassis.cpp
    subsystems/TT_Trailer.h
    subsystems/TT_Trailer.cpp
)

SET(DEMO
    demo_VEH_TractorTrailer
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp)

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
