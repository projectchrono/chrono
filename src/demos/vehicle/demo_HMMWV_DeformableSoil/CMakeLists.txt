#=============================================================================
# CMake configuration file for the HMMWV_DEFORMABLE_SOIL demo.
# This example program requires Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

MESSAGE(STATUS "...add HMMWV_DEFORMABLE_SOIL demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO_FILES
    demo_HMMWV_DeformableSoil.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Add executable

ADD_EXECUTABLE(demo_HMMWV_DeformableSoil ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_HMMWV_DeformableSoil PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV_DeformableSoil
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle)
INSTALL(TARGETS demo_HMMWV_DeformableSoil DESTINATION bin)
