#=============================================================================
# CMake configuration file for the DEFORMABLE_SOIL demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

IF(NOT ENABLE_MODULE_PARDISO_MKL)
    RETURN()
ENDIF()

IF(NOT ENABLE_MODULE_VEHICLE)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(PROGRAM demo_VEH_DeformableSoilAndTire)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoEngine_pardisomkl
)


#set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
#set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")


#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
