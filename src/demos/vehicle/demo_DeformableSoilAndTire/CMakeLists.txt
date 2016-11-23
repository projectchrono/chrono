#=============================================================================
# CMake configuration file for the DEFORMABLE_SOIL demo.
# This example program works only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

IF(NOT ENABLE_MODULE_MKL)
    RETURN()
ENDIF()

IF(NOT ENABLE_MODULE_VEHICLE)
    RETURN()
ENDIF()

IF(NOT ENABLE_MODULE_FEA)
    RETURN()
ENDIF()


#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO
    demo_VEH_DeformableSoilAndTire
)

SOURCE_GROUP("" FILES ${DEMO}.cpp)

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
	ChronoEngine_mkl
    ${CH_IRRLICHTLIB}
)


#set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
#set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")


#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES})
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

