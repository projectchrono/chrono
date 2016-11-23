#=============================================================================
# CMake configuration file for the VEHICLE_ASSEMBLY demo.
# This example program requires unit_PARALLEL with OpenGL support.
#=============================================================================

#--------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_OPENGL OR NOT ENABLE_MODULE_PARALLEL)
  RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO
    demo_VEH_WheeledAssembly
)

SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(
    ${CH_PARALLEL_INCLUDES}
    ${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_parallel
    ChronoEngine_opengl)

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES})
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
