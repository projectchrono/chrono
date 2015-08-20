#=============================================================================
# CMake configuration file for the VEHICLE_ASSEMBLY demo.
# This example program requires unit_PARALLEL with OpenGL support.
#=============================================================================

#--------------------------------------------------------------
# Check dependencies.
# Note that it is enough to check if ENABLE_OPENGL is true.

IF(NOT ENABLE_OPENGL)
  RETURN()
ENDIF()

MESSAGE(STATUS "...add VEHICLE_ASSEMBLY demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO_FILES
    demo_VehicleAssembly.cpp
)

SOURCE_GROUP("" FILES ${DEMO_FILES})

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(
    "${CMAKE_SOURCE_DIR}/unit_VEHICLE"
    ${CH_PARALLEL_INCLUDES}
    ${CH_OPENGL_INCLUDES}
    ${PROJECT_BINARY_DIR}
)

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_Vehicle
    ChronoEngine_Parallel
    ChronoEngine_ParallelOpenGL
)

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------
# Create the executable

ADD_EXECUTABLE(demo_VehicleAssembly ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_VehicleAssembly PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_VehicleAssembly ${LIBRARIES})
INSTALL(TARGETS demo_VehicleAssembly DESTINATION bin)

