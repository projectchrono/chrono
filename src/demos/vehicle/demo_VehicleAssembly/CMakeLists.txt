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
    ChronoEngine_ParallelUtils
    ChronoEngine_ParallelOpenGL
)

#--------------------------------------------------------------
# OpenMP flags

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------
# Create the executable

ADD_EXECUTABLE(demo_VehicleAssembly ${DEMO_FILES})
SET_TARGET_PROPERTIES(demo_VehicleAssembly PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_VehicleAssembly ${LIBRARIES})
INSTALL(TARGETS demo_VehicleAssembly DESTINATION bin)

