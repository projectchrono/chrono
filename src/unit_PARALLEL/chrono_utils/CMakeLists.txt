#=============================================================================
# CMake configuration file for the ChronoEngine_Utils library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_Utils LIBRARY
# ------------------------------------------------------------------------------

SET(ChronoEngine_Utils_SOURCES
    ChUtilsCreators.cpp
    ChUtilsGenerators.cpp
    ChUtilsInputOutput.cpp
    )

SET(ChronoEngine_Utils_HEADERS
    ChUtilsCommon.h
    ChUtilsGeometry.h
    ChUtilsCreators.h
    ChUtilsGenerators.h
    ChUtilsSamplers.h
    ChUtilsInputOutput.h
    )

# Include Chrono and Chrono_Parallel HEADERS

INCLUDE_DIRECTORIES(
    ${CH_CHRONO_SDKDIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

# Link to Chrono and Chrono_Parallel LIBRARIES

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_Parallel
    )

# ------------------------------------------------------------------------------
# Optional Chrono::Vehicle support
# ------------------------------------------------------------------------------
IF(ENABLE_VEHICLE)

  SET(ChronoEngine_Utils_SOURCES
      ${ChronoEngine_Utils_SOURCES}
      ChUtilsVehicle.cpp
      )

  SET(ChronoEngine_Utils_HEADERS
      ${ChronoEngine_Utils_HEADERS}
      ChUtilsVehicle.h
      )

   INCLUDE_DIRECTORIES(
      ${CHRONOVEHICLE_INCLUDES}
      )

  SET(LIBRARIES
      ${LIBRARIES}
      ${CHRONOVEHICLE_LIBRARIES}
      )

ENDIF()

# ------------------------------------------------------------------------------
# ADD THE ChronoEngine_Utils LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_Utils SHARED
    ${ChronoEngine_Utils_HEADERS}
    ${ChronoEngine_Utils_SOURCES})

SET_TARGET_PROPERTIES(ChronoEngine_Utils PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS")

TARGET_LINK_LIBRARIES(ChronoEngine_Utils ${LIBRARIES})

INSTALL(TARGETS ChronoEngine_Utils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

