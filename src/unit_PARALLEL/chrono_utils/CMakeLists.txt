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
    ChUtilsCreators.h
    ChUtilsGenerators.h
    ChUtilsSamplers.h
    ChUtilsInputOutput.h
    )

#
# INCLUDE Chrono and Chrono_Parallel HEADERS
#

INCLUDE_DIRECTORIES(
    ${CHRONO_INC}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

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

TARGET_LINK_LIBRARIES(ChronoEngine_Utils ${CHRONO_LIB} ChronoEngine_Parallel)

INSTALL(TARGETS ChronoEngine_Utils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

