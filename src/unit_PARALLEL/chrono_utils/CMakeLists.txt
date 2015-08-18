#=============================================================================
# CMake configuration file for the ChronoEngine_ParallelUtils library
#
# Invoked from the unit_PARALLEL CMakeLists.txt using ADD_SUBDIRECTORY()
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_ParallelUtils LIBRARY
# ------------------------------------------------------------------------------

SET(ChronoEngine_ParallelUtils_SOURCES
    ChUtilsCreators.cpp
    ChUtilsGenerators.cpp
    ChUtilsInputOutput.cpp
    )

SET(ChronoEngine_ParallelUtils_HEADERS
    ChApiUtils.h
    ChUtilsGeometry.h
    ChUtilsCreators.h
    ChUtilsGenerators.h
    ChUtilsSamplers.h
    ChUtilsInputOutput.h
    )

# Link to Chrono and Chrono_Parallel LIBRARIES

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_Parallel
    )

# ------------------------------------------------------------------------------
# ADD THE ChronoEngine_ParallelUtils LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_ParallelUtils SHARED
    ${ChronoEngine_ParallelUtils_HEADERS}
    ${ChronoEngine_ParallelUtils_SOURCES})

SET_TARGET_PROPERTIES(ChronoEngine_ParallelUtils PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_GPU}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS")

TARGET_LINK_LIBRARIES(ChronoEngine_ParallelUtils ${LIBRARIES})

INSTALL(TARGETS ChronoEngine_ParallelUtils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

