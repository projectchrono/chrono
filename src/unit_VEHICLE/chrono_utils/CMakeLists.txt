#=============================================================================
# CMake configuration file for the ChronoEngine_VehicleUtils library
#=============================================================================

# ----------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_VehicleUtils LIBRARY
# ----------------------------------------------------------------------------

SET(FILES
    ChApiUtils.h
    ChUtilsGeometry.h
    ChUtilsCreators.h
    ChUtilsCreators.cpp
    ChUtilsInputOutput.h
    ChUtilsInputOutput.cpp
    ChUtilsValidation.h
    ChUtilsValidation.cpp
)

SOURCE_GROUP("utils" FILES ${FILES})

# ----------------------------------------------------------------------------
# Required libraries
# ----------------------------------------------------------------------------

SET(LIBRARIES
    ChronoEngine)

# ----------------------------------------------------------------------------
# ADD THE ChronoEngine_VehicleUtils LIBRARY
# ----------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_VehicleUtils SHARED ${FILES})

SET_TARGET_PROPERTIES(ChronoEngine_VehicleUtils PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS"
)

TARGET_LINK_LIBRARIES(ChronoEngine_VehicleUtils ${LIBRARIES})

INSTALL(TARGETS ChronoEngine_VehicleUtils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

