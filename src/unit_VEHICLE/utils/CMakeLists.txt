#=============================================================================
# CMake configuration file for the ChronoVehicle_Utils library
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoVehicle_Utils LIBRARY
# ------------------------------------------------------------------------------

SET(CV_UTILS_FILES
    ChApiUtils.h
    ChUtilsGeometry.h
    ChUtilsCreators.h
    ChUtilsCreators.cpp
    ChUtilsInputOutput.h
    ChUtilsInputOutput.cpp
    ChUtilsValidation.h
    ChUtilsValidation.cpp
)

SOURCE_GROUP("utils" FILES ${CV_UTILS_FILES})

# ------------------------------------------------------------------------------
# ADD THE ChronoVehicle_Utils LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoVehicle_Utils SHARED ${CV_UTILS_FILES})

SET_TARGET_PROPERTIES(ChronoVehicle_Utils PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_GPU}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS"
)

TARGET_LINK_LIBRARIES(ChronoVehicle_Utils ${CHRONOENGINE_LIBRARY})

INSTALL(TARGETS ChronoVehicle_Utils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

