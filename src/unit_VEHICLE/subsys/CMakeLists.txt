#=============================================================================
# CMake configuration file for the Chrono_Subsys library
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the Chrono_Subsys LIBRARY
# ------------------------------------------------------------------------------

SET(Chrono_Subsys_HEADERS
    ChApiSubsys.h
    ChDriver.h
    ChPowertrain.h
    ChSuspension.h
    ChVehicle.h
    ChWheel.h
    )

SET(Chrono_Subsys_SOURCES
    ChDriver.cpp
    ChPowertrain.cpp
    ChSuspension.cpp
    ChVehicle.cpp
    ChWheel.cpp
    )

SET(Chrono_Susp_HEADERS
    suspension/ChDoubleWishboneReduced.h
    )

SET(Chrono_Susp_SOURCES
    suspension/ChDoubleWishboneReduced.cpp
    )

SOURCE_GROUP("Header Files" FILES ${Chrono_Subsys_HEADERS})
SOURCE_GROUP("Source Files" FILES ${Chrono_Subsys_SOURCES})

SOURCE_GROUP("Header Files\\susp" FILES ${Chrono_Susp_HEADERS})
SOURCE_GROUP("Source Files\\susp" FILES ${Chrono_Susp_SOURCES})

# ------------------------------------------------------------------------------
# ADD THE Chrono_Subsys LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(Chrono_Subsys SHARED
    ${Chrono_Subsys_HEADERS}
    ${Chrono_Subsys_SOURCES}
    ${Chrono_Susp_HEADERS}
    ${Chrono_Susp_SOURCES}
    )

SET_TARGET_PROPERTIES(Chrono_Subsys PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_SUBSYS")

TARGET_LINK_LIBRARIES(Chrono_Subsys ${CHRONOENGINE_LIBRARY})

INSTALL(TARGETS Chrono_Subsys
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )

