#=============================================================================
# CMake configuration file for the Chrono_Subsys library
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the Chrono_Subsys LIBRARY
# ------------------------------------------------------------------------------

SET(Chrono_Subsys_FILES
    ChApiSubsys.h
    ChDriver.h
    ChDriver.cpp
    ChPowertrain.h
    ChPowertrain.cpp
    ChSuspension.h
    ChSuspension.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChWheel.h
    ChWheel.cpp
    ChTire.h
    ChTire.cpp
    ChTerrain.h
    ChTerrain.cpp
)


SET(Chrono_Susp_FILES
    suspension/ChDoubleWishboneReduced.h
    suspension/ChDoubleWishboneReduced.cpp
)

SET(Chrono_Driver_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
)

SET(Chrono_Powertrain_FILES
    powertrain/ChSimplePowertrain.h
    powertrain/ChSimplePowertrain.cpp
    powertrain/ChShaftsPowertrain.h
    powertrain/ChShaftsPowertrain.cpp
    powertrain/ChShaftsPowertrain4wd.h
    powertrain/ChShaftsPowertrain4wd.cpp
)

SET(Chrono_Tire_FILES
    tire/ChRigidTire.h
    tire/ChRigidTire.cpp
	tire/ChPacejkaTire.h
	tire/ChPacejkaTire.cpp
)


IF (ENABLE_IRRLICHT)
    SET(Chrono_Driver_FILES ${Chrono_Driver_FILES}
        driver/ChIrrGuiDriver.h
        driver/ChIrrGuiDriver.cpp
    )

    # On Windows, disable warning C4275 
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
    ENDIF()
ENDIF()

SOURCE_GROUP("base" FILES ${Chrono_Subsys_FILES})
SOURCE_GROUP("suspension" FILES ${Chrono_Susp_FILES})
SOURCE_GROUP("driver" FILES ${Chrono_Driver_FILES})
SOURCE_GROUP("powertrain" FILES ${Chrono_Powertrain_FILES})
SOURCE_GROUP("tire" FILES ${Chrono_Tire_FILES})

# ------------------------------------------------------------------------------
# ADD THE Chrono_Subsys LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(Chrono_Subsys SHARED
    ${Chrono_Subsys_FILES}
    ${Chrono_Susp_FILES}
    ${Chrono_Driver_FILES}
    ${Chrono_Powertrain_FILES}
    ${Chrono_Tire_FILES}
)

SET_TARGET_PROPERTIES(Chrono_Subsys PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_SUBSYS")

TARGET_LINK_LIBRARIES(Chrono_Subsys Chrono_Utils ${CHRONOENGINE_LIBRARIES})

INSTALL(TARGETS Chrono_Subsys
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

