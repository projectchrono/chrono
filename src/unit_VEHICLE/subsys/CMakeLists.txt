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
    ChDriveline.h
    ChDriveline.cpp
    ChSuspension.h
    ChSuspension.cpp
    ChSteering.h
    ChSteering.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChWheel.h
    ChWheel.cpp
    ChTire.h
    ChTire.cpp
    ChTerrain.h
    ChTerrain.cpp
    ChBrake.h
    ChBrake.cpp
)


SET(Chrono_Susp_FILES
    suspension/ChDoubleWishbone.h
    suspension/ChDoubleWishbone.cpp
    suspension/ChDoubleWishboneReduced.h
    suspension/ChDoubleWishboneReduced.cpp
    suspension/ChSolidAxle.h
    suspension/ChSolidAxle.cpp

    suspension/DoubleWishbone.h
    suspension/DoubleWishbone.cpp
    suspension/SolidAxle.h
    suspension/SolidAxle.cpp
)

SET(Chrono_Wheel_FILES
    wheel/Wheel.h
    wheel/Wheel.cpp
)

SET(Chrono_Steering_FILES
    steering/ChPitmanArm.h
    steering/ChPitmanArm.cpp
    steering/ChRackPinion.h
    steering/ChRackPinion.cpp

    steering/PitmanArm.h
    steering/PitmanArm.cpp
    steering/RackPinion.h
    steering/RackPinion.cpp
)

SET(Chrono_Driveline_FILES
    driveline/ChShaftsDriveline2WD.h
    driveline/ChShaftsDriveline2WD.cpp
    driveline/ChShaftsDriveline4WD.h
    driveline/ChShaftsDriveline4WD.cpp

    driveline/ShaftsDriveline2WD.h
    driveline/ShaftsDriveline2WD.cpp
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
)

SET(Chrono_Tire_FILES
    tire/ChRigidTire.h
    tire/ChRigidTire.cpp
    tire/ChPacejkaTire.h
    tire/ChPacejkaTire.cpp
    tire/ChLugreTire.h
    tire/ChLugreTire.cpp
)

SET(Chrono_Brake_FILES
    brake/ChBrakeSimple.h
    brake/ChBrakeSimple.cpp

    brake/BrakeSimple.h
    brake/BrakeSimple.cpp
)

SET(Chrono_terrain_FILES
    terrain/FlatTerrain.h
    terrain/FlatTerrain.cpp
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
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
SOURCE_GROUP("wheel" FILES ${Chrono_Wheel_FILES})
SOURCE_GROUP("steering" FILES ${Chrono_Steering_FILES})
SOURCE_GROUP("driveline" FILES ${Chrono_Driveline_FILES})
SOURCE_GROUP("driver" FILES ${Chrono_Driver_FILES})
SOURCE_GROUP("powertrain" FILES ${Chrono_Powertrain_FILES})
SOURCE_GROUP("tire" FILES ${Chrono_Tire_FILES})
SOURCE_GROUP("brake" FILES ${Chrono_Brake_FILES})
SOURCE_GROUP("terrain" FILES ${Chrono_terrain_FILES})

# ------------------------------------------------------------------------------
# ADD THE Chrono_Subsys LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(Chrono_Subsys SHARED
    ${Chrono_Subsys_FILES}
    ${Chrono_Susp_FILES}
    ${Chrono_Wheel_FILES}
    ${Chrono_Steering_FILES}
    ${Chrono_Driveline_FILES}
    ${Chrono_Driver_FILES}
    ${Chrono_Powertrain_FILES}
    ${Chrono_Tire_FILES}
    ${Chrono_Brake_FILES}
	${Chrono_terrain_FILES}
)

SET_TARGET_PROPERTIES(Chrono_Subsys PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_SUBSYS")

TARGET_LINK_LIBRARIES(Chrono_Subsys 
				Chrono_Utils 
				${CHRONOENGINE_LIBRARIES}
				${CH_IRRKLANGLIB}
				)

INSTALL(TARGETS Chrono_Subsys
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

