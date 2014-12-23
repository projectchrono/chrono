#=============================================================================
# CMake configuration file for the ChronoVehicle library
#=============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoVehicle LIBRARY
# ------------------------------------------------------------------------------

SET(CV_BASE_FILES
    ChApiSubsys.h
    ChSubsysDefs.h
    ChVehicleModelData.h
    ChVehicleModelData.cpp
    ChDriver.h
    ChDriver.cpp
    ChPowertrain.h
    ChPowertrain.cpp
    ChDriveline.h
    ChDriveline.cpp
    ChSuspension.h
    ChSuspension.cpp
    ChSuspensionTest.h
    ChSuspensionTest.cpp
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

SET(CV_VEHICLE_FILES
    vehicle/Vehicle.h
    vehicle/Vehicle.cpp
)

SET(CV_SUSPENSION_FILES
    suspension/ChDoubleWishbone.h
    suspension/ChDoubleWishbone.cpp
    suspension/ChDoubleWishboneReduced.h
    suspension/ChDoubleWishboneReduced.cpp
    suspension/ChSolidAxle.h
    suspension/ChSolidAxle.cpp
    suspension/ChMultiLink.h
    suspension/ChMultiLink.cpp

    suspension/DoubleWishbone.h
    suspension/DoubleWishbone.cpp
    suspension/DoubleWishboneReduced.h
    suspension/DoubleWishboneReduced.cpp
    suspension/SolidAxle.h
    suspension/SolidAxle.cpp
    suspension/MultiLink.h
    suspension/MultiLink.cpp
)

SET(CV_WHEEL_FILES
    wheel/Wheel.h
    wheel/Wheel.cpp
)

SET(CV_STEERING_FILES
    steering/ChPitmanArm.h
    steering/ChPitmanArm.cpp
    steering/ChRackPinion.h
    steering/ChRackPinion.cpp

    steering/PitmanArm.h
    steering/PitmanArm.cpp
    steering/RackPinion.h
    steering/RackPinion.cpp
)

SET(CV_DRIVELINE_FILES
    driveline/ChShaftsDriveline2WD.h
    driveline/ChShaftsDriveline2WD.cpp
    driveline/ChShaftsDriveline4WD.h
    driveline/ChShaftsDriveline4WD.cpp

    driveline/ShaftsDriveline2WD.h
    driveline/ShaftsDriveline2WD.cpp
    driveline/ShaftsDriveline4WD.h
    driveline/ShaftsDriveline4WD.cpp
)

SET(CV_DRIVER_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
)

SET(CV_POVERTRAIN_FILES
    powertrain/ChSimplePowertrain.h
    powertrain/ChSimplePowertrain.cpp
    powertrain/ChShaftsPowertrain.h
    powertrain/ChShaftsPowertrain.cpp

    powertrain/SimplePowertrain.h
    powertrain/SimplePowertrain.cpp
)

SET(CV_TIRE_FILES
    tire/ChRigidTire.h
    tire/ChRigidTire.cpp
    tire/ChPacejkaTire.h
    tire/ChPacejkaTire.cpp
    tire/ChLugreTire.h
    tire/ChLugreTire.cpp

    tire/RigidTire.h
    tire/RigidTire.cpp
    tire/LugreTire.h
    tire/LugreTire.cpp
)

SET(CV_BRAKE_FILES
    brake/ChBrakeSimple.h
    brake/ChBrakeSimple.cpp

    brake/BrakeSimple.h
    brake/BrakeSimple.cpp
)

SET(CV_TERRAIN_FILES
    terrain/FlatTerrain.h
    terrain/FlatTerrain.cpp
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
)

SET(CV_SUSPENSIONTEST_FILES
    suspensionTest/SuspensionTest.h
    suspensionTest/SuspensionTest.cpp
)

IF (ENABLE_IRRLICHT)
    SET(CV_DRIVER_FILES ${CV_DRIVER_FILES}
        driver/ChChaseCamera.h
        driver/ChChaseCamera.cpp
        driver/ChIrrGuiDriver.h
        driver/ChIrrGuiDriver.cpp
        driver/ChIrrGuiST.h
        driver/ChIrrGuiST.cpp
    )

    # On Windows, disable warning C4275 
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
    ENDIF()
ENDIF()

SOURCE_GROUP("base" FILES ${CV_BASE_FILES})
SOURCE_GROUP("vehicle" FILES ${CV_VEHICLE_FILES})
SOURCE_GROUP("suspension" FILES ${CV_SUSPENSION_FILES})
SOURCE_GROUP("wheel" FILES ${CV_WHEEL_FILES})
SOURCE_GROUP("steering" FILES ${CV_STEERING_FILES})
SOURCE_GROUP("driveline" FILES ${CV_DRIVELINE_FILES})
SOURCE_GROUP("driver" FILES ${CV_DRIVER_FILES})
SOURCE_GROUP("powertrain" FILES ${CV_POVERTRAIN_FILES})
SOURCE_GROUP("tire" FILES ${CV_TIRE_FILES})
SOURCE_GROUP("brake" FILES ${CV_BRAKE_FILES})
SOURCE_GROUP("terrain" FILES ${CV_TERRAIN_FILES})
SOURCE_GROUP("suspensionTest" FILES ${CV_SUSPENSIONTEST_FILES})

# ------------------------------------------------------------------------------
# ADD THE ChronoVehicle LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoVehicle SHARED
    ${CV_BASE_FILES}
    ${CV_VEHICLE_FILES}
    ${CV_SUSPENSION_FILES}
    ${CV_WHEEL_FILES}
    ${CV_STEERING_FILES}
    ${CV_DRIVELINE_FILES}
    ${CV_DRIVER_FILES}
    ${CV_POVERTRAIN_FILES}
    ${CV_TIRE_FILES}
    ${CV_BRAKE_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_SUSPENSIONTEST_FILES}
)

SET_TARGET_PROPERTIES(ChronoVehicle PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_GPU}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_SUBSYS"
)

TARGET_LINK_LIBRARIES(ChronoVehicle 
    ${CHRONOENGINE_LIBRARIES}
    ${CH_IRRKLANGLIB}
)

INSTALL(TARGETS ChronoVehicle
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

