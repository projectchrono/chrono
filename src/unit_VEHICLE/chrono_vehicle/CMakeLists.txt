#=============================================================================
# CMake configuration file for the ChronoEngine_Vehicle libraries
#=============================================================================

# ----------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_Vehicle LIBRARY
# ----------------------------------------------------------------------------

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
    ChAntirollBar.h
    ChAntirollBar.cpp
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
    suspension/ChHendricksonPRIMAXX.h
    suspension/ChHendricksonPRIMAXX.cpp

    suspension/DoubleWishbone.h
    suspension/DoubleWishbone.cpp
    suspension/DoubleWishboneReduced.h
    suspension/DoubleWishboneReduced.cpp
    suspension/SolidAxle.h
    suspension/SolidAxle.cpp
    suspension/MultiLink.h
    suspension/MultiLink.cpp
    suspension/HendricksonPRIMAXX.h
    suspension/HendricksonPRIMAXX.cpp
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
    driveline/ChSimpleDriveline.h
    driveline/ChSimpleDriveline.cpp

    driveline/ShaftsDriveline2WD.h
    driveline/ShaftsDriveline2WD.cpp
    driveline/ShaftsDriveline4WD.h
    driveline/ShaftsDriveline4WD.cpp
    driveline/SimpleDriveline.h
    driveline/SimpleDriveline.cpp
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
    powertrain/ShaftsPowertrain.h
    powertrain/ShaftsPowertrain.cpp
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

SET(CV_ANTIROLL_FILES
    antirollbar/ChAntirollBarRSD.h
    antirollbar/ChAntirollBarRSD.cpp

    antirollbar/AntirollBarRSD.h
    antirollbar/AntirollBarRSD.cpp
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

IF (ENABLE_UNIT_IRRLICHT)
    SET(CVIRR_DRIVER_FILES
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
ELSE()
    SET(CVIRR_DRIVER_FILES "")
ENDIF()

SOURCE_GROUP("base" FILES ${CV_BASE_FILES})
SOURCE_GROUP("vehicle" FILES ${CV_VEHICLE_FILES})
SOURCE_GROUP("suspension" FILES ${CV_SUSPENSION_FILES})
SOURCE_GROUP("wheel" FILES ${CV_WHEEL_FILES})
SOURCE_GROUP("steering" FILES ${CV_STEERING_FILES})
SOURCE_GROUP("driveline" FILES ${CV_DRIVELINE_FILES})
SOURCE_GROUP("driver" FILES ${CV_DRIVER_FILES} ${CVIRR_DRIVER_FILES})
SOURCE_GROUP("powertrain" FILES ${CV_POVERTRAIN_FILES})
SOURCE_GROUP("tire" FILES ${CV_TIRE_FILES})
SOURCE_GROUP("brake" FILES ${CV_BRAKE_FILES})
SOURCE_GROUP("antirollbar" FILES ${CV_ANTIROLL_FILES})
SOURCE_GROUP("terrain" FILES ${CV_TERRAIN_FILES})
SOURCE_GROUP("suspensionTest" FILES ${CV_SUSPENSIONTEST_FILES})

# ----------------------------------------------------------------------------
# ADD THE ChronoEngine_Vehicle LIBRARY
# ----------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_Vehicle SHARED
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
    ${CV_ANTIROLL_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_SUSPENSIONTEST_FILES}
)

SET_TARGET_PROPERTIES(ChronoEngine_Vehicle PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_VEHICLE"
)

TARGET_LINK_LIBRARIES(ChronoEngine_Vehicle 
    ChronoEngine
)

INSTALL(TARGETS ChronoEngine_Vehicle
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# ----------------------------------------------------------------------------
# ADD THE ChronoVehicle_Irrlicht LIBRARY
# ----------------------------------------------------------------------------

IF (ENABLE_UNIT_IRRLICHT)
  INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
  
  ADD_LIBRARY(ChronoEngine_VehicleIrrlicht SHARED
    ${CVIRR_DRIVER_FILES}
  )

  SET_TARGET_PROPERTIES(ChronoEngine_VehicleIrrlicht PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_GPU}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_VEHICLE"
  )

  TARGET_LINK_LIBRARIES(ChronoEngine_VehicleIrrlicht
    ChronoEngine_Vehicle
    ChronoEngine
    ChronoEngine_IRRLICHT
    ${CH_IRRKLANGLIB}
  )

  INSTALL(TARGETS ChronoEngine_VehicleIrrlicht
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  # On Windows, copy the Irrklang DLL (if enabled)
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    IF(ENABLE_IRRKLANG)
      IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        SET(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
      ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
      ENDIF()

      ADD_CUSTOM_COMMAND(
        TARGET ChronoEngine_VehicleIrrlicht POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CH_IRRKLANG_DLL}"
                "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
        )
    ENDIF()

  ENDIF()


ENDIF()
