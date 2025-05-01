#=============================================================================
# CMake configuration file for Chrono Vehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_VEHICLE)
   mark_as_advanced(FORCE CH_ENABLE_OPENCRG)
   mark_as_advanced(FORCE CH_ENABLE_IRRKLANG)
   mark_as_advanced(FORCE IRRKLANG_INSTALL_DIR)
   mark_as_advanced(FORCE IRRKLANG_LIBRARY)
   return()
endif()

message(STATUS "\n==== Chrono Vehicle module ====\n")

mark_as_advanced(CLEAR CH_ENABLE_OPENCRG)
mark_as_advanced(CLEAR CH_ENABLE_IRRKLANG)
mark_as_advanced(CLEAR IRRKLANG_INSTALL_DIR)
mark_as_advanced(CLEAR IRRKLANG_LIBRARY)

# Provide option to add OpenCRG support.
option(CH_ENABLE_OPENCRG "Enable OpenCRG terrain library support" OFF)

# If Irrlicht support was enabled, provide option to add Irrklang support.
cmake_dependent_option(CH_ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
                       "CH_ENABLE_MODULE_IRRLICHT" OFF)

# ----------------------------------------------------------------------------
# Find the OpenCRG library
# ----------------------------------------------------------------------------

if(CH_ENABLE_OPENCRG)

  find_package(OpenCRG)

  if (OpenCRG_FOUND)
    message(STATUS "OpenCRG include dir: ${OpenCRG_INCLUDE_DIR}")
    message(STATUS "OpenCRG library:     ${OpenCRG_LIBRARY}")

    mark_as_advanced(FORCE OpenCRG_INCLUDE_DIR)
    mark_as_advanced(FORCE OpenCRG_LIBRARY)
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        mark_as_advanced(FORCE OpenCRG_DLL)
    endif()

    set(HAVE_OPENCRG ON)

    file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindOpenCRG.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
    install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindOpenCRG.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})

  else()
    message("WARNINIG: OpenCRG not found; disabling CRG support in Chrono::Vehicle")

    set(OpenCRG_INCLUDE_DIR "" CACHE PATH "Directory containing OpenCRG include files")
    set(OpenCRG_LIBRARY "" CACHE FILEPATH "OpenCRG library")

    mark_as_advanced(CLEAR OpenCRG_INCLUDE_DIR)
    mark_as_advanced(CLEAR OpenCRG_LIBRARY)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        set(OpenCRG_DLL "" CACHE FILEPATH "OpenCRG DLL (Windows only)")
        mark_as_advanced(CLEAR OpenCRG_DLL)
    endif()

    set(HAVE_OPENCRG OFF)

  endif()
endif()

SET(HAVE_OPENCRG "${HAVE_OPENCRG}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------

if(CH_ENABLE_IRRKLANG)

  set(IRRKLANG_ROOT "" CACHE PATH "IrrKlang SDK root directory")
  set(IRRKLANG_LIBDIR "" CACHE PATH "IrrKlang SDK library directory")
  find_package(IRRKLANG)

  if (NOT IRRKLANG_FOUND)
    mark_as_advanced(CLEAR IRRKLANG_ROOT)
    mark_as_advanced(CLEAR IRRKLANG_LIBDIR)
  else()
    set(IRRKLANG_LIBDIR ${IRRKLANG_LIBRARY_DIR} CACHE STRING "IrrKlang SDK library directory" FORCE)
    mark_as_advanced(FORCE IRRKLANG_ROOT)
    mark_as_advanced(FORCE IRRKLANG_LIBDIR)
  endif()

  message(STATUS "IrrKlang include dir: ${IRRKLANG_INCLUDE_DIR}")
  message(STATUS "IrrKlang library:     ${IRRKLANG_LIBRARY}")

  set(HAVE_IRRKLANG ${IRRKLANG_FOUND})

else()

  set(HAVE_IRRKLANG FALSE)

endif()

SET(HAVE_IRRKLANG "${HAVE_IRRKLANG}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables
if(HAVE_OPENCRG)
  set(CHRONO_OPENCRG "#define CHRONO_OPENCRG")
else()
  set(CHRONO_OPENCRG "#undef CHRONO_OPENCRG")
endif()

if(CH_ENABLE_IRRKLANG)
  set(CHRONO_IRRKLANG "#define CHRONO_IRRKLANG")
else()
  set(CHRONO_IRRKLANG "#undef CHRONO_IRRKLANG")
endif()

# Generate the configuration header file using substitution variables.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigVehicle.h.in
               ${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicle.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicle.h"
        DESTINATION include/chrono_vehicle)

if(CH_ENABLE_MODULE_VSG)
    # Find the VSG library
    if (VULKAN_SDK)
        set(ENV{VULKAN_SDK} ${VULKAN_SDK})
    endif()
    
    find_package(vsg REQUIRED)
    find_package(vsgImGui REQUIRED)
    find_package(vsgXchange REQUIRED)
endif()

# ----------------------------------------------------------------------------
# List the files in the Chrono_vehicle library
# ----------------------------------------------------------------------------

# --------------- COMMON FILES

set(CV_BASE_FILES
    ChApiVehicle.h
    ChChassis.h
    ChChassis.cpp
    ChDriver.h
    ChDriver.cpp
    ChPart.h
    ChPart.cpp
    ChVehicleJoint.h
    ChVehicleJoint.cpp
    ChDriveline.h
    ChDriveline.cpp
    ChPowertrainAssembly.h
    ChPowertrainAssembly.cpp
    ChEngine.h
    ChEngine.cpp
    ChTransmission.h
    ChTransmission.cpp
    ChSubsysDefs.h
    ChSubsysDefs.cpp
    ChTerrain.h
    ChTerrain.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChVehicleModelData.h
    ChVehicleModelData.cpp
    ChVehicleVisualSystem.h
    ChVehicleVisualSystem.cpp
    ChVehicleOutput.h
    ChWorldFrame.cpp
    ChWorldFrame.h
)
source_group("base" FILES ${CV_BASE_FILES})

set(CV_CHASSIS_FILES
    chassis/ChRigidChassis.h
    chassis/ChRigidChassis.cpp

    chassis/ChChassisConnectorArticulated.h
    chassis/ChChassisConnectorArticulated.cpp
    chassis/ChChassisConnectorHitch.h
    chassis/ChChassisConnectorHitch.cpp
    chassis/ChChassisConnectorFifthWheel.h
    chassis/ChChassisConnectorFifthWheel.cpp
    chassis/ChChassisConnectorTorsion.h
    chassis/ChChassisConnectorTorsion.cpp

    chassis/RigidChassis.h
    chassis/RigidChassis.cpp

    chassis/ChassisConnectorHitch.h
    chassis/ChassisConnectorHitch.cpp
    chassis/ChassisConnectorFifthWheel.h
    chassis/ChassisConnectorFifthWheel.cpp
    chassis/ChassisConnectorArticulated.h
    chassis/ChassisConnectorArticulated.cpp
    chassis/ChassisConnectorTorsion.h
    chassis/ChassisConnectorTorsion.cpp
)
source_group("chassis" FILES ${CV_CHASSIS_FILES})

set(CV_DRIVER_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
    driver/ChHumanDriver.h
    driver/ChHumanDriver.cpp
    driver/ChAIDriver.h
    driver/ChAIDriver.cpp
    driver/ChPathFollowerDriver.h
    driver/ChPathFollowerDriver.cpp
    driver/ChPathFollowerACCDriver.h
    driver/ChPathFollowerACCDriver.cpp
    driver/ChInteractiveDriver.h
    driver/ChInteractiveDriver.cpp

    driver/AIDriver.h
    driver/AIDriver.cpp
)
source_group("driver" FILES ${CV_DRIVER_FILES})

set(CV_POVERTRAIN_FILES
    powertrain/ChEngineSimple.h
    powertrain/ChEngineSimple.cpp
    powertrain/ChEngineSimpleMap.h
    powertrain/ChEngineSimpleMap.cpp
    powertrain/ChEngineShafts.h
    powertrain/ChEngineShafts.cpp
    powertrain/EngineSimple.h
    powertrain/EngineSimple.cpp
    powertrain/EngineSimpleMap.h
    powertrain/EngineSimpleMap.cpp
    powertrain/EngineShafts.h
    powertrain/EngineShafts.cpp

    powertrain/ChAutomaticTransmissionSimpleMap.h
    powertrain/ChAutomaticTransmissionSimpleMap.cpp
    powertrain/ChAutomaticTransmissionShafts.h
    powertrain/ChAutomaticTransmissionShafts.cpp
    powertrain/ChManualTransmissionShafts.h
    powertrain/ChManualTransmissionShafts.cpp
    powertrain/AutomaticTransmissionSimpleMap.h
    powertrain/AutomaticTransmissionSimpleMap.cpp
    powertrain/AutomaticTransmissionShafts.h
    powertrain/AutomaticTransmissionShafts.cpp
    powertrain/ManualTransmissionShafts.h
    powertrain/ManualTransmissionShafts.cpp
    powertrain/ChAutomaticTransmissionSimpleCVT.cpp
    powertrain/ChAutomaticTransmissionSimpleCVT.h
    powertrain/AutomaticTransmissionSimpleCVT.cpp
    powertrain/AutomaticTransmissionSimpleCVT.h
)
source_group("powertrain" FILES ${CV_POVERTRAIN_FILES})

set(CV_TERRAIN_FILES
    terrain/FlatTerrain.h
    terrain/FlatTerrain.cpp
    terrain/ObsModTerrain.h
    terrain/ObsModTerrain.cpp
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
    terrain/RandomSurfaceTerrain.h
    terrain/RandomSurfaceTerrain.cpp
    terrain/SCMTerrain.h
    terrain/SCMTerrain.cpp
    terrain/GranularTerrain.h
    terrain/GranularTerrain.cpp
    terrain/FEATerrain.h
    terrain/FEATerrain.cpp
)
if(HAVE_OPENCRG)
    set(CV_TERRAIN_FILES ${CV_TERRAIN_FILES}
        terrain/CRGTerrain.h
        terrain/CRGTerrain.cpp
    )
endif()
if(CH_ENABLE_MODULE_FSI)
    set(CV_TERRAIN_FILES ${CV_TERRAIN_FILES}
        terrain/CRMTerrain.h
        terrain/CRMTerrain.cpp    
    )
endif()
source_group("terrain" FILES ${CV_TERRAIN_FILES})

set(CV_UTILS_FILES
    utils/ChSteeringController.h
    utils/ChSteeringController.cpp
    utils/ChSpeedController.h
    utils/ChSpeedController.cpp
    utils/ChAdaptiveSpeedController.h
    utils/ChAdaptiveSpeedController.cpp
    utils/ChVehiclePath.h
    utils/ChVehiclePath.cpp
    utils/ChUtilsJSON.h
    utils/ChUtilsJSON.cpp
)
source_group("utils" FILES ${CV_UTILS_FILES})

set(CV_OUTPUT_FILES
    output/ChVehicleOutputASCII.h
    output/ChVehicleOutputASCII.cpp
)
if (HDF5_FOUND)
    set(CVHDF5_OUTPUT_FILES
        output/ChVehicleOutputHDF5.h
        output/ChVehicleOutputHDF5.cpp
    )
else()
    set(CVHDF5_OUTPUT_FILES "")
endif()
source_group("output" FILES ${CV_OUTPUT_FILES} ${CVHDF5_OUTPUT_FILES})

# --------------- WHEELED VEHICLE FILES

set(CV_WV_BASE_FILES
    wheeled_vehicle/ChAxle.h
    wheeled_vehicle/ChAxle.cpp
    wheeled_vehicle/ChAntirollBar.h
    wheeled_vehicle/ChAntirollBar.cpp
    wheeled_vehicle/ChBrake.h
    wheeled_vehicle/ChBrake.cpp
    wheeled_vehicle/ChDrivelineWV.h
    wheeled_vehicle/ChDrivelineWV.cpp
    wheeled_vehicle/ChSuspension.h
    wheeled_vehicle/ChSuspension.cpp
    wheeled_vehicle/ChSteering.h
    wheeled_vehicle/ChSteering.cpp
    wheeled_vehicle/ChSubchassis.h
    wheeled_vehicle/ChSubchassis.cpp
    wheeled_vehicle/ChWheeledTrailer.h
    wheeled_vehicle/ChWheeledTrailer.cpp
    wheeled_vehicle/ChWheeledVehicle.h
    wheeled_vehicle/ChWheeledVehicle.cpp
    wheeled_vehicle/ChSpindle.h
    wheeled_vehicle/ChSpindle.cpp
    wheeled_vehicle/ChWheel.h
    wheeled_vehicle/ChWheel.cpp
    wheeled_vehicle/ChTire.h
    wheeled_vehicle/ChTire.cpp
)
source_group("wheeled_vehicle\\base" FILES ${CV_WV_BASE_FILES})

set(CV_WV_ANTIROLL_FILES
    wheeled_vehicle/antirollbar/ChAntirollBarRSD.h
    wheeled_vehicle/antirollbar/ChAntirollBarRSD.cpp

    wheeled_vehicle/antirollbar/AntirollBarRSD.h
    wheeled_vehicle/antirollbar/AntirollBarRSD.cpp
)
source_group("wheeled_vehicle\\antirollbar" FILES ${CV_WV_ANTIROLL_FILES})

set(CV_WV_BRAKE_FILES
    wheeled_vehicle/brake/ChBrakeSimple.h
    wheeled_vehicle/brake/ChBrakeSimple.cpp
    wheeled_vehicle/brake/ChBrakeShafts.h
    wheeled_vehicle/brake/ChBrakeShafts.cpp

    wheeled_vehicle/brake/BrakeSimple.h
    wheeled_vehicle/brake/BrakeSimple.cpp
    wheeled_vehicle/brake/BrakeShafts.h
    wheeled_vehicle/brake/BrakeShafts.cpp
)
source_group("wheeled_vehicle\\brake" FILES ${CV_WV_BRAKE_FILES})

set(CV_WV_DRIVELINE_FILES
    wheeled_vehicle/driveline/ChShaftsDriveline2WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline2WD.cpp
    wheeled_vehicle/driveline/ChShaftsDriveline4WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline4WD.cpp
    wheeled_vehicle/driveline/ChShaftsDriveline6WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline6WD.cpp
    wheeled_vehicle/driveline/ChShaftsDriveline8WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline8WD.cpp
    wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h
    wheeled_vehicle/driveline/ChSimpleDrivelineXWD.cpp
    wheeled_vehicle/driveline/ChSimpleDriveline.h
    wheeled_vehicle/driveline/ChSimpleDriveline.cpp

    wheeled_vehicle/driveline/ShaftsDriveline2WD.h
    wheeled_vehicle/driveline/ShaftsDriveline2WD.cpp
    wheeled_vehicle/driveline/ShaftsDriveline4WD.h
    wheeled_vehicle/driveline/ShaftsDriveline4WD.cpp
    wheeled_vehicle/driveline/SimpleDriveline.h
    wheeled_vehicle/driveline/SimpleDriveline.cpp
    wheeled_vehicle/driveline/SimpleDrivelineXWD.h
    wheeled_vehicle/driveline/SimpleDrivelineXWD.cpp
)
source_group("wheeled_vehicle\\driveline" FILES ${CV_WV_DRIVELINE_FILES})

set(CV_WV_STEERING_FILES
    wheeled_vehicle/steering/ChPitmanArm.h
    wheeled_vehicle/steering/ChPitmanArm.cpp
    wheeled_vehicle/steering/ChPitmanArmShafts.h
    wheeled_vehicle/steering/ChPitmanArmShafts.cpp
    wheeled_vehicle/steering/ChRackPinion.h
    wheeled_vehicle/steering/ChRackPinion.cpp
    wheeled_vehicle/steering/ChRotaryArm.h
    wheeled_vehicle/steering/ChRotaryArm.cpp

    wheeled_vehicle/steering/PitmanArm.h
    wheeled_vehicle/steering/PitmanArm.cpp
    wheeled_vehicle/steering/RackPinion.h
    wheeled_vehicle/steering/RackPinion.cpp
    wheeled_vehicle/steering/RotaryArm.h
    wheeled_vehicle/steering/RotaryArm.cpp
)
source_group("wheeled_vehicle\\steering" FILES ${CV_WV_STEERING_FILES})

set(CV_WV_SUBCHASSIS_FILES
    wheeled_vehicle/subchassis/ChBalancer.h
    wheeled_vehicle/subchassis/ChBalancer.cpp

    wheeled_vehicle/subchassis/Balancer.h
    wheeled_vehicle/subchassis/Balancer.cpp
)
source_group("wheeled_vehicle\\subchassis" FILES ${CV_WV_SUBCHASSIS_FILES})

set(CV_WV_SUSPENSION_FILES
    wheeled_vehicle/suspension/ChDeDionAxle.h
    wheeled_vehicle/suspension/ChDeDionAxle.cpp
    wheeled_vehicle/suspension/ChDoubleWishbone.h
    wheeled_vehicle/suspension/ChDoubleWishbone.cpp
    wheeled_vehicle/suspension/ChDoubleWishboneReduced.h
    wheeled_vehicle/suspension/ChDoubleWishboneReduced.cpp
    wheeled_vehicle/suspension/ChSingleWishbone.h
    wheeled_vehicle/suspension/ChSingleWishbone.cpp
    wheeled_vehicle/suspension/ChSolidAxle.h
    wheeled_vehicle/suspension/ChSolidAxle.cpp
    wheeled_vehicle/suspension/ChMultiLink.h
    wheeled_vehicle/suspension/ChMultiLink.cpp
    wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h
    wheeled_vehicle/suspension/ChHendricksonPRIMAXX.cpp
    wheeled_vehicle/suspension/ChMacPhersonStrut.h
    wheeled_vehicle/suspension/ChMacPhersonStrut.cpp
    wheeled_vehicle/suspension/ChSemiTrailingArm.h
    wheeled_vehicle/suspension/ChSemiTrailingArm.cpp
    wheeled_vehicle/suspension/ChThreeLinkIRS.h
    wheeled_vehicle/suspension/ChThreeLinkIRS.cpp
    wheeled_vehicle/suspension/ChRigidSuspension.h
    wheeled_vehicle/suspension/ChRigidSuspension.cpp
    wheeled_vehicle/suspension/ChRigidPinnedAxle.h
    wheeled_vehicle/suspension/ChRigidPinnedAxle.cpp
    wheeled_vehicle/suspension/ChLeafspringAxle.h
    wheeled_vehicle/suspension/ChLeafspringAxle.cpp
    wheeled_vehicle/suspension/ChPushPipeAxle.h
    wheeled_vehicle/suspension/ChPushPipeAxle.cpp
    wheeled_vehicle/suspension/ChSAELeafspringAxle.h
    wheeled_vehicle/suspension/ChSAELeafspringAxle.cpp
    wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.h
    wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ChSolidThreeLinkAxle.h
    wheeled_vehicle/suspension/ChSolidThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ChToeBarDeDionAxle.h
    wheeled_vehicle/suspension/ChToeBarDeDionAxle.cpp
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/ChSAEToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ChSAEToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/ChToeBarPushPipeAxle.h
    wheeled_vehicle/suspension/ChToeBarPushPipeAxle.cpp
    wheeled_vehicle/suspension/ChGenericWheeledSuspension.h
    wheeled_vehicle/suspension/ChGenericWheeledSuspension.cpp
    wheeled_vehicle/suspension/ChRigidPanhardAxle.cpp
    wheeled_vehicle/suspension/ChRigidPanhardAxle.h
    wheeled_vehicle/suspension/ChToeBarRigidPanhardAxle.cpp
    wheeled_vehicle/suspension/ChToeBarRigidPanhardAxle.h

    wheeled_vehicle/suspension/DeDionAxle.h
    wheeled_vehicle/suspension/DeDionAxle.cpp
    wheeled_vehicle/suspension/DoubleWishbone.h
    wheeled_vehicle/suspension/DoubleWishbone.cpp
    wheeled_vehicle/suspension/DoubleWishboneReduced.h
    wheeled_vehicle/suspension/DoubleWishboneReduced.cpp
    wheeled_vehicle/suspension/SingleWishbone.h
    wheeled_vehicle/suspension/SingleWishbone.cpp
    wheeled_vehicle/suspension/SolidAxle.h
    wheeled_vehicle/suspension/SolidAxle.cpp
    wheeled_vehicle/suspension/MultiLink.h
    wheeled_vehicle/suspension/MultiLink.cpp
    wheeled_vehicle/suspension/HendricksonPRIMAXX.h
    wheeled_vehicle/suspension/HendricksonPRIMAXX.cpp
    wheeled_vehicle/suspension/MacPhersonStrut.h
    wheeled_vehicle/suspension/MacPhersonStrut.cpp
    wheeled_vehicle/suspension/SemiTrailingArm.h
    wheeled_vehicle/suspension/SemiTrailingArm.cpp
    wheeled_vehicle/suspension/ThreeLinkIRS.h
    wheeled_vehicle/suspension/ThreeLinkIRS.cpp
    wheeled_vehicle/suspension/RigidSuspension.h
    wheeled_vehicle/suspension/RigidSuspension.cpp
    wheeled_vehicle/suspension/RigidPinnedAxle.h
    wheeled_vehicle/suspension/RigidPinnedAxle.cpp
    wheeled_vehicle/suspension/LeafspringAxle.h
    wheeled_vehicle/suspension/LeafspringAxle.cpp
    wheeled_vehicle/suspension/PushPipeAxle.h
    wheeled_vehicle/suspension/PushPipeAxle.cpp
    wheeled_vehicle/suspension/SAELeafspringAxle.h
    wheeled_vehicle/suspension/SAELeafspringAxle.cpp
    wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h
    wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.cpp
    wheeled_vehicle/suspension/SolidThreeLinkAxle.h
    wheeled_vehicle/suspension/SolidThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ToeBarDeDionAxle.h
    wheeled_vehicle/suspension/ToeBarDeDionAxle.cpp
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/ToeBarPushPipeAxle.h
    wheeled_vehicle/suspension/ToeBarPushPipeAxle.cpp
    wheeled_vehicle/suspension/GenericWheeledSuspension.h
    wheeled_vehicle/suspension/GenericWheeledSuspension.cpp
    wheeled_vehicle/suspension/RigidPanhardAxle.cpp
    wheeled_vehicle/suspension/RigidPanhardAxle.h
    wheeled_vehicle/suspension/ToeBarRigidPanhardAxle.cpp
    wheeled_vehicle/suspension/ToeBarRigidPanhardAxle.h
)
source_group("wheeled_vehicle\\suspension" FILES ${CV_WV_SUSPENSION_FILES})

set(CV_WV_TEST_RIG_FILES
    wheeled_vehicle/test_rig/ChSuspensionTestRig.h
    wheeled_vehicle/test_rig/ChSuspensionTestRig.cpp
    wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h
    wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.cpp
    wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h
    wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.cpp
    wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h
    wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.cpp    
    wheeled_vehicle/test_rig/ChTireTestRig.h
    wheeled_vehicle/test_rig/ChTireTestRig.cpp
)
source_group("wheeled_vehicle\\test_rig" FILES ${CV_WV_TEST_RIG_FILES})

set(CV_WV_TIRE_FILES
    wheeled_vehicle/tire/ChRigidTire.h
    wheeled_vehicle/tire/ChRigidTire.cpp
    wheeled_vehicle/tire/ChForceElementTire.h
    wheeled_vehicle/tire/ChForceElementTire.cpp
    wheeled_vehicle/tire/ChPac89Tire.h
    wheeled_vehicle/tire/ChPac89Tire.cpp
    wheeled_vehicle/tire/ChFialaTire.h
    wheeled_vehicle/tire/ChFialaTire.cpp
    wheeled_vehicle/tire/ChTMsimpleTire.h
    wheeled_vehicle/tire/ChTMsimpleTire.cpp
    wheeled_vehicle/tire/ChTMeasyTire.h
    wheeled_vehicle/tire/ChTMeasyTire.cpp
    wheeled_vehicle/tire/ChDeformableTire.h
    wheeled_vehicle/tire/ChDeformableTire.cpp
    wheeled_vehicle/tire/ChANCFTire.h
    wheeled_vehicle/tire/ChANCFTire.cpp
    wheeled_vehicle/tire/ChReissnerTire.h
    wheeled_vehicle/tire/ChReissnerTire.cpp
    wheeled_vehicle/tire/ChFEATire.h
    wheeled_vehicle/tire/ChFEATire.cpp
    wheeled_vehicle/tire/ChPac02Tire.h
    wheeled_vehicle/tire/ChPac02Tire.cpp

    wheeled_vehicle/tire/RigidTire.h
    wheeled_vehicle/tire/RigidTire.cpp
    wheeled_vehicle/tire/Pac89Tire.h
    wheeled_vehicle/tire/Pac89Tire.cpp
    wheeled_vehicle/tire/FialaTire.h
    wheeled_vehicle/tire/FialaTire.cpp
    wheeled_vehicle/tire/TMeasyTire.h
    wheeled_vehicle/tire/TMeasyTire.cpp
    wheeled_vehicle/tire/TMsimpleTire.h
    wheeled_vehicle/tire/TMsimpleTire.cpp
    wheeled_vehicle/tire/ReissnerTire.h
    wheeled_vehicle/tire/ReissnerTire.cpp
    wheeled_vehicle/tire/ANCFTire.h
    wheeled_vehicle/tire/ANCFTire.cpp
    wheeled_vehicle/tire/FEATire.h
    wheeled_vehicle/tire/FEATire.cpp
    wheeled_vehicle/tire/ANCFToroidalTire.h
    wheeled_vehicle/tire/ANCFToroidalTire.cpp
    wheeled_vehicle/tire/ReissnerToroidalTire.h
    wheeled_vehicle/tire/ReissnerToroidalTire.cpp
    wheeled_vehicle/tire/Pac02Tire.h
    wheeled_vehicle/tire/Pac02Tire.cpp
)
source_group("wheeled_vehicle\\tire" FILES ${CV_WV_TIRE_FILES})

set(CV_WV_VEHICLE_FILES
    wheeled_vehicle/vehicle/WheeledTrailer.h
    wheeled_vehicle/vehicle/WheeledTrailer.cpp
    wheeled_vehicle/vehicle/WheeledVehicle.h
    wheeled_vehicle/vehicle/WheeledVehicle.cpp
)
source_group("wheeled_vehicle\\vehicle" FILES ${CV_WV_VEHICLE_FILES})

set(CV_WV_WHEEL_FILES
    wheeled_vehicle/wheel/Wheel.h
    wheeled_vehicle/wheel/Wheel.cpp
)
source_group("wheeled_vehicle\\wheel" FILES ${CV_WV_WHEEL_FILES})

# --------------- TRACKED VEHICLE FILES

set(CV_TV_BASE_FILES
    tracked_vehicle/ChIdler.h
    tracked_vehicle/ChIdler.cpp
    tracked_vehicle/ChTrackWheel.h 
    tracked_vehicle/ChTrackWheel.cpp 
    tracked_vehicle/ChTrackSuspension.h
    tracked_vehicle/ChTrackSuspension.cpp
    tracked_vehicle/ChSprocket.h
    tracked_vehicle/ChSprocket.cpp
    tracked_vehicle/ChTrackAssembly.h
    tracked_vehicle/ChTrackAssembly.cpp
    tracked_vehicle/ChTrackBrake.h
    tracked_vehicle/ChTrackBrake.cpp
    tracked_vehicle/ChDrivelineTV.h
    tracked_vehicle/ChDrivelineTV.cpp
    tracked_vehicle/ChTrackedVehicle.h
    tracked_vehicle/ChTrackedVehicle.cpp
    tracked_vehicle/ChTrackShoe.h
    tracked_vehicle/ChTrackShoe.cpp
    tracked_vehicle/ChTrackContactManager.h
    tracked_vehicle/ChTrackContactManager.cpp
)
source_group("tracked_vehicle\\base" FILES ${CV_TV_BASE_FILES})

set(CV_TV_BRAKE_FILES
    tracked_vehicle/brake/ChTrackBrakeSimple.h
    tracked_vehicle/brake/ChTrackBrakeSimple.cpp
    tracked_vehicle/brake/ChTrackBrakeShafts.h
    tracked_vehicle/brake/ChTrackBrakeShafts.cpp

    tracked_vehicle/brake/TrackBrakeSimple.h
    tracked_vehicle/brake/TrackBrakeSimple.cpp
    tracked_vehicle/brake/TrackBrakeShafts.h
    tracked_vehicle/brake/TrackBrakeShafts.cpp
)
source_group("tracked_vehicle\\brake" FILES ${CV_TV_BRAKE_FILES})

set(CV_TV_DRIVELINE_FILES
    tracked_vehicle/driveline/ChTrackDrivelineBDS.h
    tracked_vehicle/driveline/ChTrackDrivelineBDS.cpp
    tracked_vehicle/driveline/ChSimpleTrackDriveline.h
    tracked_vehicle/driveline/ChSimpleTrackDriveline.cpp

    tracked_vehicle/driveline/TrackDrivelineBDS.h
    tracked_vehicle/driveline/TrackDrivelineBDS.cpp
    tracked_vehicle/driveline/SimpleTrackDriveline.h
    tracked_vehicle/driveline/SimpleTrackDriveline.cpp
)
source_group("tracked_vehicle\\driveline" FILES ${CV_TV_DRIVELINE_FILES})

set(CV_TV_IDLER_FILES
    tracked_vehicle/idler/ChTranslationalIdler.h
    tracked_vehicle/idler/ChTranslationalIdler.cpp
    tracked_vehicle/idler/ChDistanceIdler.h
    tracked_vehicle/idler/ChDistanceIdler.cpp

    tracked_vehicle/idler/TranslationalIdler.h
    tracked_vehicle/idler/TranslationalIdler.cpp
    tracked_vehicle/idler/DistanceIdler.h
    tracked_vehicle/idler/DistanceIdler.cpp
)
source_group("tracked_vehicle\\idler" FILES ${CV_TV_IDLER_FILES})

set(CV_TV_TRACKWHEEL_FILES
    tracked_vehicle/track_wheel/ChSingleTrackWheel.h
    tracked_vehicle/track_wheel/ChSingleTrackWheel.cpp
    tracked_vehicle/track_wheel/ChDoubleTrackWheel.h
    tracked_vehicle/track_wheel/ChDoubleTrackWheel.cpp

    tracked_vehicle/track_wheel/SingleTrackWheel.h
    tracked_vehicle/track_wheel/SingleTrackWheel.cpp
    tracked_vehicle/track_wheel/DoubleTrackWheel.h
    tracked_vehicle/track_wheel/DoubleTrackWheel.cpp
)
source_group("tracked_vehicle\\track_wheel" FILES ${CV_TV_TRACKWHEEL_FILES})

set(CV_TV_SPROCKET_FILES
    tracked_vehicle/sprocket/ChSprocketSinglePin.h
    tracked_vehicle/sprocket/ChSprocketSinglePin.cpp
    tracked_vehicle/sprocket/ChSprocketDoublePin.h
    tracked_vehicle/sprocket/ChSprocketDoublePin.cpp
    tracked_vehicle/sprocket/ChSprocketBand.h
    tracked_vehicle/sprocket/ChSprocketBand.cpp

    tracked_vehicle/sprocket/SprocketSinglePin.h
    tracked_vehicle/sprocket/SprocketSinglePin.cpp
    tracked_vehicle/sprocket/SprocketDoublePin.h
    tracked_vehicle/sprocket/SprocketDoublePin.cpp
    tracked_vehicle/sprocket/SprocketBand.h
    tracked_vehicle/sprocket/SprocketBand.cpp
)
source_group("tracked_vehicle\\sprocket" FILES ${CV_TV_SPROCKET_FILES})

set(CV_TV_SUSPENSION_FILES
    tracked_vehicle/suspension/ChTranslationalDamperSuspension.h
    tracked_vehicle/suspension/ChTranslationalDamperSuspension.cpp
    tracked_vehicle/suspension/ChRotationalDamperSuspension.h
    tracked_vehicle/suspension/ChRotationalDamperSuspension.cpp

    tracked_vehicle/suspension/TranslationalDamperSuspension.h
    tracked_vehicle/suspension/TranslationalDamperSuspension.cpp
    tracked_vehicle/suspension/RotationalDamperSuspension.h
    tracked_vehicle/suspension/RotationalDamperSuspension.cpp
)
source_group("tracked_vehicle\\suspension" FILES ${CV_TV_SUSPENSION_FILES})

set(CV_TV_TRACKSHOE_FILES
    tracked_vehicle/track_shoe/ChTrackShoeSegmented.h
    tracked_vehicle/track_shoe/ChTrackShoeSegmented.cpp
    tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h
    tracked_vehicle/track_shoe/ChTrackShoeSinglePin.cpp
    tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h
    tracked_vehicle/track_shoe/ChTrackShoeDoublePin.cpp
    tracked_vehicle/track_shoe/ChTrackShoeBand.h
    tracked_vehicle/track_shoe/ChTrackShoeBand.cpp
    tracked_vehicle/track_shoe/ChTrackShoeBandBushing.h
    tracked_vehicle/track_shoe/ChTrackShoeBandBushing.cpp
    tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h
    tracked_vehicle/track_shoe/ChTrackShoeBandANCF.cpp

    tracked_vehicle/track_shoe/TrackShoeSinglePin.h
    tracked_vehicle/track_shoe/TrackShoeSinglePin.cpp
    tracked_vehicle/track_shoe/TrackShoeDoublePin.h
    tracked_vehicle/track_shoe/TrackShoeDoublePin.cpp
    tracked_vehicle/track_shoe/TrackShoeBandBushing.h
    tracked_vehicle/track_shoe/TrackShoeBandBushing.cpp
    tracked_vehicle/track_shoe/TrackShoeBandANCF.h
    tracked_vehicle/track_shoe/TrackShoeBandANCF.cpp
)
source_group("tracked_vehicle\\track_shoe" FILES ${CV_TV_TRACKSHOE_FILES})

set(CV_TV_TRACKASSEMBLY_FILES
    tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h
    tracked_vehicle/track_assembly/ChTrackAssemblySegmented.cpp
    tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h
    tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.cpp
    tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h
    tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.cpp
    tracked_vehicle/track_assembly/ChTrackAssemblyBand.h
    tracked_vehicle/track_assembly/ChTrackAssemblyBand.cpp
    tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.h
    tracked_vehicle/track_assembly/ChTrackAssemblyBandBushing.cpp
    tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.h
    tracked_vehicle/track_assembly/ChTrackAssemblyBandANCF.cpp

    tracked_vehicle/track_assembly/TrackAssemblySinglePin.h
    tracked_vehicle/track_assembly/TrackAssemblySinglePin.cpp
    tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h
    tracked_vehicle/track_assembly/TrackAssemblyDoublePin.cpp
    tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h
    tracked_vehicle/track_assembly/TrackAssemblyBandBushing.cpp
    tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h
    tracked_vehicle/track_assembly/TrackAssemblyBandANCF.cpp
)
source_group("tracked_vehicle\\track_assembly" FILES ${CV_TV_TRACKASSEMBLY_FILES})

set(CV_TV_TEST_RIG_FILES
    tracked_vehicle/test_rig/ChTrackTestRig.h
    tracked_vehicle/test_rig/ChTrackTestRig.cpp
    tracked_vehicle/test_rig/ChTrackTestRigDriver.h
    tracked_vehicle/test_rig/ChTrackTestRigDriver.cpp
    tracked_vehicle/test_rig/ChTrackTestRigDataDriver.h
    tracked_vehicle/test_rig/ChTrackTestRigDataDriver.cpp
    tracked_vehicle/test_rig/ChTrackTestRigRoadDriver.h
    tracked_vehicle/test_rig/ChTrackTestRigRoadDriver.cpp
    tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.h
    tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.cpp
)
source_group("tracked_vehicle\\test_rig" FILES ${CV_TV_TEST_RIG_FILES})

set(CV_TV_UTILS_FILES
)

set(CV_TV_VEHICLE_FILES
    tracked_vehicle/vehicle/TrackedVehicle.h
    tracked_vehicle/vehicle/TrackedVehicle.cpp
)
source_group("tracked_vehicle\\vehicle" FILES ${CV_TV_VEHICLE_FILES})

# --------------- THIRD-PARTY FILES

set(CV_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("utils\\stb" FILES ${CV_STB_FILES})

# --------------- VEHICLE IRRLICHT INTERFACE

if(CH_ENABLE_MODULE_IRRLICHT)
    set(CVIRR_FILES
        ChVehicleVisualSystemIrrlicht.h
        ChVehicleVisualSystemIrrlicht.cpp
    )
    set(CVIRR_WV_FILES
        wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h
        wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.cpp
        wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h
        wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.cpp
    )
    set(CVIRR_TV_FILES
        tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h
        tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.cpp
        tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.h
        tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.cpp
    )
else()
    set(CVIRR_FILES "")
    set(CVIRR_WV_FILES "")
    set(CVIRR_TV_FILES "")
endif()

source_group("" FILES ${CVIRR_FILES})
source_group("wheeled_vehicle" FILES ${CVIRR_WV_FILES})
source_group("tracked_vehicle" FILES ${CVIRR_TV_FILES})

# --------------- VEHICLE VSG INTERFACE

if(CH_ENABLE_MODULE_VSG)
    set(CVVSG_FILES
        ChVehicleVisualSystemVSG.h
        ChVehicleVisualSystemVSG.cpp
    )
    set(CVVSG_WV_FILES
        wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h
        wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.cpp
        wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.h
        wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.cpp    
    )
    set(CVVSG_TV_FILES
        tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h
        tracked_vehicle/ChTrackedVehicleVisualSystemVSG.cpp
        tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.h
        tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.cpp
    )
else()
    set(CVVSG_FILES "")
    set(CVVSG_WV_FILES "")
    set(CVVSG_TV_FILES "")
endif()

source_group("" FILES ${CVVSG_FILES})
source_group("wheeled_vehicle" FILES ${CVVSG_WV_FILES})
source_group("tracked_vehicle" FILES ${CVVSG_TV_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_VEHICLE "")
if(CH_ENABLE_MODULE_IRRLICHT)
  list (APPEND DEPENDENCIES_VEHICLE "IRRLICHT")
endif()
if(CH_ENABLE_MODULE_VSG)
  list (APPEND DEPENDENCIES_VEHICLE "VSG")
endif()
if(CH_ENABLE_MODULE_FSI)
  list (APPEND DEPENDENCIES_VEHICLE "FSI")
endif()

set(DEPENDENCIES_VEHICLE ${DEPENDENCIES_VEHICLE} PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Add the Chrono_vehicle library
# ----------------------------------------------------------------------------

message(STATUS "Add Chrono_vehicle library")

add_library(Chrono_vehicle
#
    ${CV_BASE_FILES}
    ${CV_CHASSIS_FILES}
    ${CV_DRIVER_FILES}
    ${CV_POVERTRAIN_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_UTILS_FILES}
    ${CV_OUTPUT_FILES}
    ${CVHDF5_OUTPUT_FILES}
#
    ${CV_WV_BASE_FILES}
    ${CV_WV_ANTIROLL_FILES}
    ${CV_WV_BRAKE_FILES}
    ${CV_WV_DRIVELINE_FILES}
    ${CV_WV_STEERING_FILES}
    ${CV_WV_SUBCHASSIS_FILES}
    ${CV_WV_SUSPENSION_FILES}
    ${CV_WV_TEST_RIG_FILES}
    ${CV_WV_TIRE_FILES}
    ${CV_WV_VEHICLE_FILES}
    ${CV_WV_WHEEL_FILES}
#
    ${CV_TV_BASE_FILES}
    ${CV_TV_BRAKE_FILES}
    ${CV_TV_DRIVELINE_FILES}
    ${CV_TV_IDLER_FILES}
    ${CV_TV_TRACKWHEEL_FILES}
    ${CV_TV_SPROCKET_FILES}
    ${CV_TV_SUSPENSION_FILES}
    ${CV_TV_TRACKSHOE_FILES}
    ${CV_TV_TRACKASSEMBLY_FILES}
    ${CV_TV_TEST_RIG_FILES}
    ${CV_TV_UTILS_FILES}
    ${CV_TV_VEHICLE_FILES}
#
    ${CV_STB_FILES}
)

add_library(Chrono::vehicle ALIAS Chrono_vehicle)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_vehicle PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_vehicle PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_vehicle PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_vehicle PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_vehicle PUBLIC Chrono_core)

if(HAVE_OPENCRG)
  target_link_libraries(Chrono_vehicle PRIVATE OpenCRG::OpenCRG)
endif()

if(CH_ENABLE_MODULE_FSI)
  target_link_libraries(Chrono_vehicle PRIVATE Chrono_fsisph)
endif()

target_compile_definitions(Chrono_vehicle INTERFACE 
    $<BUILD_INTERFACE:CHRONO_VEHICLE_DATA_DIR=\"${PROJECT_BINARY_DIR}/bin/data/vehicle/\">
    $<INSTALL_INTERFACE:CHRONO_VEHICLE_DATA_DIR=\"$<INSTALL_PREFIX>/data/vehicle/\">)

target_compile_definitions(Chrono_vehicle PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_VEHICLE>)

if(HDF5_FOUND)
    target_compile_definitions(Chrono_vehicle PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${HDF5_COMPILE_DEFS}>)
    target_include_directories(Chrono_vehicle PUBLIC ${HDF5_INCLUDE_DIRS})
    target_link_libraries(Chrono_vehicle PUBLIC ${HDF5_CXX_LIBRARIES})
endif()

install(TARGETS Chrono_vehicle
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle)

#-------------------------------------------------------------------------------
# Add the Chrono_vehicle_irrlicht library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_IRRLICHT)
    message(STATUS "Add Chrono_vehicle_irrlicht library")

    add_library(Chrono_vehicle_irrlicht
        ${CVIRR_FILES}
        ${CVIRR_WV_FILES}
        ${CVIRR_TV_FILES}
    )
    add_library(Chrono::vehicle_irrlicht ALIAS Chrono_vehicle_irrlicht)

    if(CH_WHOLE_PROG_OPT)
      set_target_properties(Chrono_vehicle_irrlicht PROPERTIES COMPILE_FLAGS "/GL")
      set_target_properties(Chrono_vehicle_irrlicht PROPERTIES LINK_FLAGS "/LTCG")
    endif()

    if (CH_STATIC)
      set_target_properties(Chrono_vehicle_irrlicht PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()

    if(MSVC)
      set_target_properties(Chrono_vehicle_irrlicht PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_link_libraries(Chrono_vehicle_irrlicht PRIVATE Chrono_core Chrono_vehicle)
    target_link_libraries(Chrono_vehicle_irrlicht PUBLIC Chrono_irrlicht)

    if(CH_ENABLE_IRRKLANG)
        target_link_libraries(Chrono_vehicle_irrlicht PUBLIC IrrKlang::IrrKlang)
    endif()

    target_compile_definitions(Chrono_vehicle_irrlicht PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_VEHICLE>)

    install(TARGETS Chrono_vehicle_irrlicht
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_vehicle) # TODO DARIOM: verify if correct folder

endif()

#-------------------------------------------------------------------------------
# Add the Chrono_vehicle_vsg library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
    message(STATUS "Add Chrono_vehicle_vsg library")

    add_library(Chrono_vehicle_vsg
        ${CVVSG_FILES}
        ${CVVSG_WV_FILES}
        ${CVVSG_TV_FILES}
    )
    add_library(Chrono::vehicle_vsg ALIAS Chrono_vehicle_vsg)

    if(CH_WHOLE_PROG_OPT)
      set_target_properties(Chrono_vehicle_vsg PROPERTIES COMPILE_FLAGS "/GL")
      set_target_properties(Chrono_vehicle_vsg PROPERTIES LINK_FLAGS "/LTCG")
    endif()

    if (CH_STATIC)
      set_target_properties(Chrono_vehicle_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()

    if(MSVC)
      set_target_properties(Chrono_vehicle_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_vehicle_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_VEHICLE>)

    target_link_libraries(Chrono_vehicle_vsg PRIVATE Chrono_core Chrono_vehicle)
    target_link_libraries(Chrono_vehicle_vsg PUBLIC Chrono_vsg)

    install(TARGETS Chrono_vehicle_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_vehicle) # TODO DARIOM: verify if correct folder

endif()

#-------------------------------------------------------------------------------
# Trigger CMake for vehicle cosimulation
#-------------------------------------------------------------------------------

option(CH_ENABLE_MODULE_VEHICLE_COSIM "Build the Chrono vehicle cosimulation library" TRUE)
mark_as_advanced(FORCE CH_ENABLE_MODULE_VEHICLE_COSIM)
if(CH_ENABLE_MODULE_VEHICLE_COSIM)
  add_subdirectory(cosim)
endif()

set(CH_ENABLE_MODULE_VEHICLE_COSIM  "${CH_ENABLE_MODULE_VEHICLE_COSIM}" PARENT_SCOPE)
set(DEPENDENCIES_VEHICLECOSIM ${DEPENDENCIES_VEHICLECOSIM} PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Trigger CMake for vehicle FMUs
#-------------------------------------------------------------------------------

cmake_dependent_option(CH_ENABLE_MODULE_VEHICLE_FMI  "Enable FMI support for Chrono::Vehicle" ON "CH_ENABLE_MODULE_FMI" OFF)
mark_as_advanced(FORCE CH_ENABLE_MODULE_VEHICLE_FMI)
if(CH_ENABLE_MODULE_VEHICLE_FMI)
  add_subdirectory(fmi)
endif()

set(CH_ENABLE_MODULE_VEHICLE_FMI  "${CH_ENABLE_MODULE_VEHICLE_FMI}" PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vehicle
        FILES_MATCHING PATTERN "*.h")

# Install 3rd party headers       
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

# Install files for chrono-config (in both build and install trees)
file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindIrrKlang.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindIrrKlang.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})

# On Windows, copy DLLs
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(CH_ENABLE_MODULE_IRRLICHT AND EXISTS "${IRRKLANG_DLL}")
        add_custom_command(TARGET Chrono_vehicle_irrlicht POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${IRRKLANG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

    if(EXISTS "${OpenCRG_DLL}")
        add_custom_command(TARGET Chrono_vehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OpenRG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${OpenCRG_DLL}" DESTINATION bin)
    endif()

  # Make variable visible from outside this directory
  set(CH_IRRKLANG_DLL "${IRRKLANG_DLL}" PARENT_SCOPE)
  set(CH_OPENCRG_DLL "${OpenCRG_DLL}" PARENT_SCOPE)
endif()
