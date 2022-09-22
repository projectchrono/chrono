#=============================================================================
# CMake configuration file for Chrono Vehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_VEHICLE)
   mark_as_advanced(ENABLE_OPENCRG)
   mark_as_advanced(FORCE ENABLE_IRRKLANG)
   mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
   mark_as_advanced(FORCE CH_IRRKLANGLIB)
   return()
endif()

message(STATUS "\n==== Chrono Vehicle module ====\n")

mark_as_advanced(CLEAR ENABLE_OPENCRG)
mark_as_advanced(CLEAR ENABLE_IRRKLANG)
mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
mark_as_advanced(CLEAR CH_IRRKLANGLIB)

# Provide option to add OpenCRG support.
option(ENABLE_OPENCRG "Enable OpenCRG terrain library support" OFF)

# If Irrlicht support was enabled, provide option to add Irrklang support.
cmake_dependent_option(ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
                       "ENABLE_MODULE_IRRLICHT" OFF)

# ----------------------------------------------------------------------------
# Find the OpenCRG library
# ----------------------------------------------------------------------------
if (ENABLE_OPENCRG)
   
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(OPENCRG_INCLUDE_DIR ""  CACHE PATH   "The directory where crgBaseLib.h resides")
      set(OPENCRG_LIBRARY   "" CACHE FILEPATH "The OpenCRG library")
      set(OPENCRG_DLL  "" CACHE FILEPATH "The OpenCRG DLL")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      find_path(OPENCRG_INCLUDE_DIR NAMES crgBaseLib.h PATHS "/usr/local/include")
      find_library(OPENCRG_LIBRARY NAMES OpenCRG PATHS "/usr/local/lib")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      find_path(OPENCRG_INCLUDE_DIR NAMES crgBaseLib.h PATHS "/usr/local/include")
      find_library(OPENCRG_LIBRARY NAMES OpenCRG PATHS "/usr/local/lib")
   endif()

   mark_as_advanced(CLEAR OPENCRG_INCLUDE_DIR)
   mark_as_advanced(CLEAR OPENCRG_LIBRARY)
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
     mark_as_advanced(CLEAR OPENCRG_DLL)
   endif()

   if(EXISTS "${OPENCRG_INCLUDE_DIR}/crgBaseLib.h" AND EXISTS "${OPENCRG_LIBRARY}")
     set(HAVE_OPENCRG ON)
   else()
     set(HAVE_OPENCRG OFF)
   endif()

else()

   mark_as_advanced(FORCE OPENCRG_INCLUDE_DIR)
   mark_as_advanced(FORCE OPENCRG_LIBRARY)
   if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
     mark_as_advanced(FORCE OPENCRG_DLL)
   endif()

   set(HAVE_OPENCRG OFF)

endif()

SET(HAVE_OPENCRG "${HAVE_OPENCRG}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------
if(ENABLE_IRRKLANG)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
      set(CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklang library?")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  endif()

  if(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  else()
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  endif()

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
    elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
    endif()
  endif()

  mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
  mark_as_advanced(CLEAR CH_IRRKLANGLIB)

else()

  mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
  mark_as_advanced(FORCE CH_IRRKLANGLIB)

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables
if(HAVE_OPENCRG)
  set(CHRONO_OPENCRG "#define CHRONO_OPENCRG")
else()
  set(CHRONO_OPENCRG "#undef CHRONO_OPENCRG")
endif()

if(ENABLE_IRRKLANG)
  set(CHRONO_IRRKLANG "#define CHRONO_IRRKLANG")
else()
  set(CHRONO_IRRKLANG "#undef CHRONO_IRRKLANG")
endif()

# Generate the configuration header file using substitution variables.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigVehicle.h.in
               ${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicle.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicle.h"
        DESTINATION include/chrono_vehicle)

# ----------------------------------------------------------------------------
# List the files in the ChronoEngine_vehicle library
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
    ChPowertrain.h
    ChPowertrain.cpp
    ChSubsysDefs.h
    ChSubsysDefs.cpp
    ChTerrain.h
    ChTerrain.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChVehicleGeometry.h
    ChVehicleGeometry.cpp
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
    chassis/ChChassisConnectorTorsion.h
    chassis/ChChassisConnectorTorsion.cpp

    chassis/RigidChassis.h
    chassis/RigidChassis.cpp

    chassis/ChassisConnectorHitch.h
    chassis/ChassisConnectorHitch.cpp
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

    driver/AIDriver.h
    driver/AIDriver.cpp
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_DRIVER_FILES
        driver/ChIrrGuiDriver.h
        driver/ChIrrGuiDriver.cpp
    )
else()
    set(CVIRR_DRIVER_FILES "")
endif()
source_group("driver" FILES ${CV_DRIVER_FILES} ${CVIRR_DRIVER_FILES})

set(CV_POVERTRAIN_FILES
    powertrain/ChSimplePowertrain.h
    powertrain/ChSimplePowertrain.cpp
    powertrain/ChShaftsPowertrain.h
    powertrain/ChShaftsPowertrain.cpp
    powertrain/ChSimpleMapPowertrain.h
    powertrain/ChSimpleMapPowertrain.cpp
    powertrain/ChSimpleCVTPowertrain.h
    powertrain/ChSimpleCVTPowertrain.cpp

    powertrain/SimpleMapPowertrain.h
    powertrain/SimpleMapPowertrain.cpp
    powertrain/SimplePowertrain.h
    powertrain/SimplePowertrain.cpp
    powertrain/SimpleCVTPowertrain.h
    powertrain/SimpleCVTPowertrain.cpp
    powertrain/ShaftsPowertrain.h
    powertrain/ShaftsPowertrain.cpp
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
    terrain/SCMDeformableTerrain.h
    terrain/SCMDeformableTerrain.cpp
    terrain/GranularTerrain.h
    terrain/GranularTerrain.cpp
    terrain/FEADeformableTerrain.h
    terrain/FEADeformableTerrain.cpp
)
if(HAVE_OPENCRG)
    set(CV_OPENCRG_FILES
        terrain/CRGTerrain.h
        terrain/CRGTerrain.cpp
    )
else()
    set(CV_OPENCRG_FILES "")
endif()
source_group("terrain" FILES ${CV_TERRAIN_FILES} ${CV_OPENCRG_FILES})

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
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_UTILS_FILES
        utils/ChVehicleVisualSystemIrrlicht.h
        utils/ChVehicleVisualSystemIrrlicht.cpp
    )
else()
    set(CVIRR_UTILS_FILES "")
endif()
source_group("utils" FILES ${CV_UTILS_FILES} ${CVIRR_UTILS_FILES})


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
    wheeled_vehicle/suspension/ChSAELeafspringAxle.h
    wheeled_vehicle/suspension/ChSAELeafspringAxle.cpp
    wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.h
    wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ChSolidThreeLinkAxle.h
    wheeled_vehicle/suspension/ChSolidThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/ChSAEToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ChSAEToeBarLeafspringAxle.cpp

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
    wheeled_vehicle/suspension/SAELeafspringAxle.h
    wheeled_vehicle/suspension/SAELeafspringAxle.cpp
    wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h
    wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.cpp
    wheeled_vehicle/suspension/SolidThreeLinkAxle.h
    wheeled_vehicle/suspension/SolidThreeLinkAxle.cpp
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.cpp
    wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.cpp
)
source_group("wheeled_vehicle\\suspension" FILES ${CV_WV_SUSPENSION_FILES})

set(CV_WV_TEST_RIG_FILES
    wheeled_vehicle/test_rig/ChSuspensionTestRig.h
    wheeled_vehicle/test_rig/ChSuspensionTestRig.cpp
    wheeled_vehicle/test_rig/ChDriverSTR.h
    wheeled_vehicle/test_rig/ChDriverSTR.cpp
    wheeled_vehicle/test_rig/ChDataDriverSTR.h
    wheeled_vehicle/test_rig/ChDataDriverSTR.cpp
    wheeled_vehicle/test_rig/ChTireTestRig.h
    wheeled_vehicle/test_rig/ChTireTestRig.cpp
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_TEST_RIG_FILES
        wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h
        wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.cpp    
    )
else()
    set(CVIRR_WV_TEST_RIG_FILES "")
endif()
source_group("wheeled_vehicle\\test_rig" FILES ${CV_WV_TEST_RIG_FILES} ${CVIRR_WV_TEST_RIG_FILES})

set(CV_WV_TIRE_FILES
    wheeled_vehicle/tire/ChRigidTire.h
    wheeled_vehicle/tire/ChRigidTire.cpp
    wheeled_vehicle/tire/ChForceElementTire.h
    wheeled_vehicle/tire/ChForceElementTire.cpp
    wheeled_vehicle/tire/ChPacejkaTire.h
    wheeled_vehicle/tire/ChPacejkaTire.cpp
    wheeled_vehicle/tire/ChPac02Tire.h
    wheeled_vehicle/tire/ChPac02Tire.cpp
    wheeled_vehicle/tire/ChPac89Tire.h
    wheeled_vehicle/tire/ChPac89Tire.cpp
    wheeled_vehicle/tire/ChLugreTire.h
    wheeled_vehicle/tire/ChLugreTire.cpp
    wheeled_vehicle/tire/ChFialaTire.h
    wheeled_vehicle/tire/ChFialaTire.cpp
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

    wheeled_vehicle/tire/RigidTire.h
    wheeled_vehicle/tire/RigidTire.cpp
    wheeled_vehicle/tire/PacejkaTire.h
    wheeled_vehicle/tire/PacejkaTire.cpp
    wheeled_vehicle/tire/Pac02Tire.h
    wheeled_vehicle/tire/Pac02Tire.cpp
    wheeled_vehicle/tire/Pac89Tire.h
    wheeled_vehicle/tire/Pac89Tire.cpp
    wheeled_vehicle/tire/LugreTire.h
    wheeled_vehicle/tire/LugreTire.cpp
    wheeled_vehicle/tire/FialaTire.h
    wheeled_vehicle/tire/FialaTire.cpp
    wheeled_vehicle/tire/TMeasyTire.h
    wheeled_vehicle/tire/TMeasyTire.cpp
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
)
source_group("wheeled_vehicle\\tire" FILES ${CV_WV_TIRE_FILES})

set(CV_WV_UTILS_FILES
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_UTILS_FILES
        wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h
        wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.cpp
    )
else()
    set(CVIRR_WV_UTILS_FILES "")
endif()
source_group("wheeled_vehicle\\utils" FILES ${CV_WV_UTILS_FILES} ${CVIRR_WV_UTILS_FILES})

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
    tracked_vehicle/test_rig/ChDriverTTR.h
    tracked_vehicle/test_rig/ChDriverTTR.cpp
    tracked_vehicle/test_rig/ChDataDriverTTR.h
    tracked_vehicle/test_rig/ChDataDriverTTR.cpp
    tracked_vehicle/test_rig/ChRoadDriverTTR.h
    tracked_vehicle/test_rig/ChRoadDriverTTR.cpp
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_TV_TEST_RIG_FILES
        tracked_vehicle/test_rig/ChIrrGuiDriverTTR.h
        tracked_vehicle/test_rig/ChIrrGuiDriverTTR.cpp
    )
else()
    set(CVIRR_TV_TEST_RIG_FILES "")
endif()
source_group("tracked_vehicle\\test_rig" FILES ${CV_TV_TEST_RIG_FILES} ${CVIRR_TV_TEST_RIG_FILES})

set(CV_TV_UTILS_FILES
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_TV_UTILS_FILES
        tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h
        tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.cpp
        tracked_vehicle/utils/ChTrackTestRigVisualSystemIrrlicht.h
        tracked_vehicle/utils/ChTrackTestRigVisualSystemIrrlicht.cpp
    )
else()
    set(CVIRR_TV_UTILS_FILES "")
endif()
source_group("tracked_vehicle\\utils" FILES ${CV_TV_UTILS_FILES} ${CVIRR_TV_UTILS_FILES})

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

# ----------------------------------------------------------------------------
# Add the ChronoEngine_vehicle library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(COMPILE_DEFS "CH_API_COMPILE_VEHICLE")
set(LINK_FLAGS ${CH_LINKERFLAG_SHARED})
set(LIBRARIES "ChronoEngine")

if(HDF5_FOUND)
    set(COMPILE_DEFS "${COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
    include_directories(${HDF5_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${HDF5_CXX_LIBRARIES})
endif()

if(HAVE_OPENCRG)
    include_directories(${OPENCRG_INCLUDE_DIR})
    list(APPEND LIBRARIES ${OPENCRG_LIBRARY})
endif()

add_library(ChronoEngine_vehicle SHARED
#
    ${CV_BASE_FILES}
    ${CV_CHASSIS_FILES}
    ${CV_DRIVER_FILES}
    ${CV_POVERTRAIN_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_OPENCRG_FILES}
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
    ${CV_WV_UTILS_FILES}
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

message(STATUS "Add ChronoEngine_vehicle library")

set_target_properties(ChronoEngine_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_vehicle PRIVATE "CH_API_COMPILE_VEHICLE")
target_compile_definitions(ChronoEngine_vehicle PRIVATE "CH_IGNORE_DEPRECATED")
if(HDF5_FOUND)
target_compile_definitions(ChronoEngine_vehicle PRIVATE ${HDF5_COMPILE_DEFS})
endif()

target_link_libraries(ChronoEngine_vehicle ${LIBRARIES})

install(TARGETS ChronoEngine_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


#-------------------------------------------------------------------------------
# Add the ChronoEngine_vehicle_irrlicht library
#-------------------------------------------------------------------------------

if(ENABLE_MODULE_IRRLICHT)

set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
list(APPEND LIBRARIES ChronoEngine_vehicle)
list(APPEND LIBRARIES ChronoEngine_irrlicht) 
include_directories(${CH_IRRLICHTINC})

add_library(ChronoEngine_vehicle_irrlicht SHARED
    ${CVIRR_DRIVER_FILES}
    ${CVIRR_UTILS_FILES}
    ${CVIRR_WV_TEST_RIG_FILES}
    ${CVIRR_WV_UTILS_FILES}
    ${CVIRR_TV_TEST_RIG_FILES}
    ${CVIRR_TV_UTILS_FILES}
)

if(ENABLE_IRRKLANG)
    target_include_directories(ChronoEngine_vehicle_irrlicht PUBLIC ${CH_IRRKLANGINC})
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    set(CH_IRRLICHTINC ${CH_IRRLICHTINC} ${CH_IRRKLANGINC} PARENT_SCOPE)
    set(IRRLICHT_LIBRARY ${IRRLICHT_LIBRARY} ${CH_IRRKLANGLIB} PARENT_SCOPE)
endif()

set_target_properties(ChronoEngine_vehicle_irrlicht PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_vehicle_irrlicht PRIVATE "CH_API_COMPILE_VEHICLE")
target_compile_definitions(ChronoEngine_vehicle_irrlicht PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_vehicle_irrlicht ${LIBRARIES})

install(TARGETS ChronoEngine_vehicle_irrlicht
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

endif()

#-------------------------------------------------------------------------------
# Trigger CMake for vehicle cosimulation
#-------------------------------------------------------------------------------
add_subdirectory(cosim)

#-------------------------------------------------------------------------------
# Install Chrono::Vehicle headers
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vehicle
        FILES_MATCHING PATTERN "*.h")

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

#-------------------------------------------------------------------------------
# On Windows, copy DLLs (if specified)
#-------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    if(ENABLE_MODULE_IRRLICHT AND EXISTS "${CH_IRRKLANG_DLL}")
        add_custom_command(TARGET ChronoEngine_vehicle_irrlicht POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRKLANG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

    if(EXISTS "${OPENCRG_DLL}")
        add_custom_command(TARGET ChronoEngine_vehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPENCRG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${OPENCRG_DLL}" DESTINATION bin)
    endif()

endif()
