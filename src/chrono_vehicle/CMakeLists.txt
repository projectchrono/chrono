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

message(STATUS "==== Chrono Vehicle module ====")

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
    ChPowertrain.h
    ChPowertrain.cpp
    ChSubsysDefs.h
	ChSubsysDefs.cpp
    ChTerrain.h
    ChTerrain.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChVehicleModelData.h
    ChVehicleModelData.cpp
    ChVehicleOutput.h
)
source_group("base" FILES ${CV_BASE_FILES})

set(CV_CHASSIS_FILES
	chassis/ChRigidChassis.h
	chassis/ChRigidChassis.cpp

	chassis/RigidChassis.h
	chassis/RigidChassis.cpp
)
source_group("chassis" FILES ${CV_CHASSIS_FILES})

set(CV_DRIVER_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
    driver/ChPathFollowerDriver.h
    driver/ChPathFollowerDriver.cpp
    driver/ChPathFollowerACCDriver.h
    driver/ChPathFollowerACCDriver.cpp
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
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
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
        utils/ChVehicleIrrApp.h
        utils/ChVehicleIrrApp.cpp
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
    wheeled_vehicle/ChAntirollBar.h
    wheeled_vehicle/ChAntirollBar.cpp
    wheeled_vehicle/ChBrake.h
    wheeled_vehicle/ChBrake.cpp
    wheeled_vehicle/ChDriveline.h
    wheeled_vehicle/ChDriveline.cpp
    wheeled_vehicle/ChSuspension.h
    wheeled_vehicle/ChSuspension.cpp
    wheeled_vehicle/ChSteering.h
    wheeled_vehicle/ChSteering.cpp
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

    wheeled_vehicle/brake/BrakeSimple.h
    wheeled_vehicle/brake/BrakeSimple.cpp
)
source_group("wheeled_vehicle\\brake" FILES ${CV_WV_BRAKE_FILES})

set(CV_WV_DRIVELINE_FILES
    wheeled_vehicle/driveline/ChShaftsDriveline2WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline2WD.cpp
    wheeled_vehicle/driveline/ChShaftsDriveline4WD.h
    wheeled_vehicle/driveline/ChShaftsDriveline4WD.cpp
    wheeled_vehicle/driveline/ChSimpleDriveline.h
    wheeled_vehicle/driveline/ChSimpleDriveline.cpp

    wheeled_vehicle/driveline/ShaftsDriveline2WD.h
    wheeled_vehicle/driveline/ShaftsDriveline2WD.cpp
    wheeled_vehicle/driveline/ShaftsDriveline4WD.h
    wheeled_vehicle/driveline/ShaftsDriveline4WD.cpp
    wheeled_vehicle/driveline/SimpleDriveline.h
    wheeled_vehicle/driveline/SimpleDriveline.cpp
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

set(CV_WV_SUSPENSION_FILES
    wheeled_vehicle/suspension/ChDoubleWishbone.h
    wheeled_vehicle/suspension/ChDoubleWishbone.cpp
    wheeled_vehicle/suspension/ChDoubleWishboneReduced.h
    wheeled_vehicle/suspension/ChDoubleWishboneReduced.cpp
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
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ChToeBarLeafspringAxle.cpp

    wheeled_vehicle/suspension/DoubleWishbone.h
    wheeled_vehicle/suspension/DoubleWishbone.cpp
    wheeled_vehicle/suspension/DoubleWishboneReduced.h
    wheeled_vehicle/suspension/DoubleWishboneReduced.cpp
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
    wheeled_vehicle/suspension/LeafspringAxle.h
    wheeled_vehicle/suspension/LeafspringAxle.cpp
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.h
    wheeled_vehicle/suspension/ToeBarLeafspringAxle.cpp
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
    wheeled_vehicle/tire/ChPacejkaTire.h
    wheeled_vehicle/tire/ChPacejkaTire.cpp
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
    wheeled_vehicle/utils/ChWheeledVehicleAssembly.h
    wheeled_vehicle/utils/ChWheeledVehicleAssembly.cpp
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_UTILS_FILES
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.cpp
    )
else()
    set(CVIRR_WV_UTILS_FILES "")
endif()
source_group("wheeled_vehicle\\utils" FILES ${CV_WV_UTILS_FILES} ${CVIRR_WV_UTILS_FILES})

set(CV_WV_VEHICLE_FILES
    wheeled_vehicle/vehicle/WheeledVehicle.h
    wheeled_vehicle/vehicle/WheeledVehicle.cpp
)
source_group("wheeled_vehicle\\vehicle" FILES ${CV_WV_VEHICLE_FILES})

set(CV_WV_WHEEL_FILES
    wheeled_vehicle/wheel/Wheel.h
    wheeled_vehicle/wheel/Wheel.cpp
)
source_group("wheeled_vehicle\\wheel" FILES ${CV_WV_WHEEL_FILES})

#if(MPI_CXX_FOUND)
#    set(CV_WV_COSIM_FILES
#        wheeled_vehicle/cosim/ChCosimManager.h
#        wheeled_vehicle/cosim/ChCosimManager.cpp
#        wheeled_vehicle/cosim/ChCosimNode.h
#        wheeled_vehicle/cosim/ChCosimVehicleNode.h
#        wheeled_vehicle/cosim/ChCosimVehicleNode.cpp
#        wheeled_vehicle/cosim/ChCosimTireNode.h
#        wheeled_vehicle/cosim/ChCosimTireNode.cpp
#        wheeled_vehicle/cosim/ChCosimTerrainNode.h
#        wheeled_vehicle/cosim/ChCosimTerrainNode.cpp
#    )
#    source_group("wheeled_vehicle\\cosim" FILES ${CV_WV_COSIM_FILES})
#else()
    set(CV_WV_COSIM_FILES "")
#endif()

# --------------- TRACKED VEHICLE FILES

set(CV_TV_BASE_FILES
    tracked_vehicle/ChIdler.h
    tracked_vehicle/ChIdler.cpp
    tracked_vehicle/ChRoadWheel.h 
    tracked_vehicle/ChRoadWheel.cpp 
    tracked_vehicle/ChRoadWheelAssembly.h
    tracked_vehicle/ChRoadWheelAssembly.cpp
    tracked_vehicle/ChRoller.h
    tracked_vehicle/ChRoller.cpp
    tracked_vehicle/ChSprocket.h
    tracked_vehicle/ChSprocket.cpp
    tracked_vehicle/ChTrackAssembly.h
    tracked_vehicle/ChTrackAssembly.cpp
    tracked_vehicle/ChTrackBrake.h
    tracked_vehicle/ChTrackBrake.cpp
    tracked_vehicle/ChTrackDriveline.h
    tracked_vehicle/ChTrackDriveline.cpp
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

    tracked_vehicle/brake/TrackBrakeSimple.h
    tracked_vehicle/brake/TrackBrakeSimple.cpp
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
    tracked_vehicle/idler/ChSingleIdler.h
    tracked_vehicle/idler/ChSingleIdler.cpp
    tracked_vehicle/idler/ChDoubleIdler.h
    tracked_vehicle/idler/ChDoubleIdler.cpp

    tracked_vehicle/idler/SingleIdler.h
    tracked_vehicle/idler/SingleIdler.cpp
    tracked_vehicle/idler/DoubleIdler.h
    tracked_vehicle/idler/DoubleIdler.cpp
)
source_group("tracked_vehicle\\idler" FILES ${CV_TV_IDLER_FILES})

set(CV_TV_ROADWHEEL_FILES
    tracked_vehicle/road_wheel/ChSingleRoadWheel.h
    tracked_vehicle/road_wheel/ChSingleRoadWheel.cpp
    tracked_vehicle/road_wheel/ChDoubleRoadWheel.h
    tracked_vehicle/road_wheel/ChDoubleRoadWheel.cpp

    tracked_vehicle/road_wheel/SingleRoadWheel.h
    tracked_vehicle/road_wheel/SingleRoadWheel.cpp
    tracked_vehicle/road_wheel/DoubleRoadWheel.h
    tracked_vehicle/road_wheel/DoubleRoadWheel.cpp
)
source_group("tracked_vehicle\\road_wheel" FILES ${CV_TV_ROADWHEEL_FILES})

set(CV_TV_ROLLER_FILES
    tracked_vehicle/roller/ChDoubleRoller.h
    tracked_vehicle/roller/ChDoubleRoller.cpp

    tracked_vehicle/roller/DoubleRoller.h
    tracked_vehicle/roller/DoubleRoller.cpp
)
source_group("tracked_vehicle\\roller" FILES ${CV_TV_ROLLER_FILES})

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
    tracked_vehicle/suspension/ChLinearDamperRWAssembly.h
    tracked_vehicle/suspension/ChLinearDamperRWAssembly.cpp
    tracked_vehicle/suspension/ChRotationalDamperRWAssembly.h
    tracked_vehicle/suspension/ChRotationalDamperRWAssembly.cpp

    tracked_vehicle/suspension/LinearDamperRWAssembly.h
    tracked_vehicle/suspension/LinearDamperRWAssembly.cpp
    tracked_vehicle/suspension/RotationalDamperRWAssembly.h
    tracked_vehicle/suspension/RotationalDamperRWAssembly.cpp
)
source_group("tracked_vehicle\\suspension" FILES ${CV_TV_SUSPENSION_FILES})

set(CV_TV_TRACKSHOE_FILES
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
        tracked_vehicle/utils/ChTrackedVehicleIrrApp.h
        tracked_vehicle/utils/ChTrackedVehicleIrrApp.cpp
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

set(CV_EASYBMP_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP/EasyBMP.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP/EasyBMP.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP/EasyBMP_BMP.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP/EasyBMP_DataStructures.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/Easy_BMP/EasyBMP_VariousBMPutilities.h
)
source_group("easyBMP" FILES ${CV_EASYBMP_FILES})

# ----------------------------------------------------------------------------
# Add the ChronoEngine_vehicle library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(COMPILE_DEFS "CH_API_COMPILE_VEHICLE")
set(LINK_FLAGS ${CH_LINKERFLAG_SHARED})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht) 
    include_directories(${CH_IRRLICHTINC})
endif()

if(MPI_CXX_FOUND)
    set(CXX_FLAGS "${CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")
    set(LINK_FLAGS "${LINK_FLAGS} ${MPI_CXX_LINK_FLAGS}")
    include_directories(${MPI_CXX_INCLUDE_PATH})
    list(APPEND LIBRARIES ${MPI_CXX_LIBRARIES})
endif()

if(HDF5_FOUND)
    set(COMPILE_DEFS "${COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
    include_directories(${HDF5_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${HDF5_CXX_LIBRARIES})
endif()

if(HAVE_OPENCRG)
    include_directories(${OPENCRG_INCLUDE_DIR})
    list(APPEND LIBRARIES ${OPENCRG_LIBRARY})
endif()

if(ENABLE_IRRKLANG)
    include_directories(${CH_IRRKLANGINC})
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
endif()

add_library(ChronoEngine_vehicle SHARED
#
    ${CV_BASE_FILES}
    ${CV_CHASSIS_FILES}
    ${CV_DRIVER_FILES}
    ${CVIRR_DRIVER_FILES}
    ${CV_POVERTRAIN_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_OPENCRG_FILES}
    ${CV_UTILS_FILES}
    ${CVIRR_UTILS_FILES}
    ${CV_OUTPUT_FILES}
    ${CVHDF5_OUTPUT_FILES}
#
    ${CV_WV_BASE_FILES}
    ${CV_WV_ANTIROLL_FILES}
    ${CV_WV_BRAKE_FILES}
    ${CV_WV_DRIVELINE_FILES}
    ${CV_WV_STEERING_FILES}
    ${CV_WV_SUSPENSION_FILES}
    ${CV_WV_TEST_RIG_FILES}
    ${CVIRR_WV_TEST_RIG_FILES}
    ${CV_WV_TIRE_FILES}
    ${CV_WV_UTILS_FILES}
    ${CVIRR_WV_UTILS_FILES}
    ${CV_WV_VEHICLE_FILES}
    ${CV_WV_WHEEL_FILES}
    ${CV_WV_COSIM_FILES}
#
    ${CV_TV_BASE_FILES}
    ${CV_TV_BRAKE_FILES}
    ${CV_TV_DRIVELINE_FILES}
    ${CV_TV_IDLER_FILES}
    ${CV_TV_ROADWHEEL_FILES}
    ${CV_TV_ROLLER_FILES}
    ${CV_TV_SPROCKET_FILES}
    ${CV_TV_SUSPENSION_FILES}
    ${CV_TV_TRACKSHOE_FILES}
    ${CV_TV_TRACKASSEMBLY_FILES}
    ${CV_TV_TEST_RIG_FILES}
    ${CVIRR_TV_TEST_RIG_FILES}
    ${CV_TV_UTILS_FILES}
    ${CVIRR_TV_UTILS_FILES}
    ${CV_TV_VEHICLE_FILES}
#
    ${CV_EASYBMP_FILES}
)

set_target_properties(ChronoEngine_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_VEHICLE")

target_link_libraries(ChronoEngine_vehicle ${LIBRARIES})

install(TARGETS ChronoEngine_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vehicle
        FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# On Windows, copy DLLs (if specified)
#-------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    if(EXISTS ${CH_IRRKLANG_DLL})
        add_custom_command(TARGET ChronoEngine_vehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRKLANG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

    if(EXISTS ${OPENCRG_DLL})
        add_custom_command(TARGET ChronoEngine_vehicle POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPENCRG_DLL}"
            "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
    endif()

endif()
