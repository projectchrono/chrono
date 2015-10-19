#=============================================================================
# CMake configuration file for Chrono Vehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

message(STATUS "==== Chrono Vehicle module ====")

# If Irrlicht support was enabled, provide option to add Irrklang support.
cmake_dependent_option(ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
                       "ENABLE_MODULE_IRRLICHT" OFF)

# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------
if(ENABLE_IRRKLANG)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
      set(CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklank library?")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  endif()

  if(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  else()
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  endif()

  include_directories(${CH_IRRKLANGINC})

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

set(CV_BASE_FILES
    ChApiVehicle.h
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

set(CV_VEHICLE_FILES
    vehicle/Vehicle.h
    vehicle/Vehicle.cpp
)

set(CV_SUSPENSION_FILES
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

set(CV_WHEEL_FILES
    wheel/Wheel.h
    wheel/Wheel.cpp
)

set(CV_STEERING_FILES
    steering/ChPitmanArm.h
    steering/ChPitmanArm.cpp
    steering/ChRackPinion.h
    steering/ChRackPinion.cpp

    steering/PitmanArm.h
    steering/PitmanArm.cpp
    steering/RackPinion.h
    steering/RackPinion.cpp
)

set(CV_DRIVELINE_FILES
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

set(CV_DRIVER_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
    driver/ChPathFollowerDriver.h
    driver/ChPathFollowerDriver.cpp
)

set(CV_POVERTRAIN_FILES
    powertrain/ChSimplePowertrain.h
    powertrain/ChSimplePowertrain.cpp
    powertrain/ChShaftsPowertrain.h
    powertrain/ChShaftsPowertrain.cpp

    powertrain/SimplePowertrain.h
    powertrain/SimplePowertrain.cpp
    powertrain/ShaftsPowertrain.h
    powertrain/ShaftsPowertrain.cpp
)

set(CV_TIRE_FILES
    tire/ChRigidTire.h
    tire/ChRigidTire.cpp
    tire/ChPacejkaTire.h
    tire/ChPacejkaTire.cpp
    tire/ChLugreTire.h
    tire/ChLugreTire.cpp
    tire/ChFialaTire.h
    tire/ChFialaTire.cpp
	
    tire/RigidTire.h
    tire/RigidTire.cpp
    tire/LugreTire.h
    tire/LugreTire.cpp
    tire/FialaTire.h
    tire/FialaTire.cpp	
)

set(CV_BRAKE_FILES
    brake/ChBrakeSimple.h
    brake/ChBrakeSimple.cpp

    brake/BrakeSimple.h
    brake/BrakeSimple.cpp
)

set(CV_ANTIROLL_FILES
    antirollbar/ChAntirollBarRSD.h
    antirollbar/ChAntirollBarRSD.cpp

    antirollbar/AntirollBarRSD.h
    antirollbar/AntirollBarRSD.cpp
)

set(CV_TERRAIN_FILES
    terrain/FlatTerrain.h
    terrain/FlatTerrain.cpp
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
)

set(CV_SUSPENSIONTEST_FILES
    suspensionTest/SuspensionTest.h
    suspensionTest/SuspensionTest.cpp
)

set(CV_UTILS_FILES
    utils/ChWheeledVehicleAssembly.h
    utils/ChWheeledVehicleAssembly.cpp
    utils/ChSteeringController.h
    utils/ChSteeringController.cpp
    utils/ChSpeedController.h
    utils/ChSpeedController.cpp
)

if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_DRIVER_FILES
        driver/ChIrrGuiDriver.h
        driver/ChIrrGuiDriver.cpp
        driver/ChIrrGuiST.h
        driver/ChIrrGuiST.cpp
    )
    set(CVIRR_UTILS_FILES
        utils/ChVehicleIrrApp.h
        utils/ChVehicleIrrApp.cpp
    )
else()
    set(CVIRR_DRIVER_FILES "")
    set(CVIRR_UTILS_FILES "")
endif()

set(CV_EASYBMP_FILES
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP.cpp
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_BMP.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_DataStructures.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_VariousBMPutilities.h
    )

source_group("base" FILES ${CV_BASE_FILES})
source_group("vehicle" FILES ${CV_VEHICLE_FILES})
source_group("suspension" FILES ${CV_SUSPENSION_FILES})
source_group("wheel" FILES ${CV_WHEEL_FILES})
source_group("steering" FILES ${CV_STEERING_FILES})
source_group("driveline" FILES ${CV_DRIVELINE_FILES})
source_group("driver" FILES ${CV_DRIVER_FILES} ${CVIRR_DRIVER_FILES})
source_group("powertrain" FILES ${CV_POVERTRAIN_FILES})
source_group("tire" FILES ${CV_TIRE_FILES})
source_group("brake" FILES ${CV_BRAKE_FILES})
source_group("antirollbar" FILES ${CV_ANTIROLL_FILES})
source_group("terrain" FILES ${CV_TERRAIN_FILES})
source_group("suspensionTest" FILES ${CV_SUSPENSIONTEST_FILES})
source_group("utils" FILES ${CV_UTILS_FILES} ${CVIRR_UTILS_FILES})
source_group("easyBMP" FILES ${CV_EASYBMP_FILES})

# ----------------------------------------------------------------------------
# Add the ChronoEngine_vehicle library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")
if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    include_directories( ${CH_IRRLICHTINC} )
endif()

add_library(ChronoEngine_vehicle SHARED
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
    ${CV_UTILS_FILES}
    ${CVIRR_DRIVER_FILES}
    ${CVIRR_UTILS_FILES}
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
# On Windows, copy the Irrklang DLL (if enabled)
#-------------------------------------------------------------------------------
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND ENBALE_IRRKLANG)
    if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
    elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
    endif()

    add_custom_command(TARGET ChronoEngine_vehicle POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CH_IRRKLANG_DLL}"
        "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
endif()
