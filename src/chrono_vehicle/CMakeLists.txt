#=============================================================================
# CMake configuration file for Chrono Vehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_VEHICLE)
   mark_as_advanced(FORCE ENABLE_IRRKLANG)
   mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
   mark_as_advanced(FORCE CH_IRRKLANGLIB)
   return()
endif()

message(STATUS "==== Chrono Vehicle module ====")

mark_as_advanced(CLEAR ENABLE_IRRKLANG)
mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
mark_as_advanced(CLEAR CH_IRRKLANGLIB)

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

# --------------- COMMON FILES

set(CV_BASE_FILES
    ChApiVehicle.h
    ChDriver.h
    ChDriver.cpp
    ChPowertrain.h
    ChPowertrain.cpp
    ChSubsysDefs.h
    ChTerrain.h
    ChTerrain.cpp
    ChVehicle.h
    ChVehicle.cpp
    ChVehicleModelData.h
    ChVehicleModelData.cpp
)
source_group("base" FILES ${CV_BASE_FILES})

set(CV_DRIVER_FILES
    driver/ChDataDriver.h
    driver/ChDataDriver.cpp
    driver/ChPathFollowerDriver.h
    driver/ChPathFollowerDriver.cpp
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

    powertrain/SimplePowertrain.h
    powertrain/SimplePowertrain.cpp
    powertrain/ShaftsPowertrain.h
    powertrain/ShaftsPowertrain.cpp
)
source_group("powertrain" FILES ${CV_POVERTRAIN_FILES})

set(CV_TERRAIN_FILES
    terrain/FlatTerrain.h
    terrain/FlatTerrain.cpp
    terrain/RigidTerrain.h
    terrain/RigidTerrain.cpp
	terrain/DeformableTerrain.h
    terrain/DeformableTerrain.cpp
)
source_group("terrain" FILES ${CV_TERRAIN_FILES})

set(CV_UTILS_FILES
    utils/ChSteeringController.h
    utils/ChSteeringController.cpp
    utils/ChSpeedController.h
    utils/ChSpeedController.cpp
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
    wheeled_vehicle/steering/ChRackPinion.h
    wheeled_vehicle/steering/ChRackPinion.cpp

    wheeled_vehicle/steering/PitmanArm.h
    wheeled_vehicle/steering/PitmanArm.cpp
    wheeled_vehicle/steering/RackPinion.h
    wheeled_vehicle/steering/RackPinion.cpp
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
)
source_group("wheeled_vehicle\\suspension" FILES ${CV_WV_SUSPENSION_FILES})

set(CV_WV_TIRE_FILES
    wheeled_vehicle/tire/ChRigidTire.h
    wheeled_vehicle/tire/ChRigidTire.cpp
    wheeled_vehicle/tire/ChPacejkaTire.h
    wheeled_vehicle/tire/ChPacejkaTire.cpp
    wheeled_vehicle/tire/ChLugreTire.h
    wheeled_vehicle/tire/ChLugreTire.cpp
    wheeled_vehicle/tire/ChFialaTire.h
    wheeled_vehicle/tire/ChFialaTire.cpp

    wheeled_vehicle/tire/RigidTire.h
    wheeled_vehicle/tire/RigidTire.cpp
    wheeled_vehicle/tire/LugreTire.h
    wheeled_vehicle/tire/LugreTire.cpp
    wheeled_vehicle/tire/FialaTire.h
    wheeled_vehicle/tire/FialaTire.cpp
)
source_group("wheeled_vehicle\\tire" FILES ${CV_WV_TIRE_FILES})

set(CV_WV_UTILS_FILES
    wheeled_vehicle/utils/ChWheeledVehicleAssembly.h
    wheeled_vehicle/utils/ChWheeledVehicleAssembly.cpp
    wheeled_vehicle/utils/ChSuspensionTestRig.h
    wheeled_vehicle/utils/ChSuspensionTestRig.cpp
)
if(ENABLE_MODULE_IRRLICHT)
    set(CVIRR_WV_UTILS_FILES
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h
        wheeled_vehicle/utils/ChWheeledVehicleIrrApp.cpp
        wheeled_vehicle/utils/ChIrrGuiDriverSTR.h
        wheeled_vehicle/utils/ChIrrGuiDriverSTR.cpp
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

# --------------- TRACKED VEHICLE FILES

set(CV_TV_BASE_FILES

)

# --------------- THIRD-PARTY FILES

set(CV_EASYBMP_FILES
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP.cpp
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_BMP.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_DataStructures.h
    ${CMAKE_SOURCE_DIR}/src/thirdparty/Easy_BMP/EasyBMP_VariousBMPutilities.h
)
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
#
    ${CV_BASE_FILES}
    ${CV_DRIVER_FILES}
    ${CVIRR_DRIVER_FILES}
    ${CV_POVERTRAIN_FILES}
    ${CV_TERRAIN_FILES}
    ${CV_UTILS_FILES}
    ${CVIRR_UTILS_FILES}
#
    ${CV_WV_BASE_FILES}
    ${CV_WV_ANTIROLL_FILES}
    ${CV_WV_BRAKE_FILES}
    ${CV_WV_DRIVELINE_FILES}
    ${CV_WV_STEERING_FILES}
    ${CV_WV_SUSPENSION_FILES}
    ${CV_WV_TIRE_FILES}
    ${CV_WV_UTILS_FILES}
    ${CVIRR_WV_UTILS_FILES}
    ${CV_WV_VEHICLE_FILES}
    ${CV_WV_WHEEL_FILES}
#
    ${CV_TV_BASE_FILES}
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
