#=============================================================================
# CMake configuration file for Chrono Vehicle Models
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "Models for the VEHICLE module...")

# ----------------------------------------------------------------------------
# List the files in the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CVM_COMMON_FILES
    ../ChApiModels.h
    ChVehicleModelDefs.h
)
source_group("" FILES ${CVM_COMMON_FILES})

set(CVM_HMMWV_FILES
    hmmwv/HMMWV.cpp
    hmmwv/HMMWV.h
    hmmwv/HMMWV_BrakeSimple.cpp
    hmmwv/HMMWV_BrakeSimple.h
    hmmwv/HMMWV_Chassis.h
    hmmwv/HMMWV_Chassis.cpp
    hmmwv/HMMWV_DoubleWishbone.cpp
    hmmwv/HMMWV_DoubleWishbone.h
    hmmwv/HMMWV_DoubleWishboneReduced.cpp
    hmmwv/HMMWV_DoubleWishboneReduced.h
    hmmwv/HMMWV_Driveline2WD.cpp
    hmmwv/HMMWV_Driveline2WD.h
    hmmwv/HMMWV_Driveline4WD.cpp
    hmmwv/HMMWV_Driveline4WD.h
    hmmwv/HMMWV_RigidTire.cpp
    hmmwv/HMMWV_RigidTire.h
    hmmwv/HMMWV_FialaTire.cpp
    hmmwv/HMMWV_FialaTire.h
    hmmwv/HMMWV_LugreTire.cpp
    hmmwv/HMMWV_LugreTire.h
    hmmwv/HMMWV_Pac02Tire.cpp
    hmmwv/HMMWV_Pac02Tire.h
    hmmwv/HMMWV_Pac89Tire.cpp
    hmmwv/HMMWV_Pac89Tire.h
    hmmwv/HMMWV_TMeasyTire.cpp
    hmmwv/HMMWV_TMeasyTire.h
    hmmwv/HMMWV_ANCFTire.cpp
    hmmwv/HMMWV_ANCFTire.h
    hmmwv/HMMWV_ReissnerTire.cpp
    hmmwv/HMMWV_ReissnerTire.h
    hmmwv/HMMWV_PitmanArm.cpp
    hmmwv/HMMWV_PitmanArm.h
    hmmwv/HMMWV_PitmanArmShafts.cpp
    hmmwv/HMMWV_PitmanArmShafts.h
    hmmwv/HMMWV_Powertrain.cpp
    hmmwv/HMMWV_Powertrain.h
    hmmwv/HMMWV_RackPinion.cpp
    hmmwv/HMMWV_RackPinion.h
    hmmwv/HMMWV_SimpleDriveline.cpp
    hmmwv/HMMWV_SimpleDriveline.h
    hmmwv/HMMWV_SimplePowertrain.cpp
    hmmwv/HMMWV_SimplePowertrain.h
    hmmwv/HMMWV_SimpleCVTPowertrain.cpp
    hmmwv/HMMWV_SimpleCVTPowertrain.h
    hmmwv/HMMWV_SimpleMapPowertrain.cpp
    hmmwv/HMMWV_SimpleMapPowertrain.h
    hmmwv/HMMWV_Vehicle.h
    hmmwv/HMMWV_VehicleFull.cpp
    hmmwv/HMMWV_VehicleFull.h
    hmmwv/HMMWV_VehicleReduced.cpp
    hmmwv/HMMWV_VehicleReduced.h
    hmmwv/HMMWV_Wheel.cpp
    hmmwv/HMMWV_Wheel.h
)
source_group("hmmwv" FILES ${CVM_HMMWV_FILES})

set(CVM_UAZ_FILES
    uaz/UAZBUS.cpp
    uaz/UAZBUS.h
    uaz/UAZBUS_BrakeSimple.cpp
    uaz/UAZBUS_BrakeSimple.h
    uaz/UAZBUS_Chassis.cpp
    uaz/UAZBUS_Chassis.h
    uaz/UAZBUS_Driveline2WD.cpp
    uaz/UAZBUS_Driveline2WD.h
    uaz/UAZBUS_Driveline4WD.cpp
    uaz/UAZBUS_Driveline4WD.h
    uaz/UAZBUS_LeafspringAxle.cpp
    uaz/UAZBUS_LeafspringAxle.h
    uaz/UAZBUS_RigidTire.cpp
    uaz/UAZBUS_RigidTire.h
    uaz/UAZBUS_RotaryArm.cpp
    uaz/UAZBUS_RotaryArm.h
    uaz/UAZBUS_SimpleMapPowertrain.cpp
    uaz/UAZBUS_SimpleMapPowertrain.h
    uaz/UAZBUS_TMeasyTire.cpp
    uaz/UAZBUS_TMeasyTire.h
    uaz/UAZBUS_ToeBarLeafspringAxle.cpp
    uaz/UAZBUS_ToeBarLeafspringAxle.h
    uaz/UAZBUS_Vehicle.cpp
    uaz/UAZBUS_Vehicle.h
    uaz/UAZBUS_Wheel.cpp
    uaz/UAZBUS_Wheel.h
)
source_group("uaz" FILES ${CVM_UAZ_FILES})

set(CVM_GENERIC_FILES
    generic/Generic_AntirollBarRSD.h
    generic/Generic_AntirollBarRSD.cpp
    generic/Generic_BrakeSimple.h
    generic/Generic_BrakeSimple.cpp
    generic/Generic_Chassis.cpp
    generic/Generic_Chassis.h
    generic/Generic_DoubleWishbone.cpp
    generic/Generic_DoubleWishbone.h
    generic/Generic_Driveline2WD.h
    generic/Generic_Driveline2WD.cpp
    generic/Generic_FuncDriver.h
    generic/Generic_HendricksonPRIMAXX.cpp
    generic/Generic_HendricksonPRIMAXX.h
    generic/Generic_MacPhersonStrut.cpp
    generic/Generic_MacPhersonStrut.h
    generic/Generic_MultiLink.cpp
    generic/Generic_MultiLink.h
    generic/Generic_RackPinion.h
    generic/Generic_RackPinion.cpp
    generic/Generic_RigidTire.h
    generic/Generic_RigidTire.cpp
    generic/Generic_RigidMeshTire.h
    generic/Generic_RigidMeshTire.cpp
    generic/Generic_FialaTire.h
    generic/Generic_FialaTire.cpp
    generic/Generic_SimplePowertrain.h
    generic/Generic_SimplePowertrain.cpp
    generic/Generic_SimpleMapPowertrain.cpp
    generic/Generic_SimpleMapPowertrain.h
    generic/Generic_SolidAxle.cpp
    generic/Generic_SolidAxle.h
    generic/Generic_Vehicle.cpp
    generic/Generic_Vehicle.h
    generic/Generic_Wheel.h
    generic/Generic_Wheel.cpp
    generic/Generic_SimpleDriveline.cpp
    generic/Generic_SimpleDriveline.h
    generic/Generic_RigidSuspension.h
    generic/Generic_RigidSuspension.cpp
    generic/Generic_RigidPinnedAxle.h
    generic/Generic_RigidPinnedAxle.cpp
)
source_group("generic" FILES ${CVM_GENERIC_FILES})

set(CVM_SEDAN_FILES
    sedan/Sedan.cpp
    sedan/Sedan.h
    sedan/Sedan_BrakeSimple.cpp
    sedan/Sedan_BrakeSimple.h
    sedan/Sedan_Chassis.h
    sedan/Sedan_Chassis.cpp
    sedan/Sedan_DoubleWishbone.cpp
    sedan/Sedan_DoubleWishbone.h
    sedan/Sedan_Driveline2WD.cpp
    sedan/Sedan_Driveline2WD.h
    sedan/Sedan_MultiLink.cpp
    sedan/Sedan_MultiLink.h
    sedan/Sedan_RigidTire.cpp
    sedan/Sedan_RigidTire.h
    sedan/Sedan_TMeasyTire.cpp
    sedan/Sedan_TMeasyTire.h
    sedan/Sedan_RackPinion.cpp
    sedan/Sedan_RackPinion.h
    sedan/Sedan_SimpleMapPowertrain.cpp
    sedan/Sedan_SimpleMapPowertrain.h
    sedan/Sedan_Vehicle.h
    sedan/Sedan_Vehicle.cpp
    sedan/Sedan_Wheel.cpp
    sedan/Sedan_Wheel.h
)
source_group("sedan" FILES ${CVM_SEDAN_FILES})

set(CVM_CITYBUS_FILES
    citybus/CityBus.cpp
    citybus/CityBus.h
    citybus/CityBus_BrakeSimple.cpp
    citybus/CityBus_BrakeSimple.h
    citybus/CityBus_Chassis.h
    citybus/CityBus_Chassis.cpp
    citybus/CityBus_SolidAxle.h
    citybus/CityBus_SolidAxle.cpp
    citybus/CityBus_ToeBarLeafspringAxle.h
    citybus/CityBus_ToeBarLeafspringAxle.cpp
    citybus/CityBus_RotaryArm.h
    citybus/CityBus_RotaryArm.cpp
    citybus/CityBus_LeafspringAxle.h
    citybus/CityBus_LeafspringAxle.cpp
    citybus/CityBus_Driveline2WD.cpp
    citybus/CityBus_Driveline2WD.h
    citybus/CityBus_RigidTire.cpp
    citybus/CityBus_RigidTire.h
    citybus/CityBus_TMeasyTire.cpp
    citybus/CityBus_TMeasyTire.h
    citybus/CityBus_RackPinion.cpp
    citybus/CityBus_RackPinion.h
    citybus/CityBus_SimpleMapPowertrain.cpp
    citybus/CityBus_SimpleMapPowertrain.h
    citybus/CityBus_Vehicle.h
    citybus/CityBus_Vehicle.cpp
    citybus/CityBus_Wheel.cpp
    citybus/CityBus_Wheel.h
)
source_group("citybus" FILES ${CVM_CITYBUS_FILES})


set(CVM_M113_FILES
    m113/M113_BrakeSimple.h
    m113/M113_Chassis.h
    m113/M113_Chassis.cpp
    m113/M113_DrivelineBDS.cpp
    m113/M113_DrivelineBDS.h
    m113/M113_Idler.cpp
    m113/M113_Idler.h
    m113/M113_RoadWheel.cpp
    m113/M113_RoadWheel.h
    m113/M113_ShaftsPowertrain.cpp
    m113/M113_ShaftsPowertrain.h
    m113/M113_SimpleDriveline.cpp
    m113/M113_SimpleDriveline.h
    m113/M113_SimplePowertrain.cpp
    m113/M113_SimplePowertrain.h
    m113/M113_SprocketSinglePin.cpp
    m113/M113_SprocketSinglePin.h
    m113/M113_SprocketDoublePin.cpp
    m113/M113_SprocketDoublePin.h
    m113/M113_SprocketBand.cpp
    m113/M113_SprocketBand.h
    m113/M113_Suspension.cpp
    m113/M113_Suspension.h
    m113/M113_TrackAssemblySinglePin.cpp
    m113/M113_TrackAssemblySinglePin.h
    m113/M113_TrackAssemblyDoublePin.cpp
    m113/M113_TrackAssemblyDoublePin.h
    m113/M113_TrackAssemblyBandBushing.cpp
    m113/M113_TrackAssemblyBandBushing.h
    m113/M113_TrackAssemblyBandANCF.cpp
    m113/M113_TrackAssemblyBandANCF.h
    m113/M113_TrackShoeSinglePin.cpp
    m113/M113_TrackShoeSinglePin.h
    m113/M113_TrackShoeDoublePin.cpp
    m113/M113_TrackShoeDoublePin.h
    m113/M113_TrackShoeBandBushing.cpp
    m113/M113_TrackShoeBandBushing.h
    m113/M113_TrackShoeBandANCF.cpp
    m113/M113_TrackShoeBandANCF.h
    m113/M113_Vehicle.cpp
    m113/M113_Vehicle.h
)
source_group("m113" FILES ${CVM_M113_FILES})

set(CVM_M113_A_FILES
    m113a/M113a_BrakeSimple.h
    m113a/M113a_Chassis.cpp
    m113a/M113a_Chassis.h
    m113a/M113a_DrivelineBDS.cpp
    m113a/M113a_DrivelineBDS.h
    m113a/M113a_Idler.cpp
    m113a/M113a_Idler.h
    m113a/M113a_RoadWheel.cpp
    m113a/M113a_RoadWheel.h
    m113a/M113a_ShaftsPowertrain.cpp
    m113a/M113a_ShaftsPowertrain.h
    m113a/M113a_SimpleDriveline.cpp
    m113a/M113a_SimpleDriveline.h
    m113a/M113a_SimplePowertrain.cpp
    m113a/M113a_SimplePowertrain.h
    m113a/M113a_SprocketSinglePin.cpp
    m113a/M113a_SprocketSinglePin.h
    m113a/M113a_Suspension.cpp
    m113a/M113a_Suspension.h
    m113a/M113a_TrackAssemblySinglePin.cpp
    m113a/M113a_TrackAssemblySinglePin.h
    m113a/M113a_TrackShoeSinglePin.cpp
    m113a/M113a_TrackShoeSinglePin.h
    m113a/M113a_Vehicle.cpp
    m113a/M113a_Vehicle.h
 )
source_group("m113a" FILES ${CVM_M113_A_FILES})

# ----------------------------------------------------------------------------
# Add the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)
if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    include_directories(${CH_IRRLICHTINC})
endif()

add_library(ChronoModels_vehicle SHARED
    ${CVM_COMMON_FILES}
    ${CVM_HMMWV_FILES}
    ${CVM_UAZ_FILES}
    ${CVM_GENERIC_FILES}
    ${CVM_SEDAN_FILES}
    ${CVM_CITYBUS_FILES}
    ${CVM_M113_FILES}
    ${CVM_M113_A_FILES}
 )

set_target_properties(ChronoModels_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MODELS")

target_link_libraries(ChronoModels_vehicle ${LIBRARIES})

install(TARGETS ChronoModels_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_models/vehicle
        FILES_MATCHING PATTERN "*.h")
