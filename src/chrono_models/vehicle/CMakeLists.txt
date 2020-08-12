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
    hmmwv/HMMWV_BrakeShafts.cpp
    hmmwv/HMMWV_BrakeShafts.h
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
    hmmwv/HMMWV_PacejkaTire.cpp
    hmmwv/HMMWV_PacejkaTire.h
    hmmwv/HMMWV_Pac89Tire.cpp
    hmmwv/HMMWV_Pac89Tire.h
    hmmwv/HMMWV_Pac02Tire.cpp
    hmmwv/HMMWV_Pac02Tire.h
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

set(CVM_FMTV_FILES
    mtv/FMTV_ChassisFront.cpp
    mtv/FMTV_ChassisFront.h
    mtv/FMTV_ToebarLeafspringAxle.cpp
    mtv/FMTV_ToebarLeafspringAxle.h
    mtv/FMTV_Wheel.cpp
    mtv/FMTV_Wheel.h
    mtv/FMTV_BrakeSimple.cpp
    mtv/FMTV_BrakeSimple.h
    mtv/FMTV_BrakeShafts.cpp
    mtv/FMTV_BrakeShafts.h
    mtv/FMTV_RotaryArm.cpp
    mtv/FMTV_RotaryArm.h
    mtv/FMTV_AntiRollBar.cpp
    mtv/FMTV_AntiRollBar.h
    mtv/FMTV_SimpleDriveline.cpp
    mtv/FMTV_SimpleDriveline.h
    mtv/FMTV_Driveline4WD.cpp
    mtv/FMTV_Driveline4WD.h
    mtv/FMTV_SimpleCVTPowertrain.cpp
    mtv/FMTV_SimpleCVTPowertrain.h
    mtv/FMTV_SimpleMapPowertrain.cpp
    mtv/FMTV_SimpleMapPowertrain.h
    mtv/FMTV_SimplePowertrain.cpp
    mtv/FMTV_SimplePowertrain.h
    mtv/FMTV_Powertrain.cpp
    mtv/FMTV_Powertrain.h
    mtv/FMTV_TMeasyTire.cpp
    mtv/FMTV_TMeasyTire.h
    mtv/FMTV_RigidTire.cpp
    mtv/FMTV_RigidTire.h

    mtv/LMTV.cpp
    mtv/LMTV.h
    mtv/LMTV_Vehicle.cpp
    mtv/LMTV_Vehicle.h
    mtv/LMTV_ChassisRear.cpp
    mtv/LMTV_ChassisRear.h
    mtv/LMTV_LeafspringAxle.cpp
    mtv/LMTV_LeafspringAxle.h

    mtv/MTV.cpp
    mtv/MTV.h
    mtv/MTV_Vehicle.cpp
    mtv/MTV_Vehicle.h
    mtv/MTV_ChassisRear.cpp
    mtv/MTV_ChassisRear.h
    mtv/MTV_Balancer.cpp
    mtv/MTV_Balancer.h
    mtv/MTV_LeafspringAxle1.cpp
    mtv/MTV_LeafspringAxle1.h
    mtv/MTV_LeafspringAxle2.cpp
    mtv/MTV_LeafspringAxle2.h
)
source_group("mtv" FILES ${CVM_FMTV_FILES})

set(CVM_UAZ_FILES
    uaz/UAZBUS.cpp
    uaz/UAZBUS.h
    uaz/UAZBUS_SAE.cpp
    uaz/UAZBUS_SAE.h
    uaz/UAZBUS_BrakeSimple.cpp
    uaz/UAZBUS_BrakeSimple.h
    uaz/UAZBUS_BrakeShafts.cpp
    uaz/UAZBUS_BrakeShafts.h
    uaz/UAZBUS_Chassis.cpp
    uaz/UAZBUS_Chassis.h
    uaz/UAZBUS_Driveline2WD.cpp
    uaz/UAZBUS_Driveline2WD.h
    uaz/UAZBUS_Driveline4WD.cpp
    uaz/UAZBUS_Driveline4WD.h
    uaz/UAZBUS_LeafspringAxle.cpp
    uaz/UAZBUS_LeafspringAxle.h
    uaz/UAZBUS_SAELeafspringAxle.cpp
    uaz/UAZBUS_SAELeafspringAxle.h
    uaz/UAZBUS_RigidTire.cpp
    uaz/UAZBUS_RigidTire.h
    uaz/UAZBUS_RotaryArm.cpp
    uaz/UAZBUS_RotaryArm.h
    uaz/UAZBUS_SimpleMapPowertrain.cpp
    uaz/UAZBUS_SimpleMapPowertrain.h
    uaz/UAZBUS_TMeasyTire.cpp
    uaz/UAZBUS_TMeasyTire.h
    uaz/UAZBUS_Pac02Tire.cpp
    uaz/UAZBUS_Pac02Tire.h
    uaz/UAZBUS_SAEToeBarLeafspringAxle.cpp
    uaz/UAZBUS_SAEToeBarLeafspringAxle.h
    uaz/UAZBUS_ToeBarLeafspringAxle.cpp
    uaz/UAZBUS_ToeBarLeafspringAxle.h
    uaz/UAZBUS_Vehicle.cpp
    uaz/UAZBUS_Vehicle.h
    uaz/UAZBUS_SAEVehicle.cpp
    uaz/UAZBUS_SAEVehicle.h
    uaz/UAZBUS_Wheel.cpp
    uaz/UAZBUS_Wheel.h
)
source_group("uaz" FILES ${CVM_UAZ_FILES})

set(CVM_MAN_FILES
    man/MAN_5t_BellcrankSolid3LinkAxle.cpp
    man/MAN_5t_BellcrankSolid3LinkAxle.h
    man/MAN_5t_BrakeSimple.cpp
    man/MAN_5t_BrakeSimple.h
    man/MAN_5t_BrakeShafts.cpp
    man/MAN_5t_BrakeShafts.h
    man/MAN_5t_Chassis.cpp
    man/MAN_5t_Chassis.h
    man/MAN_5t_Driveline4WD.cpp
    man/MAN_5t_Driveline4WD.h
    man/MAN_5t_RotaryArm.cpp
    man/MAN_5t_RotaryArm.h
    man/MAN_5t_SimpleDriveline.cpp
    man/MAN_5t_SimpleDriveline.h
    man/MAN_5t_SimpleDrivelineXWD.cpp
    man/MAN_5t_SimpleDrivelineXWD.h
    man/MAN_5t_SimpleCVTPowertrain.cpp
    man/MAN_5t_SimpleCVTPowertrain.h
    man/MAN_5t_SimpleMapPowertrain.cpp
    man/MAN_5t_SimpleMapPowertrain.h
    man/MAN_5t_Solid3LinkAxle.cpp
    man/MAN_5t_Solid3LinkAxle.h
    man/MAN_5t_TMeasyTire.cpp
    man/MAN_5t_TMeasyTire.h
    man/MAN_5t_Wheel.cpp
    man/MAN_5t_Wheel.h
    man/MAN_5t_Vehicle.cpp
    man/MAN_5t_Vehicle.h
    man/MAN_5t.cpp
    man/MAN_5t.h

    man/MAN_7t_Chassis.cpp
    man/MAN_7t_Chassis.h
    man/MAN_7t_SimpleCVTPowertrain.cpp
    man/MAN_7t_SimpleCVTPowertrain.h
    man/MAN_7t_SimpleMapPowertrain.cpp
    man/MAN_7t_SimpleMapPowertrain.h
    man/MAN_7t_Solid3LinkAxle.cpp
    man/MAN_7t_Solid3LinkAxle.h
    man/MAN_7t_Vehicle.cpp
    man/MAN_7t_Vehicle.h
    man/MAN_7t.cpp
    man/MAN_7t.h

    man/MAN_10t_Chassis.cpp
    man/MAN_10t_Chassis.h
    man/MAN_10t_Front1Axle.cpp
    man/MAN_10t_Front1Axle.h
    man/MAN_10t_Front2Axle.cpp
    man/MAN_10t_Front2Axle.h
    man/MAN_10t_RotaryArm2.cpp
    man/MAN_10t_RotaryArm2.h
    man/MAN_10t_Vehicle.cpp
    man/MAN_10t_Vehicle.h
    man/MAN_10t.cpp
    man/MAN_10t.h
)
source_group("man" FILES ${CVM_MAN_FILES})

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
    sedan/Sedan_BrakeShafts.cpp
    sedan/Sedan_BrakeShafts.h
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
    sedan/Sedan_Pac02Tire.cpp
    sedan/Sedan_Pac02Tire.h
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
    citybus/CityBus_BrakeShafts.cpp
    citybus/CityBus_BrakeShafts.h
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
    citybus/CityBus_Pac02Tire.cpp
    citybus/CityBus_Pac02Tire.h
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

set(CVM_GATOR_FILES
    gator/Gator.cpp
    gator/Gator.h
    gator/Gator_BrakeSimple.cpp
    gator/Gator_BrakeSimple.h
    gator/Gator_BrakeShafts.cpp
    gator/Gator_BrakeShafts.h
    gator/Gator_Chassis.h
    gator/Gator_Chassis.cpp
    gator/Gator_SingleWishbone.cpp
    gator/Gator_SingleWishbone.h
    gator/Gator_SimpleDriveline.cpp
    gator/Gator_SimpleDriveline.h
    gator/Gator_Driveline2WD.cpp
    gator/Gator_Driveline2WD.h
    gator/Gator_RigidSuspension.cpp
    gator/Gator_RigidSuspension.h
    gator/Gator_RigidTire.cpp
    gator/Gator_RigidTire.h
    gator/Gator_TMeasyTire.cpp
    gator/Gator_TMeasyTire.h
    gator/Gator_RackPinion.cpp
    gator/Gator_RackPinion.h
    gator/Gator_SimplePowertrain.cpp
    gator/Gator_SimplePowertrain.h
    gator/Gator_SimpleMapPowertrain.cpp
    gator/Gator_SimpleMapPowertrain.h
    gator/Gator_Vehicle.h
    gator/Gator_Vehicle.cpp
    gator/Gator_Wheel.cpp
    gator/Gator_Wheel.h
)
source_group("gator" FILES ${CVM_GATOR_FILES})

set(CVM_M113_FILES
    m113/M113_BrakeSimple.h
    m113/M113_BrakeShafts.h
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

# ----------------------------------------------------------------------------
# Add the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)

add_library(ChronoModels_vehicle SHARED
    ${CVM_COMMON_FILES}
    ${CVM_HMMWV_FILES}
    ${CVM_FMTV_FILES}
    ${CVM_UAZ_FILES}
    ${CVM_GENERIC_FILES}
    ${CVM_SEDAN_FILES}
    ${CVM_CITYBUS_FILES}
    ${CVM_GATOR_FILES}
    ${CVM_M113_FILES}
    ${CVM_MAN_FILES}
 )

set_target_properties(ChronoModels_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoModels_vehicle PRIVATE "CH_API_COMPILE_MODELS")
target_compile_definitions(ChronoModels_vehicle PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoModels_vehicle ${LIBRARIES})

install(TARGETS ChronoModels_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_models/vehicle
        FILES_MATCHING PATTERN "*.h")
