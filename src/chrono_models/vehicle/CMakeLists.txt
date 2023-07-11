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

set(CVM_HMMWV_SUSPENSION_FILES
    hmmwv/suspension/HMMWV_DoubleWishbone.cpp
    hmmwv/suspension/HMMWV_DoubleWishbone.h
    hmmwv/suspension/HMMWV_DoubleWishboneReduced.cpp
    hmmwv/suspension/HMMWV_DoubleWishboneReduced.h
)
set(CVM_HMMWV_STEERING_FILES
    hmmwv/steering/HMMWV_PitmanArm.cpp
    hmmwv/steering/HMMWV_PitmanArm.h
    hmmwv/steering/HMMWV_PitmanArmShafts.cpp
    hmmwv/steering/HMMWV_PitmanArmShafts.h
    hmmwv/steering/HMMWV_RackPinion.cpp
    hmmwv/steering/HMMWV_RackPinion.h    
)
set(CVM_HMMWV_DRIVELINE_FILES
    hmmwv/driveline/HMMWV_Driveline2WD.cpp
    hmmwv/driveline/HMMWV_Driveline2WD.h
    hmmwv/driveline/HMMWV_Driveline4WD.cpp
    hmmwv/driveline/HMMWV_Driveline4WD.h
    hmmwv/driveline/HMMWV_SimpleDriveline.cpp
    hmmwv/driveline/HMMWV_SimpleDriveline.h
)
set(CVM_HMMWV_BRAKE_FILES
    hmmwv/brake/HMMWV_BrakeSimple.cpp
    hmmwv/brake/HMMWV_BrakeSimple.h
    hmmwv/brake/HMMWV_BrakeShafts.cpp
    hmmwv/brake/HMMWV_BrakeShafts.h
)
set(CVM_HMMWV_POWERTRAIN_FILES
    hmmwv/powertrain/HMMWV_EngineShafts.h
    hmmwv/powertrain/HMMWV_EngineShafts.cpp
    hmmwv/powertrain/HMMWV_EngineSimpleMap.h
    hmmwv/powertrain/HMMWV_EngineSimpleMap.cpp
    hmmwv/powertrain/HMMWV_EngineSimple.h
    hmmwv/powertrain/HMMWV_EngineSimple.cpp
    hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.h
    hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.cpp
    hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.h
    hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.cpp
)
set(CVM_HMMWV_TIRE_FILES
    hmmwv/tire/HMMWV_RigidTire.cpp
    hmmwv/tire/HMMWV_RigidTire.h
    hmmwv/tire/HMMWV_FialaTire.cpp
    hmmwv/tire/HMMWV_FialaTire.h
    hmmwv/tire/HMMWV_Pac02Tire.cpp
    hmmwv/tire/HMMWV_Pac02Tire.h
    hmmwv/tire/HMMWV_Pac89Tire.cpp
    hmmwv/tire/HMMWV_Pac89Tire.h
    hmmwv/tire/HMMWV_TMeasyTire.cpp
    hmmwv/tire/HMMWV_TMeasyTire.h
    hmmwv/tire/HMMWV_TMsimpleTire.cpp
    hmmwv/tire/HMMWV_TMsimpleTire.h
    hmmwv/tire/HMMWV_ANCFTire.cpp
    hmmwv/tire/HMMWV_ANCFTire.h
    hmmwv/tire/HMMWV_ReissnerTire.cpp
    hmmwv/tire/HMMWV_ReissnerTire.h
)
set(CVM_HMMWV_VEHICLE_FILES
    hmmwv/HMMWV.cpp
    hmmwv/HMMWV.h
    hmmwv/HMMWV_Chassis.h
    hmmwv/HMMWV_Chassis.cpp
    hmmwv/HMMWV_Vehicle.h
    hmmwv/HMMWV_VehicleFull.cpp
    hmmwv/HMMWV_VehicleFull.h
    hmmwv/HMMWV_VehicleReduced.cpp
    hmmwv/HMMWV_VehicleReduced.h
    hmmwv/HMMWV_Wheel.cpp
    hmmwv/HMMWV_Wheel.h
)
set(CVM_HMMWV_FILES
    ${CVM_HMMWV_VEHICLE_FILES}
    ${CVM_HMMWV_SUSPENSION_FILES}
    ${CVM_HMMWV_STEERING_FILES}
    ${CVM_HMMWV_DRIVELINE_FILES}
    ${CVM_HMMWV_BRAKE_FILES}
    ${CVM_HMMWV_POWERTRAIN_FILES}
    ${CVM_HMMWV_TIRE_FILES}
)
source_group("hmmwv" FILES ${CVM_HMMWV_VEHICLE_FILES})
source_group("hmmwv/suspension" FILES ${CVM_HMMWV_SUSPENSION_FILES})
source_group("hmmwv/steering" FILES ${CVM_HMMWV_STEERING_FILES})
source_group("hmmwv/driveline" FILES ${CVM_HMMWV_DRIVELINE_FILES})
source_group("hmmwv/brake" FILES ${CVM_HMMWV_BRAKE_FILES})
source_group("hmmwv/powertrain" FILES ${CVM_HMMWV_POWERTRAIN_FILES})
source_group("hmmwv/tire" FILES ${CVM_HMMWV_TIRE_FILES})

set(CVM_FEDA_FILES
    feda/FEDA_AutomaticTransmissionSimpleMap.h
    feda/FEDA_AutomaticTransmissionSimpleMap.cpp
    feda/FEDA_AntirollBarRSD.h
    feda/FEDA_AntirollBarRSD.cpp
    feda/FEDA_BrakeSimple.cpp
    feda/FEDA_BrakeSimple.h
    feda/FEDA_BrakeShafts.cpp
    feda/FEDA_BrakeShafts.h
    feda/FEDA_Chassis.cpp
    feda/FEDA_Chassis.h
    feda/FEDA_Driveline4WD.cpp
    feda/FEDA_Driveline4WD.h
    feda/FEDA_DoubleWishbone.cpp
    feda/FEDA_DoubleWishbone.h
    feda/FEDA_EngineSimpleMap.cpp
    feda/FEDA_EngineSimpleMap.h
    feda/FEDA_Pac02Tire.cpp
    feda/FEDA_Pac02Tire.h
    feda/FEDA_PitmanArm.cpp
    feda/FEDA_PitmanArm.h
    feda/FEDA_RigidTire.cpp
    feda/FEDA_RigidTire.h
    feda/FEDA_TMeasyTire.cpp
    feda/FEDA_TMeasyTire.h
    feda/FEDA_TMsimpleTire.cpp
    feda/FEDA_TMsimpleTire.h
    feda/FEDA_Vehicle.cpp
    feda/FEDA_Vehicle.h   
    feda/FEDA_Wheel.cpp
    feda/FEDA_Wheel.h   
    feda/FEDA.cpp
    feda/FEDA.h   
)
source_group("feda" FILES ${CVM_FEDA_FILES})

set(CVM_MROLE_POWERTRAIN_FILES
    mrole/powertrain/mrole_AutomaticTransmissionShafts.cpp
    mrole/powertrain/mrole_AutomaticTransmissionShafts.h
    mrole/powertrain/mrole_AutomaticTransmissionSimpleMap.cpp
    mrole/powertrain/mrole_AutomaticTransmissionSimpleMap.h
    mrole/powertrain/mrole_EngineShafts.cpp
    mrole/powertrain/mrole_EngineShafts.h
    mrole/powertrain/mrole_EngineSimple.cpp
    mrole/powertrain/mrole_EngineSimple.h
    mrole/powertrain/mrole_EngineSimpleMap.cpp
    mrole/powertrain/mrole_EngineSimpleMap.h
)
set(CVM_MROLE_DRIVELINE_FILES
    mrole/driveline/mrole_Driveline2WD.cpp
    mrole/driveline/mrole_Driveline2WD.h
    mrole/driveline/mrole_Driveline4WD.cpp
    mrole/driveline/mrole_Driveline4WD.h
    mrole/driveline/mrole_Driveline6WD.h
    mrole/driveline/mrole_Driveline6WD.cpp
    mrole/driveline/mrole_Driveline8WD.h
    mrole/driveline/mrole_Driveline8WD.cpp
    mrole/driveline/mrole_SimpleDriveline.cpp
    mrole/driveline/mrole_SimpleDriveline.h
    mrole/driveline/mrole_SimpleDrivelineXWD.cpp
    mrole/driveline/mrole_SimpleDrivelineXWD.h
)
set(CVM_MROLE_VEHICLE_FILES
    mrole/mrole.cpp
    mrole/mrole.h
    mrole/mrole_BrakeSimple.cpp
    mrole/mrole_BrakeSimple.h
    mrole/mrole_BrakeShafts.cpp
    mrole/mrole_BrakeShafts.h
    mrole/mrole_Chassis.h
    mrole/mrole_Chassis.cpp
    mrole/mrole_DoubleWishbone.cpp
    mrole/mrole_DoubleWishbone.h
    mrole/mrole_DoubleWishboneReduced.cpp
    mrole/mrole_DoubleWishboneReduced.h
    mrole/mrole_RigidTire.cpp
    mrole/mrole_RigidTire.h
    mrole/mrole_TMeasyTire.cpp
    mrole/mrole_TMeasyTire.h
    mrole/mrole_PitmanArm.cpp
    mrole/mrole_PitmanArm.h
    mrole/mrole_PitmanArmShafts.cpp
    mrole/mrole_PitmanArmShafts.h
    mrole/mrole_RackPinion.cpp
    mrole/mrole_RackPinion.h
    mrole/mrole_Vehicle.h
    mrole/mrole_VehicleFull.cpp
    mrole/mrole_VehicleFull.h
    mrole/mrole_VehicleReduced.cpp
    mrole/mrole_VehicleReduced.h
    mrole/mrole_Wheel.cpp
    mrole/mrole_Wheel.h
)
set(CVM_MROLE_FILES
    ${CVM_MROLE_VEHICLE_FILES}
    ${CVM_MROLE_POWERTRAIN_FILES}
    ${CVM_MROLE_DRIVELINE_FILES}
)
source_group("mrole" FILES ${CVM_MROLE_VEHICLE_FILES})
source_group("mrole/powertrain" FILES ${CVM_MROLE_POWERTRAIN_FILES})
source_group("mrole/driveline" FILES ${CVM_MROLE_DRIVELINE_FILES})

set(CVM_FMTV_POWERTRAIN_FILES
    mtv/powertrain/FMTV_EngineShafts.cpp
    mtv/powertrain/FMTV_EngineShafts.h
    mtv/powertrain/FMTV_EngineSimple.cpp
    mtv/powertrain/FMTV_EngineSimple.h
    mtv/powertrain/FMTV_EngineSimpleMap.cpp
    mtv/powertrain/FMTV_EngineSimpleMap.h
    mtv/powertrain/FMTV_AutomaticTransmissionShafts.cpp
    mtv/powertrain/FMTV_AutomaticTransmissionShafts.h
    mtv/powertrain/FMTV_AutomaticTransmissionSimple.cpp
    mtv/powertrain/FMTV_AutomaticTransmissionSimple.h
    mtv/powertrain/FMTV_AutomaticTransmissionSimpleMap.cpp
    mtv/powertrain/FMTV_AutomaticTransmissionSimpleMap.h
)
set(CVM_FMTV_VEHICLE_FILES
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
    mtv/MTV_SpringDamper.cpp
    mtv/MTV_SpringDamper.h    
    mtv/MTV_LeafspringAxle1.cpp
    mtv/MTV_LeafspringAxle1.h
    mtv/MTV_LeafspringAxle2.cpp
    mtv/MTV_LeafspringAxle2.h
    mtv/MTV_Solid3LinkAxle1.cpp
    mtv/MTV_Solid3LinkAxle1.h
    mtv/MTV_Solid3LinkAxle2.cpp
    mtv/MTV_Solid3LinkAxle2.h
)
set(CVM_FMTV_FILES
    ${CVM_FMTV_POWERTRAIN_FILES}
    ${CVM_FMTV_VEHICLE_FILES}
)
source_group("mtv" FILES ${CVM_FMTV_VEHICLE_FILES})
source_group("mtv/powertrain" FILES ${CVM_FMTV_POWERTRAIN_FILES})

set(CVM_UAZ_FILES
    uaz/UAZBUS.cpp
    uaz/UAZBUS.h
    uaz/UAZBUS_AutomaticTransmissionSimpleMap.cpp
    uaz/UAZBUS_AutomaticTransmissionSimpleMap.h
    uaz/UAZBUS_EngineSimpleMap.cpp
    uaz/UAZBUS_EngineSimpleMap.h
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

set(CVM_UNIMOG_FILES
    unimog/U401.h
    unimog/U401.cpp
    unimog/U401_AutomaticTransmissionSimpleMap.h
    unimog/U401_AutomaticTransmissionSimpleMap.cpp
    unimog/U401_BrakeSimple.h
    unimog/U401_BrakeSimple.cpp
    unimog/U401_BrakeShafts.h
    unimog/U401_BrakeShafts.cpp
    unimog/U401_Chassis.h
    unimog/U401_Chassis.cpp
    unimog/U401_Driveline2WD.h
    unimog/U401_Driveline2WD.cpp
    unimog/U401_Driveline4WD.h
    unimog/U401_Driveline4WD.cpp
    unimog/U401_EngineSimpleMap.h
    unimog/U401_EngineSimpleMap.cpp
    unimog/U401_FialaTire.h
    unimog/U401_FialaTire.cpp
    unimog/U401_PushPipeAxle.h
    unimog/U401_PushPipeAxle.cpp
    unimog/U401_RotaryArm.h
    unimog/U401_RotaryArm.cpp
    unimog/U401_RigidTire.h
    unimog/U401_RigidTire.cpp
    unimog/U401_TMeasyTire.h
    unimog/U401_TMeasyTire.cpp
    unimog/U401_TMsimpleTire.h
    unimog/U401_TMsimpleTire.cpp
    unimog/U401_ToeBarPushPipeAxle.h
    unimog/U401_ToeBarPushPipeAxle.cpp
    unimog/U401_Vehicle.h
    unimog/U401_Vehicle.cpp
    unimog/U401_Wheel.h
    unimog/U401_Wheel.cpp
)
source_group("unimog" FILES ${CVM_UNIMOG_FILES})

set(CVM_DURO_FILES
    duro/Duro.h
    duro/Duro.cpp
    duro/Duro_BrakeShafts.h
    duro/Duro_BrakeShafts.cpp
    duro/Duro_Chassis.h
    duro/Duro_Chassis.cpp
    duro/Duro_DeDionAxle.h
    duro/Duro_DeDionAxle.cpp
    duro/Duro_Driveline4WD.h
    duro/Duro_Driveline4WD.cpp
    duro/Duro_AutomaticTransmissionShafts.h
    duro/Duro_AutomaticTransmissionShafts.cpp
    duro/Duro_EngineShafts.h
    duro/Duro_EngineShafts.cpp
    duro/Duro_RotaryArm.h
    duro/Duro_RotaryArm.cpp
    duro/Duro_TMeasyTire.h
    duro/Duro_TMeasyTire.cpp
    duro/Duro_TMsimpleTire.h
    duro/Duro_TMsimpleTire.cpp
    duro/Duro_ToeBarDeDionAxle.h
    duro/Duro_ToeBarDeDionAxle.cpp
    duro/Duro_RigidTire.h
    duro/Duro_RigidTire.cpp
    duro/Duro_Vehicle.h
    duro/Duro_Vehicle.cpp
    duro/Duro_Wheel.h
    duro/Duro_Wheel.cpp
)
source_group("duro" FILES ${CVM_DURO_FILES})

set(CVM_MAN_POWERTRAIN_FILES
    man/powertrain/MAN_5t_EngineSimple.h
    man/powertrain/MAN_5t_EngineSimple.cpp
    man/powertrain/MAN_5t_AutomaticTransmissionSimple.h
    man/powertrain/MAN_5t_AutomaticTransmissionSimple.cpp
    man/powertrain/MAN_5t_EngineSimpleMap.h
    man/powertrain/MAN_5t_EngineSimpleMap.cpp
    man/powertrain/MAN_5t_AutomaticTransmissionSimpleMap.h
    man/powertrain/MAN_5t_AutomaticTransmissionSimpleMap.cpp
    man/powertrain/MAN_7t_EngineSimple.h
    man/powertrain/MAN_7t_EngineSimple.cpp
    man/powertrain/MAN_7t_AutomaticTransmissionSimple.h
    man/powertrain/MAN_7t_AutomaticTransmissionSimple.cpp
    man/powertrain/MAN_7t_EngineSimpleMap.h
    man/powertrain/MAN_7t_EngineSimpleMap.cpp
    man/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.h
    man/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.cpp
)
set(CVM_MAN_SUSPENSION_FILES
    man/suspension/MAN_5t_BellcrankSolid3LinkAxle.cpp
    man/suspension/MAN_5t_BellcrankSolid3LinkAxle.h
    man/suspension/MAN_5t_Solid3LinkAxle.cpp
    man/suspension/MAN_5t_Solid3LinkAxle.h
    man/suspension/MAN_7t_Solid3LinkAxle.cpp
    man/suspension/MAN_7t_Solid3LinkAxle.h
    man/suspension/MAN_10t_Front1Axle.cpp
    man/suspension/MAN_10t_Front1Axle.h
    man/suspension/MAN_10t_Front2Axle.cpp
    man/suspension/MAN_10t_Front2Axle.h
 )
set(CVM_MAN_VEHICLE_FILES
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
    man/MAN_5t_TMeasyTire.cpp
    man/MAN_5t_TMeasyTire.h
    man/MAN_5t_TMsimpleTire.cpp
    man/MAN_5t_TMsimpleTire.h
    man/MAN_5t_Wheel.cpp
    man/MAN_5t_Wheel.h
    man/MAN_5t_Vehicle.cpp
    man/MAN_5t_Vehicle.h
    man/MAN_5t.cpp
    man/MAN_5t.h

    man/MAN_7t_Chassis.cpp
    man/MAN_7t_Chassis.h
    man/MAN_7t_Vehicle.cpp
    man/MAN_7t_Vehicle.h
    man/MAN_7t.cpp
    man/MAN_7t.h

    man/MAN_10t_Chassis.cpp
    man/MAN_10t_Chassis.h
    man/MAN_10t_RotaryArm2.cpp
    man/MAN_10t_RotaryArm2.h
    man/MAN_10t_Vehicle.cpp
    man/MAN_10t_Vehicle.h
    man/MAN_10t.cpp
    man/MAN_10t.h
)
set(CVM_MAN_FILES
    ${CVM_MAN_VEHICLE_FILES}
    ${CVM_MAN_POWERTRAIN_FILES}
    ${CVM_MAN_SUSPENSION_FILES}
)
source_group("man" FILES ${CVM_MAN_VEHICLE_FILES})
source_group("man/powertrain" FILES ${CVM_MAN_POWERTRAIN_FILES})
source_group("man/suspension" FILES ${CVM_MAN_SUSPENSION_FILES})

set(CVM_GENERIC_SUSPENSION_FILES
    generic/suspension/Generic_DoubleWishbone.cpp
    generic/suspension/Generic_DoubleWishbone.h
    generic/suspension/Generic_HendricksonPRIMAXX.cpp
    generic/suspension/Generic_HendricksonPRIMAXX.h
    generic/suspension/Generic_MacPhersonStrut.cpp
    generic/suspension/Generic_MacPhersonStrut.h
    generic/suspension/Generic_MultiLink.cpp
    generic/suspension/Generic_MultiLink.h
    generic/suspension/Generic_SolidAxle.cpp
    generic/suspension/Generic_SolidAxle.h
    generic/suspension/Generic_RigidSuspension.h
    generic/suspension/Generic_RigidSuspension.cpp
    generic/suspension/Generic_RigidPinnedAxle.h
    generic/suspension/Generic_RigidPinnedAxle.cpp
)
set(CVM_GENERIC_DRIVELINE_FILES
    generic/driveline/Generic_Driveline2WD.h
    generic/driveline/Generic_Driveline2WD.cpp
    generic/driveline/Generic_SimpleDriveline.cpp
    generic/driveline/Generic_SimpleDriveline.h
)
set(CVM_GENERIC_POWERTRAIN_FILES
    generic/powertrain/Generic_AutomaticTransmissionSimple.h
    generic/powertrain/Generic_AutomaticTransmissionSimple.cpp
    generic/powertrain/Generic_EngineSimple.h
    generic/powertrain/Generic_EngineSimple.cpp
    generic/powertrain/Generic_AutomaticTransmissionSimpleMap.h
    generic/powertrain/Generic_AutomaticTransmissionSimpleMap.cpp
    generic/powertrain/Generic_EngineSimpleMap.h
    generic/powertrain/Generic_EngineSimpleMap.cpp
)
set(CVM_GENERIC_TIRE_FILES
    generic/tire/Generic_FialaTire.h
    generic/tire/Generic_FialaTire.cpp
    generic/tire/Generic_RigidTire.h
    generic/tire/Generic_RigidTire.cpp
    generic/tire/Generic_RigidMeshTire.h
    generic/tire/Generic_RigidMeshTire.cpp
)
set(CVM_GENERIC_VEHICLE_FILES
    generic/Generic_AntirollBarRSD.h
    generic/Generic_AntirollBarRSD.cpp
    generic/Generic_BrakeSimple.h
    generic/Generic_BrakeSimple.cpp
    generic/Generic_Chassis.cpp
    generic/Generic_Chassis.h
    generic/Generic_FuncDriver.h
    generic/Generic_RackPinion.h
    generic/Generic_RackPinion.cpp
    generic/Generic_Vehicle.cpp
    generic/Generic_Vehicle.h
    generic/Generic_Wheel.h
    generic/Generic_Wheel.cpp
)
set(CVM_GENERIC_FILES
    ${CVM_GENERIC_VEHICLE_FILES}
    ${CVM_GENERIC_SUSPENSION_FILES}
    ${CVM_GENERIC_DRIVELINE_FILES}
    ${CVM_GENERIC_POWERTRAIN_FILES}
    ${CVM_GENERIC_TIRE_FILES}
)
source_group("generic" FILES ${CVM_GENERIC_VEHICLE_FILES})
source_group("generic/suspension" FILES ${CVM_GENERIC_SUSPENSION_FILES})
source_group("generic/driveline" FILES ${CVM_GENERIC_DRIVELINE_FILES})
source_group("generic/powertrain" FILES ${CVM_GENERIC_POWERTRAIN_FILES})
source_group("generic/tire" FILES ${CVM_GENERIC_TIRE_FILES})

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
    sedan/Sedan_TMsimpleTire.cpp
    sedan/Sedan_TMsimpleTire.h
    sedan/Sedan_Pac02Tire.cpp
    sedan/Sedan_Pac02Tire.h
    sedan/Sedan_RackPinion.cpp
    sedan/Sedan_RackPinion.h
    sedan/Sedan_AutomaticTransmissionSimpleMap.cpp
    sedan/Sedan_AutomaticTransmissionSimpleMap.h
    sedan/Sedan_EngineSimpleMap.cpp
    sedan/Sedan_EngineSimpleMap.h
    sedan/Sedan_Vehicle.h
    sedan/Sedan_Vehicle.cpp
    sedan/Sedan_Wheel.cpp
    sedan/Sedan_Wheel.h
    sedan/Sedan_AIDriver.cpp
    sedan/Sedan_AIDriver.h
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

    citybus/CityBus_EngineSimpleMap.cpp
    citybus/CityBus_EngineSimpleMap.h
    citybus/CityBus_AutomaticTransmissionSimpleMap.cpp
    citybus/CityBus_AutomaticTransmissionSimpleMap.h

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
    gator/Gator_EngineSimpleMap.cpp
    gator/Gator_EngineSimpleMap.h
    gator/Gator_EngineSimple.cpp
    gator/Gator_EngineSimple.h
    gator/Gator_AutomaticTransmissionSimpleMap.cpp
    gator/Gator_AutomaticTransmissionSimpleMap.h
    gator/Gator_AutomaticTransmissionSimple.cpp
    gator/Gator_AutomaticTransmissionSimple.h
    gator/Gator_Vehicle.h
    gator/Gator_Vehicle.cpp
    gator/Gator_Wheel.cpp
    gator/Gator_Wheel.h
)
source_group("gator" FILES ${CVM_GATOR_FILES})

set(CVM_MARDER_POWERTRAIN_FILES
    marder/powertrain/Marder_AutomaticTransmissionSimple.cpp
    marder/powertrain/Marder_AutomaticTransmissionSimple.h
    marder/powertrain/Marder_AutomaticTransmissionSimpleMap.cpp
    marder/powertrain/Marder_AutomaticTransmissionSimpleMap.h
    marder/powertrain/Marder_AutomaticTransmissionShafts.cpp
    marder/powertrain/Marder_AutomaticTransmissionShafts.h
    marder/powertrain/Marder_EngineSimple.cpp
    marder/powertrain/Marder_EngineSimple.h
    marder/powertrain/Marder_EngineSimpleMap.cpp
    marder/powertrain/Marder_EngineSimpleMap.h
    marder/powertrain/Marder_EngineShafts.cpp
    marder/powertrain/Marder_EngineShafts.h
)
set(CVM_MARDER_VEHICLE_FILES
    marder/Marder.h
    marder/Marder.cpp
    marder/Marder_Chassis.h
    marder/Marder_Chassis.cpp
    marder/Marder_BrakeSimple.h
    marder/Marder_BrakeShafts.h
    marder/Marder_Idler.h
    marder/Marder_Idler.cpp
    marder/Marder_IdlerWheel.h
    marder/Marder_IdlerWheel.cpp
    marder/Marder_RoadWheel.h
    marder/Marder_RoadWheel.cpp
    marder/Marder_SprocketSinglePin.h
    marder/Marder_SprocketSinglePin.cpp
    marder/Marder_SupportRoller.h
    marder/Marder_SupportRoller.cpp
    marder/Marder_TrackShoeSinglePin.h
    marder/Marder_TrackShoeSinglePin.cpp
    marder/Marder_Suspension.h
    marder/Marder_Suspension.cpp
    marder/Marder_SimpleDriveline.h 
    marder/Marder_SimpleDriveline.cpp
    marder/Marder_TrackAssemblySinglePin.h
    marder/Marder_TrackAssemblySinglePin.cpp
    marder/Marder_Vehicle.h
    marder/Marder_Vehicle.cpp
)

set(CVM_MARDER_FILES
    ${CVM_MARDER_VEHICLE_FILES}
    ${CVM_MARDER_POWERTRAIN_FILES}
)
source_group("marder" FILES ${CVM_MARDER_VEHICLE_FILES})
source_group("marder/powertrain" FILES ${CVM_MARDER_POWERTRAIN_FILES})

set(CVM_M113_TRACKASSEMBLY_FILES
    m113/track_assembly/M113_TrackAssemblySinglePin.cpp
    m113/track_assembly/M113_TrackAssemblySinglePin.h
    m113/track_assembly/M113_TrackAssemblyDoublePin.cpp
    m113/track_assembly/M113_TrackAssemblyDoublePin.h
    m113/track_assembly/M113_TrackAssemblyBandBushing.cpp
    m113/track_assembly/M113_TrackAssemblyBandBushing.h
    m113/track_assembly/M113_TrackAssemblyBandANCF.cpp
    m113/track_assembly/M113_TrackAssemblyBandANCF.h
)
set(CVM_M113_SPROCKET_FILES
    m113/sprocket/M113_SprocketSinglePin.cpp
    m113/sprocket/M113_SprocketSinglePin.h
    m113/sprocket/M113_SprocketDoublePin.cpp
    m113/sprocket/M113_SprocketDoublePin.h
    m113/sprocket/M113_SprocketBand.cpp
    m113/sprocket/M113_SprocketBand.h
)
set(CVM_M113_TRACKSHOE_FILES
    m113/track_shoe/M113_TrackShoeSinglePin.cpp
    m113/track_shoe/M113_TrackShoeSinglePin.h
    m113/track_shoe/M113_TrackShoeDoublePin.cpp
    m113/track_shoe/M113_TrackShoeDoublePin.h
    m113/track_shoe/M113_TrackShoeBandBushing.cpp
    m113/track_shoe/M113_TrackShoeBandBushing.h
    m113/track_shoe/M113_TrackShoeBandANCF.cpp
    m113/track_shoe/M113_TrackShoeBandANCF.h
)
set(CVM_M113_DRIVELINE_FILES
    m113/driveline/M113_DrivelineBDS.cpp
    m113/driveline/M113_DrivelineBDS.h
    m113/driveline/M113_SimpleDriveline.cpp
    m113/driveline/M113_SimpleDriveline.h
)
set(CVM_M113_POWERTRAIN_FILES
    m113/powertrain/M113_AutomaticTransmissionShafts.cpp
    m113/powertrain/M113_AutomaticTransmissionShafts.h
    m113/powertrain/M113_AutomaticTransmissionSimple.cpp
    m113/powertrain/M113_AutomaticTransmissionSimple.h
    m113/powertrain/M113_AutomaticTransmissionSimpleMap.cpp
    m113/powertrain/M113_AutomaticTransmissionSimpleMap.h
    m113/powertrain/M113_EngineShafts.cpp
    m113/powertrain/M113_EngineShafts.h
    m113/powertrain/M113_EngineSimple.cpp
    m113/powertrain/M113_EngineSimple.h
    m113/powertrain/M113_EngineSimpleMap.cpp
    m113/powertrain/M113_EngineSimpleMap.h
)
set(CVM_M113_VEHICLE_FILES
    m113/M113.h
    m113/M113.cpp
    m113/M113_BrakeSimple.h
    m113/M113_BrakeShafts.h
    m113/M113_Chassis.h
    m113/M113_Chassis.cpp
    m113/M113_Idler.cpp
    m113/M113_Idler.h
    m113/M113_IdlerWheel.cpp
    m113/M113_IdlerWheel.h
    m113/M113_RoadWheel.cpp
    m113/M113_RoadWheel.h
    m113/M113_Suspension.cpp
    m113/M113_Suspension.h
    m113/M113_Vehicle.cpp
    m113/M113_Vehicle.h
)
set(CVM_M113_FILES
    ${CVM_M113_VEHICLE_FILES}
    ${CVM_M113_TRACKASSEMBLY_FILES}
    ${CVM_M113_SPROCKET_FILES}
    ${CVM_M113_TRACKSHOE_FILES}
    ${CVM_M113_DRIVELINE_FILES}
    ${CVM_M113_POWERTRAIN_FILES}
)
source_group("m113" FILES ${CVM_M113_VEHICLE_FILES})
source_group("m113/track_assembly" FILES ${CVM_M113_TRACKASSEMBLY_FILES})
source_group("m113/sprocket" FILES ${CVM_M113_SPROCKET_FILES})
source_group("m113/track_shoe" FILES ${CVM_M113_TRACKSHOE_FILES})
source_group("m113/driveline" FILES ${CVM_M113_DRIVELINE_FILES})
source_group("m113/powertrain" FILES ${CVM_M113_POWERTRAIN_FILES})

set(CVM_RCCAR_FILES
    rccar/RCCar_BrakeSimple.cpp
    rccar/RCCar_BrakeSimple.h
    rccar/RCCar_Chassis.cpp
    rccar/RCCar_Chassis.h
    rccar/RCCar_DoubleWishbone.cpp
    rccar/RCCar_DoubleWishbone.h
    rccar/RCCar_Driveline4WD.cpp
    rccar/RCCar_Driveline4WD.h
    rccar/RCCar_PitmanArm.cpp
    rccar/RCCar_PitmanArm.h
    rccar/RCCar_RigidTire.cpp
    rccar/RCCar_RigidTire.h
    rccar/RCCar_TMeasyTire.cpp
    rccar/RCCar_TMeasyTire.h
    rccar/RCCar_AutomaticTransmissionSimpleMap.cpp
    rccar/RCCar_AutomaticTransmissionSimpleMap.h
    rccar/RCCar_EngineSimpleMap.cpp
    rccar/RCCar_EngineSimpleMap.h
    rccar/RCCar_Vehicle.cpp
    rccar/RCCar_Vehicle.h
    rccar/RCCar_Wheel.cpp
    rccar/RCCar_Wheel.h
    rccar/RCCar.cpp
    rccar/RCCar.h
)
source_group("rccar" FILES ${CVM_RCCAR_FILES})

set(CVM_KRAZ_FILES
    kraz/Kraz.cpp
    kraz/Kraz.h
    kraz/Kraz_tractor.cpp
    kraz/Kraz_tractor.h
    kraz/Kraz_tractor_AutomaticTransmissionSimpleMap.cpp
    kraz/Kraz_tractor_AutomaticTransmissionSimpleMap.h
    kraz/Kraz_tractor_Brake.cpp
    kraz/Kraz_tractor_Brake.h
    kraz/Kraz_tractor_Chassis.cpp
    kraz/Kraz_tractor_Chassis.h
    kraz/Kraz_tractor_Driveline.cpp
    kraz/Kraz_tractor_Driveline.h
    kraz/Kraz_tractor_FrontSuspension.cpp
    kraz/Kraz_tractor_FrontSuspension.h
    kraz/Kraz_tractor_EngineSimpleMap.cpp
    kraz/Kraz_tractor_EngineSimpleMap.h
    kraz/Kraz_tractor_RearSuspension.cpp
    kraz/Kraz_tractor_RearSuspension.h
    kraz/Kraz_tractor_Steering.cpp
    kraz/Kraz_tractor_Steering.h
    kraz/Kraz_tractor_Tire.cpp
    kraz/Kraz_tractor_Tire.h
    kraz/Kraz_tractor_Wheel.cpp
    kraz/Kraz_tractor_Wheel.h
    kraz/Kraz_trailer.cpp
    kraz/Kraz_trailer.h
    kraz/Kraz_trailer_Brake.cpp
    kraz/Kraz_trailer_Brake.h
    kraz/Kraz_trailer_Chassis.cpp
    kraz/Kraz_trailer_Chassis.h
    kraz/Kraz_trailer_Suspension.cpp
    kraz/Kraz_trailer_Suspension.h
    kraz/Kraz_trailer_Tire.cpp
    kraz/Kraz_trailer_Tire.h
    kraz/Kraz_trailer_Wheel.cpp
    kraz/Kraz_trailer_Wheel.h
)
source_group("kraz" FILES ${CVM_KRAZ_FILES})

# ----------------------------------------------------------------------------
# Add the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)

add_library(ChronoModels_vehicle SHARED
    ${CVM_COMMON_FILES}
    ${CVM_HMMWV_FILES}
    ${CVM_FEDA_FILES}
    ${CVM_FMTV_FILES}
    ${CVM_MROLE_FILES}
    ${CVM_UAZ_FILES}
    ${CVM_UNIMOG_FILES}
    ${CVM_DURO_FILES}
    ${CVM_GENERIC_FILES}
    ${CVM_SEDAN_FILES}
    ${CVM_CITYBUS_FILES}
    ${CVM_GATOR_FILES}
    ${CVM_M113_FILES}
    ${CVM_MARDER_FILES}
    ${CVM_MAN_FILES}
    ${CVM_RCCAR_FILES}
    ${CVM_KRAZ_FILES}
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
