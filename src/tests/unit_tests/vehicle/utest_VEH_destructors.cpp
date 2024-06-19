// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test to check destructors for various Chrono::Vehicle ChPart classes.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChassisConnectorArticulated.h"
#include "chrono_vehicle/chassis/ChassisConnectorHitch.h"
#include "chrono_vehicle/chassis/ChassisConnectorTorsion.h"

#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/EngineShafts.h"
#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/powertrain/EngineSimpleMap.h"
#include "chrono_vehicle/powertrain/ManualTransmissionShafts.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_vehicle/wheeled_vehicle/antirollbar/AntirollBarRSD.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeShafts.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDrivelineXWD.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RotaryArm.h"

#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/DeDionAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/GenericWheeledSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/HendricksonPRIMAXX.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/LeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/PushPipeAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPanhardAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPinnedAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SAELeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SemiTrailingArm.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SingleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarDeDionAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarPushPipeAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarRigidPanhardAxle.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerToroidalTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"

#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

using namespace chrono;
using namespace chrono::vehicle;

TEST(ChronoVehicle, destructors_chassis) {
    // Chassis templates

    { chrono_types::make_shared<RigidChassis>(vehicle::GetDataFile("articulated_chassis/ACV_FrontChassis.json")); }
    { chrono_types::make_shared<RigidChassisRear>(vehicle::GetDataFile("articulated_chassis/ACV_RearChassis.json")); }

    // Chassis connector templates

    {
        chrono_types::make_shared<ChassisConnectorArticulated>(
            vehicle::GetDataFile("articulated_chassis/ACV_Connector.json"));
    }
    { chrono_types::make_shared<ChassisConnectorHitch>(vehicle::GetDataFile("ultra_tow/UT_Hitch.json")); }
    {
        chrono_types::make_shared<ChassisConnectorTorsion>(
            vehicle::GetDataFile("mtv/chassis/MTV_ChassisConnector.json"));
    }
}

TEST(ChronoVehicle, destructors_powertrain) {
    // Engine templates

    { chrono_types::make_shared<EngineShafts>(vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json")); }
    { chrono_types::make_shared<EngineSimple>(vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineSimple.json")); }
    { chrono_types::make_shared<EngineSimpleMap>(vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineSimpleMap.json")); }

    // Transmission templates

    {
        chrono_types::make_shared<AutomaticTransmissionShafts>(
            vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json"));
    }
    {
        chrono_types::make_shared<AutomaticTransmissionSimpleMap>(
            vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json"));
    }
    {
        chrono_types::make_shared<ManualTransmissionShafts>(
            vehicle::GetDataFile("audi/json/audi_ManualTransmissionShafts.json"));
    }
}

TEST(ChronoVehicle, destructors_wheeled) {
    // Antiroll bar templates

    { chrono_types::make_shared<AntirollBarRSD>(vehicle::GetDataFile("generic/antirollbar/AntirollBarRSD.json")); }

    // Brake templates

    { chrono_types::make_shared<BrakeSimple>(vehicle::GetDataFile("generic/brake/BrakeSimple.json")); }
    { chrono_types::make_shared<BrakeShafts>(vehicle::GetDataFile("generic/brake/BrakeShafts.json")); }
    { chrono_types::make_shared<BrakeShafts>(vehicle::GetDataFile("generic/brake/BrakeShafts.json")); }

    // Driveline templates

    { chrono_types::make_shared<ShaftsDriveline2WD>(vehicle::GetDataFile("generic/driveline/Driveline2WD.json")); }
    { chrono_types::make_shared<ShaftsDriveline4WD>(vehicle::GetDataFile("generic/driveline/Driveline4WD.json")); }
    { chrono_types::make_shared<SimpleDriveline>(vehicle::GetDataFile("generic/driveline/DrivelineSimple.json")); }
    {
        chrono_types::make_shared<SimpleDrivelineXWD>(
            vehicle::GetDataFile("MAN_Kat1/driveline/MAN_5t_DrivelineSimpleXWD.json"));
    }

    // Steering templates

    { chrono_types::make_shared<PitmanArm>(vehicle::GetDataFile("generic/steering/PitmanArm.json")); }
    { chrono_types::make_shared<RackPinion>(vehicle::GetDataFile("generic/steering/RackPinion.json")); }
    { chrono_types::make_shared<RotaryArm>(vehicle::GetDataFile("MAN_Kat1/steering/MAN_10t_RotaryArm2.json")); }

    // Subchassis templates

    { chrono_types::make_shared<Balancer>(vehicle::GetDataFile("mtv/chassis/MTV_Balancer.json")); }

    // Suspension templates

    { chrono_types::make_shared<DeDionAxle>(vehicle::GetDataFile("generic/suspension/DeDionAxle.json")); }
    {
        chrono_types::make_shared<DoubleWishbone>(
            vehicle::GetDataFile("hmmwv/suspension/HMMWV_DoubleWishboneFront.json"));
    }
    {
        chrono_types::make_shared<DoubleWishboneReduced>(
            vehicle::GetDataFile("hmmwv/suspension/HMMWV_DoubleWishboneReducedFront.json"));
    }
    {
        chrono_types::make_shared<GenericWheeledSuspension>(
            vehicle::GetDataFile("hmmwv/suspension/HMMWV_DoubleWishboneFront_replica.json"));
    }
    {
        chrono_types::make_shared<HendricksonPRIMAXX>(
            vehicle::GetDataFile("generic/suspension/HendricksonPRIMAXX.json"));
    }
    {
        chrono_types::make_shared<LeafspringAxle>(
            vehicle::GetDataFile("uaz/suspension/UAZBUS_RearLeafspringAxle.json"));
    }
    { chrono_types::make_shared<MacPhersonStrut>(vehicle::GetDataFile("generic/suspension/MacPhersonStrut.json")); }
    { chrono_types::make_shared<MultiLink>(vehicle::GetDataFile("generic/suspension/MultiLink.json")); }
    { chrono_types::make_shared<PushPipeAxle>(vehicle::GetDataFile("uaz/suspension/UAZBUS_RearPushPipeAxle.json")); }
    { chrono_types::make_shared<RigidPanhardAxle>(vehicle::GetDataFile("gclass/suspension/G500_RearAxle.json")); }
    { chrono_types::make_shared<RigidPinnedAxle>(vehicle::GetDataFile("generic/suspension/RigidPinnedAxle.json")); }
    { chrono_types::make_shared<RigidSuspension>(vehicle::GetDataFile("generic/suspension/RigidSuspension.json")); }
    {
        chrono_types::make_shared<SAELeafspringAxle>(
            vehicle::GetDataFile("uaz/suspension/UAZBUS_RearSAELeafspringAxle.json"));
    }
    {
        chrono_types::make_shared<SAEToeBarLeafspringAxle>(
            vehicle::GetDataFile("uaz/suspension/UAZBUS_FrontSAELeafspringAxle.json"));
    }
    { chrono_types::make_shared<SemiTrailingArm>(vehicle::GetDataFile("generic/suspension/SemiTrailingArm.json")); }
    { chrono_types::make_shared<SingleWishbone>(vehicle::GetDataFile("gator/json/Gator_SingleWishboneFront.json")); }
    { chrono_types::make_shared<SolidAxle>(vehicle::GetDataFile("generic/suspension/SolidAxle.json")); }
    {
        chrono_types::make_shared<SolidBellcrankThreeLinkAxle>(
            vehicle::GetDataFile("MAN_Kat1/suspension/MAN_5t_FrontSolidThreeLinkAxle.json"));
    }
    {
        chrono_types::make_shared<SolidThreeLinkAxle>(
            vehicle::GetDataFile("MAN_Kat1/suspension/MAN_5t_RearSolidThreeLinkAxle.json"));
    }
    { chrono_types::make_shared<ThreeLinkIRS>(vehicle::GetDataFile("generic/suspension/ThreeLinkIRS.json")); }
    { chrono_types::make_shared<ToeBarDeDionAxle>(vehicle::GetDataFile("generic/suspension/ToeBarDeDionAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarLeafspringAxle>(
            vehicle::GetDataFile("uaz/suspension/UAZBUS_FrontLeafspringAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarPushPipeAxle>(
            vehicle::GetDataFile("uaz/suspension/UAZBUS_FrontPushPipeAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarRigidPanhardAxle>(
            vehicle::GetDataFile("gclass/suspension/G500_FrontAxle.json"));
    }

    // Tire templates

    { chrono_types::make_shared<ANCFTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_ANCF4Tire.json")); }
    { chrono_types::make_shared<ANCFToroidalTire>(vehicle::GetDataFile("generic/tire/ANCFToroidalTire.json")); }
    { chrono_types::make_shared<FEATire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_FEATire.json")); }
    { chrono_types::make_shared<FialaTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_FialaTire.json")); }
    { chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_Pac02Tire.json")); }
    { chrono_types::make_shared<Pac89Tire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_Pac89Tire.json")); }
    { chrono_types::make_shared<ReissnerTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_ReissnerTire.json")); }
    { chrono_types::make_shared<ReissnerToroidalTire>(vehicle::GetDataFile("generic/tire/ReissnerToroidalTire.json")); }
    { chrono_types::make_shared<RigidTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_RigidMeshTire.json")); }
    { chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json")); }
    { chrono_types::make_shared<TMsimpleTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_TMsimpleTire.json")); }

    // Wheel templates

    { chrono_types::make_shared<Wheel>(vehicle::GetDataFile("hmmwv/wheel/HMMWV_Wheel.json")); }

    // Vehicle and trailer templates

    { chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json")); }
    {
        auto veh = chrono_types::make_shared<WheeledVehicle>(
            vehicle::GetDataFile("generic/vehicle/Vehicle_DoubleWishbones.json"));
        chrono_types::make_shared<WheeledTrailer>(veh->GetSystem(), vehicle::GetDataFile("ultra_tow/UT_Trailer.json"),
                                                  false);
    }
}
