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

#include "chrono_vehicle/ChVehicleDataPath.h"

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

    { chrono_types::make_shared<RigidChassis>(GetVehicleDataFile("articulated_chassis/ACV_FrontChassis.json")); }
    { chrono_types::make_shared<RigidChassisRear>(GetVehicleDataFile("articulated_chassis/ACV_RearChassis.json")); }

    // Chassis connector templates

    {
        chrono_types::make_shared<ChassisConnectorArticulated>(
            GetVehicleDataFile("articulated_chassis/ACV_Connector.json"));
    }
    { chrono_types::make_shared<ChassisConnectorHitch>(GetVehicleDataFile("ultra_tow/UT_Hitch.json")); }
    {
        chrono_types::make_shared<ChassisConnectorTorsion>(
            GetVehicleDataFile("mtv/chassis/MTV_ChassisConnector.json"));
    }
}

TEST(ChronoVehicle, destructors_powertrain) {
    // Engine templates

    { chrono_types::make_shared<EngineShafts>(GetVehicleDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json")); }
    { chrono_types::make_shared<EngineSimple>(GetVehicleDataFile("hmmwv/powertrain/HMMWV_EngineSimple.json")); }
    { chrono_types::make_shared<EngineSimpleMap>(GetVehicleDataFile("hmmwv/powertrain/HMMWV_EngineSimpleMap.json")); }

    // Transmission templates

    {
        chrono_types::make_shared<AutomaticTransmissionShafts>(
            GetVehicleDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json"));
    }
    {
        chrono_types::make_shared<AutomaticTransmissionSimpleMap>(
            GetVehicleDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json"));
    }
    {
        chrono_types::make_shared<ManualTransmissionShafts>(
            GetVehicleDataFile("audi/json/audi_ManualTransmissionShafts.json"));
    }
}

TEST(ChronoVehicle, destructors_wheeled) {
    // Antiroll bar templates

    { chrono_types::make_shared<AntirollBarRSD>(GetVehicleDataFile("generic/antirollbar/AntirollBarRSD.json")); }

    // Brake templates

    { chrono_types::make_shared<BrakeSimple>(GetVehicleDataFile("generic/brake/BrakeSimple.json")); }
    { chrono_types::make_shared<BrakeShafts>(GetVehicleDataFile("generic/brake/BrakeShafts.json")); }
    { chrono_types::make_shared<BrakeShafts>(GetVehicleDataFile("generic/brake/BrakeShafts.json")); }

    // Driveline templates

    { chrono_types::make_shared<ShaftsDriveline2WD>(GetVehicleDataFile("generic/driveline/Driveline2WD.json")); }
    { chrono_types::make_shared<ShaftsDriveline4WD>(GetVehicleDataFile("generic/driveline/Driveline4WD.json")); }
    { chrono_types::make_shared<SimpleDriveline>(GetVehicleDataFile("generic/driveline/DrivelineSimple.json")); }
    {
        chrono_types::make_shared<SimpleDrivelineXWD>(
            GetVehicleDataFile("MAN_Kat1/driveline/MAN_5t_DrivelineSimpleXWD.json"));
    }

    // Steering templates

    { chrono_types::make_shared<PitmanArm>(GetVehicleDataFile("generic/steering/PitmanArm.json")); }
    { chrono_types::make_shared<RackPinion>(GetVehicleDataFile("generic/steering/RackPinion.json")); }
    { chrono_types::make_shared<RotaryArm>(GetVehicleDataFile("MAN_Kat1/steering/MAN_10t_RotaryArm2.json")); }

    // Subchassis templates

    { chrono_types::make_shared<Balancer>(GetVehicleDataFile("mtv/chassis/MTV_Balancer.json")); }

    // Suspension templates

    { chrono_types::make_shared<DeDionAxle>(GetVehicleDataFile("generic/suspension/DeDionAxle.json")); }
    {
        chrono_types::make_shared<DoubleWishbone>(
            GetVehicleDataFile("hmmwv/suspension/HMMWV_DoubleWishboneFront.json"));
    }
    {
        chrono_types::make_shared<DoubleWishboneReduced>(
            GetVehicleDataFile("hmmwv/suspension/HMMWV_DoubleWishboneReducedFront.json"));
    }
    {
        chrono_types::make_shared<GenericWheeledSuspension>(
            GetVehicleDataFile("hmmwv/suspension/HMMWV_DoubleWishboneFront_replica.json"));
    }
    {
        chrono_types::make_shared<HendricksonPRIMAXX>(
            GetVehicleDataFile("generic/suspension/HendricksonPRIMAXX.json"));
    }
    {
        chrono_types::make_shared<LeafspringAxle>(
            GetVehicleDataFile("uaz/suspension/UAZBUS_RearLeafspringAxle.json"));
    }
    { chrono_types::make_shared<MacPhersonStrut>(GetVehicleDataFile("generic/suspension/MacPhersonStrut.json")); }
    { chrono_types::make_shared<MultiLink>(GetVehicleDataFile("generic/suspension/MultiLink.json")); }
    { chrono_types::make_shared<PushPipeAxle>(GetVehicleDataFile("uaz/suspension/UAZBUS_RearPushPipeAxle.json")); }
    { chrono_types::make_shared<RigidPanhardAxle>(GetVehicleDataFile("gclass/suspension/G500_RearAxle.json")); }
    { chrono_types::make_shared<RigidPinnedAxle>(GetVehicleDataFile("generic/suspension/RigidPinnedAxle.json")); }
    { chrono_types::make_shared<RigidSuspension>(GetVehicleDataFile("generic/suspension/RigidSuspension.json")); }
    {
        chrono_types::make_shared<SAELeafspringAxle>(
            GetVehicleDataFile("uaz/suspension/UAZBUS_RearSAELeafspringAxle.json"));
    }
    {
        chrono_types::make_shared<SAEToeBarLeafspringAxle>(
            GetVehicleDataFile("uaz/suspension/UAZBUS_FrontSAELeafspringAxle.json"));
    }
    { chrono_types::make_shared<SemiTrailingArm>(GetVehicleDataFile("generic/suspension/SemiTrailingArm.json")); }
    { chrono_types::make_shared<SingleWishbone>(GetVehicleDataFile("gator/json/Gator_SingleWishboneFront.json")); }
    { chrono_types::make_shared<SolidAxle>(GetVehicleDataFile("generic/suspension/SolidAxle.json")); }
    {
        chrono_types::make_shared<SolidBellcrankThreeLinkAxle>(
            GetVehicleDataFile("MAN_Kat1/suspension/MAN_5t_FrontSolidThreeLinkAxle.json"));
    }
    {
        chrono_types::make_shared<SolidThreeLinkAxle>(
            GetVehicleDataFile("MAN_Kat1/suspension/MAN_5t_RearSolidThreeLinkAxle.json"));
    }
    { chrono_types::make_shared<ThreeLinkIRS>(GetVehicleDataFile("generic/suspension/ThreeLinkIRS.json")); }
    { chrono_types::make_shared<ToeBarDeDionAxle>(GetVehicleDataFile("generic/suspension/ToeBarDeDionAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarLeafspringAxle>(
            GetVehicleDataFile("uaz/suspension/UAZBUS_FrontLeafspringAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarPushPipeAxle>(
            GetVehicleDataFile("uaz/suspension/UAZBUS_FrontPushPipeAxle.json"));
    }
    {
        chrono_types::make_shared<ToeBarRigidPanhardAxle>(
            GetVehicleDataFile("gclass/suspension/G500_FrontAxle.json"));
    }

    // Tire templates

    { chrono_types::make_shared<ANCFTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_ANCF4Tire.json")); }
    { chrono_types::make_shared<ANCFToroidalTire>(GetVehicleDataFile("generic/tire/ANCFToroidalTire.json")); }
    { chrono_types::make_shared<FEATire>(GetVehicleDataFile("hmmwv/tire/HMMWV_FEATire.json")); }
    { chrono_types::make_shared<FialaTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_FialaTire.json")); }
    { chrono_types::make_shared<Pac02Tire>(GetVehicleDataFile("hmmwv/tire/HMMWV_Pac02Tire.json")); }
    { chrono_types::make_shared<Pac89Tire>(GetVehicleDataFile("hmmwv/tire/HMMWV_Pac89Tire.json")); }
    { chrono_types::make_shared<ReissnerTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_ReissnerTire.json")); }
    { chrono_types::make_shared<ReissnerToroidalTire>(GetVehicleDataFile("generic/tire/ReissnerToroidalTire.json")); }
    { chrono_types::make_shared<RigidTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_RigidMeshTire.json")); }
    { chrono_types::make_shared<TMeasyTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_TMeasyTire.json")); }
    { chrono_types::make_shared<TMsimpleTire>(GetVehicleDataFile("hmmwv/tire/HMMWV_TMsimpleTire.json")); }

    // Wheel templates

    { chrono_types::make_shared<Wheel>(GetVehicleDataFile("hmmwv/wheel/HMMWV_Wheel.json")); }

    // Vehicle and trailer templates

    { chrono_types::make_shared<WheeledVehicle>(GetVehicleDataFile("hmmwv/vehicle/HMMWV_Vehicle.json")); }
    {
        auto veh = chrono_types::make_shared<WheeledVehicle>(
            GetVehicleDataFile("generic/vehicle/Vehicle_DoubleWishbones.json"));
        chrono_types::make_shared<WheeledTrailer>(veh->GetSystem(), GetVehicleDataFile("ultra_tow/UT_Trailer.json"),
                                                  false);
    }
}
