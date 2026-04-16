// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#include <fstream>
#include <vector>
#include <utility>

#include "chrono/utils/ChForceFunctors.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChassisConnectorHitch.h"
#include "chrono_vehicle/chassis/ChassisConnectorArticulated.h"
#include "chrono_vehicle/chassis/ChassisConnectorTorsion.h"
#include "chrono_vehicle/chassis/ChassisConnectorFifthWheel.h"

#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/powertrain/EngineSimpleMap.h"
#include "chrono_vehicle/powertrain/EngineShafts.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleCVT.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionShafts.h"
#include "chrono_vehicle/powertrain/ManualTransmissionShafts.h"

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
#include "chrono_vehicle/wheeled_vehicle/suspension/DeDionAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/HendricksonPRIMAXX.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/LeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SAELeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPinnedAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SemiTrailingArm.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/PushPipeAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarPushPipeAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPanhardAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarRigidPanhardAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SingleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarDeDionAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/GenericWheeledSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeShafts.h"
#include "chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/driveline/TrackDrivelineBDS.h"
#include "chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/SingleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandANCF.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyBandBushing.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/TrackAssemblySinglePin.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

std::shared_ptr<ChChassis> ReadChassisJSON(const std::string& filename) {
    std::shared_ptr<ChChassis> chassis;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a chassis specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Chassis") == 0);

    // Extract the chassis type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the chassis using the appropriate template.
    if (subtype.compare("RigidChassis") == 0) {
        chassis = chrono_types::make_shared<RigidChassis>(d);
    } else {
        throw std::invalid_argument("Chassis type not supported in ReadChassisJSON.");
    }

    return chassis;
}

std::shared_ptr<ChChassisRear> ReadChassisRearJSON(const std::string& filename) {
    std::shared_ptr<ChChassisRear> chassis;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a rear chassis specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("ChassisRear") == 0);

    // Extract the chassis type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the chassis using the appropriate template.
    if (subtype.compare("RigidChassisRear") == 0) {
        chassis = chrono_types::make_shared<RigidChassisRear>(d);
    } else {
        throw std::invalid_argument("Chassis type not supported in ReadChassisRearJSON.");
    }

    return chassis;
}

std::shared_ptr<ChChassisConnector> ReadChassisConnectorJSON(const std::string& filename) {
    std::shared_ptr<ChChassisConnector> connector;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a chassis connector specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("ChassisConnector") == 0);

    // Extract the connector type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the connector using the appropriate template.
    if (subtype.compare("ChassisConnectorHitch") == 0) {
        connector = chrono_types::make_shared<ChassisConnectorHitch>(d);
    } else if (subtype.compare("ChassisConnectorArticulated") == 0) {
        connector = chrono_types::make_shared<ChassisConnectorArticulated>(d);
    } else if (subtype.compare("ChassisConnectorTorsion") == 0) {
        connector = chrono_types::make_shared<ChassisConnectorTorsion>(d);
    } else if (subtype.compare("ChassisConnectorFifthWheel") == 0) {
        connector = chrono_types::make_shared<ChassisConnectorFifthWheel>(d);
    } else {
        throw std::invalid_argument("Connector type not supported in ReadChassisConnectorJSON.");
    }

    return connector;
}

std::shared_ptr<ChEngine> ReadEngineJSON(const std::string& filename) {
    std::shared_ptr<ChEngine> engine;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is an engine specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Engine") == 0);

    // Extract the engine type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the engine using the appropriate template.
    if (subtype.compare("EngineSimple") == 0) {
        engine = chrono_types::make_shared<EngineSimple>(d);
    } else if (subtype.compare("EngineSimpleMap") == 0) {
        engine = chrono_types::make_shared<EngineSimpleMap>(d);
    } else if (subtype.compare("EngineShafts") == 0) {
        engine = chrono_types::make_shared<EngineShafts>(d);
    } else {
        throw std::invalid_argument("Engine type not supported in ReadEngineJSON.");
    }

    return engine;
}

std::shared_ptr<ChTransmission> ReadTransmissionJSON(const std::string& filename) {
    std::shared_ptr<ChTransmission> transmission;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a transmission specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Transmission") == 0);

    // Extract the transmission type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the transmission using the appropriate template.
    if (subtype.compare("AutomaticTransmissionSimpleMap") == 0) {
        transmission = chrono_types::make_shared<AutomaticTransmissionSimpleMap>(d);
    } else if (subtype.compare("AutomaticTransmissionSimpleCVT") == 0) {
        transmission = chrono_types::make_shared<AutomaticTransmissionSimpleCVT>(d);
    } else if (subtype.compare("AutomaticTransmissionShafts") == 0) {
        transmission = chrono_types::make_shared<AutomaticTransmissionShafts>(d);
    } else if (subtype.compare("ManualTransmissionShafts") == 0) {
        transmission = chrono_types::make_shared<ManualTransmissionShafts>(d);
    } else {
        throw std::invalid_argument("Transmission type not supported in ReadTransmissionJSON.");
    }

    return transmission;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChSuspension> ReadSuspensionJSON(const std::string& filename) {
    std::shared_ptr<ChSuspension> suspension;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a suspension specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Suspension") == 0);

    // Extract the suspension type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the suspension using the appropriate template.
    if (subtype.compare("DoubleWishbone") == 0) {
        suspension = chrono_types::make_shared<DoubleWishbone>(d);
    } else if (subtype.compare("DoubleWishboneReduced") == 0) {
        suspension = chrono_types::make_shared<DoubleWishboneReduced>(d);
    } else if (subtype.compare("SolidAxle") == 0) {
        suspension = chrono_types::make_shared<SolidAxle>(d);
    } else if (subtype.compare("DeDionAxle") == 0) {
        suspension = chrono_types::make_shared<DeDionAxle>(d);
    } else if (subtype.compare("ToeBarDeDionAxle") == 0) {
        suspension = chrono_types::make_shared<ToeBarDeDionAxle>(d);
    } else if (subtype.compare("MultiLink") == 0) {
        suspension = chrono_types::make_shared<MultiLink>(d);
    } else if (subtype.compare("MacPhersonStrut") == 0) {
        suspension = chrono_types::make_shared<MacPhersonStrut>(d);
    } else if (subtype.compare("SemiTrailingArm") == 0) {
        suspension = chrono_types::make_shared<SemiTrailingArm>(d);
    } else if (subtype.compare("ThreeLinkIRS") == 0) {
        suspension = chrono_types::make_shared<ThreeLinkIRS>(d);
    } else if (subtype.compare("ToeBarLeafspringAxle") == 0) {
        suspension = chrono_types::make_shared<ToeBarLeafspringAxle>(d);
    } else if (subtype.compare("SAEToeBarLeafspringAxle") == 0) {
        suspension = chrono_types::make_shared<SAEToeBarLeafspringAxle>(d);
    } else if (subtype.compare("LeafspringAxle") == 0) {
        suspension = chrono_types::make_shared<LeafspringAxle>(d);
    } else if (subtype.compare("SAELeafspringAxle") == 0) {
        suspension = chrono_types::make_shared<SAELeafspringAxle>(d);
    } else if (subtype.compare("SingleWishbone") == 0) {
        suspension = chrono_types::make_shared<SingleWishbone>(d);
    } else if (subtype.compare("SolidThreeLinkAxle") == 0) {
        suspension = chrono_types::make_shared<SolidThreeLinkAxle>(d);
    } else if (subtype.compare("SolidBellcrankThreeLinkAxle") == 0) {
        suspension = chrono_types::make_shared<SolidBellcrankThreeLinkAxle>(d);
    } else if (subtype.compare("RigidSuspension") == 0) {
        suspension = chrono_types::make_shared<RigidSuspension>(d);
    } else if (subtype.compare("RigidPinnedAxle") == 0) {
        suspension = chrono_types::make_shared<RigidPinnedAxle>(d);
    } else if (subtype.compare("PushPipeAxle") == 0) {
        suspension = chrono_types::make_shared<PushPipeAxle>(d);
    } else if (subtype.compare("RigidPanhardAxle") == 0) {
        suspension = chrono_types::make_shared<RigidPanhardAxle>(d);
    } else if (subtype.compare("ToeBarRigidPanhardAxle") == 0) {
        suspension = chrono_types::make_shared<ToeBarRigidPanhardAxle>(d);
    } else if (subtype.compare("ToeBarPushPipeAxle") == 0) {
        suspension = chrono_types::make_shared<ToeBarPushPipeAxle>(d);
    } else if (subtype.compare("HendricksonPRIMAXX") == 0) {
        suspension = chrono_types::make_shared<HendricksonPRIMAXX>(d);
    } else if (subtype.compare("GenericWheeledSuspension") == 0) {
        suspension = chrono_types::make_shared<GenericWheeledSuspension>(d);
    } else {
        throw std::invalid_argument("Suspension type not supported in ReadSuspensionJSON.");
    }

    return suspension;
}

std::shared_ptr<ChSteering> ReadSteeringJSON(const std::string& filename) {
    std::shared_ptr<ChSteering> steering;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Steering") == 0);

    // Extract the steering type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("PitmanArm") == 0) {
        steering = chrono_types::make_shared<PitmanArm>(d);
    } else if (subtype.compare("RackPinion") == 0) {
        steering = chrono_types::make_shared<RackPinion>(d);
    } else if (subtype.compare("RotaryArm") == 0) {
        steering = chrono_types::make_shared<RotaryArm>(d);
    } else {
        throw std::invalid_argument("Steering type not supported in ReadSteeringJSON.");
    }

    return steering;
}

std::shared_ptr<ChDrivelineWV> ReadDrivelineWVJSON(const std::string& filename) {
    std::shared_ptr<ChDrivelineWV> driveline;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a driveline specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Driveline") == 0);

    // Extract the driveline type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the driveline using the appropriate template.
    if (subtype.compare("ShaftsDriveline2WD") == 0) {
        driveline = chrono_types::make_shared<ShaftsDriveline2WD>(d);
    } else if (subtype.compare("ShaftsDriveline4WD") == 0) {
        driveline = chrono_types::make_shared<ShaftsDriveline4WD>(d);
    } else if (subtype.compare("SimpleDriveline") == 0) {
        driveline = chrono_types::make_shared<SimpleDriveline>(d);
    } else if (subtype.compare("SimpleDrivelineXWD") == 0) {
        driveline = chrono_types::make_shared<SimpleDrivelineXWD>(d);
    } else {
        throw std::invalid_argument("Driveline type not supported in ReadDrivelineWVJSON.");
    }

    return driveline;
}

std::shared_ptr<ChAntirollBar> ReadAntirollbarJSON(const std::string& filename) {
    std::shared_ptr<ChAntirollBar> antirollbar;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is an antirollbar specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Antirollbar") == 0);

    // Extract the antirollbar type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the antirollbar using the appropriate template.
    if (subtype.compare("AntirollBarRSD") == 0) {
        antirollbar = chrono_types::make_shared<AntirollBarRSD>(d);
    } else {
        throw std::invalid_argument("AntirollBar type not supported in ReadAntirollbarJSON.");
    }

    return antirollbar;
}

std::shared_ptr<ChWheel> ReadWheelJSON(const std::string& filename) {
    std::shared_ptr<ChWheel> wheel;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Wheel") == 0);

    // Extract the wheel type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the wheel using the appropriate template.
    if (subtype.compare("Wheel") == 0) {
        wheel = chrono_types::make_shared<Wheel>(d);
    } else {
        throw std::invalid_argument("Wheel type not supported in ReadWheelJSON.");
    }

    return wheel;
}

std::shared_ptr<ChSubchassis> ReadSubchassisJSON(const std::string& filename) {
    std::shared_ptr<ChSubchassis> chassis;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Subchassis") == 0);

    // Extract the wheel type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the wheel using the appropriate template.
    if (subtype.compare("Balancer") == 0) {
        chassis = chrono_types::make_shared<Balancer>(d);
    } else {
        throw std::invalid_argument("Subchassis type not supported in ReadSubchassisJSON.");
    }

    return chassis;
}

std::shared_ptr<ChBrake> ReadBrakeJSON(const std::string& filename) {
    std::shared_ptr<ChBrake> brake;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a brake specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Brake") == 0);

    // Extract the brake type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the brake using the appropriate template.
    if (subtype.compare("BrakeSimple") == 0) {
        brake = chrono_types::make_shared<BrakeSimple>(d);
    } else if (subtype.compare("BrakeShafts") == 0) {
        brake = chrono_types::make_shared<BrakeShafts>(d);
    } else {
        throw std::invalid_argument("Brake type not supported in ReadBrakeJSON.");
    }

    return brake;
}

std::shared_ptr<ChTire> ReadTireJSON(const std::string& filename) {
    std::shared_ptr<ChTire> tire;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a tire specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Tire") == 0);

    // Extract the tire type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the tire using the appropriate template.
    if (subtype.compare("RigidTire") == 0) {
        tire = chrono_types::make_shared<RigidTire>(d);
    } else if (subtype.compare("TMeasyTire") == 0) {
        tire = chrono_types::make_shared<TMeasyTire>(d);
    } else if (subtype.compare("TMsimpleTire") == 0) {
        tire = chrono_types::make_shared<TMsimpleTire>(d);
    } else if (subtype.compare("FialaTire") == 0) {
        tire = chrono_types::make_shared<FialaTire>(d);
    } else if (subtype.compare("Pac89Tire") == 0) {
        tire = chrono_types::make_shared<Pac89Tire>(d);
    } else if (subtype.compare("Pac02Tire") == 0) {
        tire = chrono_types::make_shared<Pac02Tire>(d);
    } else if (subtype.compare("ANCFTire") == 0) {
        tire = chrono_types::make_shared<ANCFTire>(d);
    } else if (subtype.compare("ReissnerTire") == 0) {
        tire = chrono_types::make_shared<ReissnerTire>(d);
    } else if (subtype.compare("FEATire") == 0) {
        tire = chrono_types::make_shared<FEATire>(d);
    } else {
        throw std::invalid_argument("Tire type not supported in ReadTireJSON.");
    }

    return tire;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChTrackAssembly> ReadTrackAssemblyJSON(const std::string& filename) {
    std::shared_ptr<ChTrackAssembly> track;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackAssembly") == 0);

    // Extract the track assembly type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("TrackAssemblySinglePin") == 0) {
        track = chrono_types::make_shared<TrackAssemblySinglePin>(d);
    } else if (subtype.compare("TrackAssemblyDoublePin") == 0) {
        track = chrono_types::make_shared<TrackAssemblyDoublePin>(d);
    } else if (subtype.compare("TrackAssemblyBandBushing") == 0) {
        track = chrono_types::make_shared<TrackAssemblyBandBushing>(d);
    } else if (subtype.compare("TrackAssemblyBandANCF") == 0) {
        track = chrono_types::make_shared<TrackAssemblyBandANCF>(d);
    } else {
        throw std::invalid_argument("TrackAssembly type not supported in ReadTrackAssemblyJSON.");
    }

    return track;
}

std::shared_ptr<ChDrivelineTV> ReadDrivelineTVJSON(const std::string& filename) {
    std::shared_ptr<ChDrivelineTV> driveline;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a driveline specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackDriveline") == 0);

    // Extract the driveline type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the driveline using the appropriate template.
    if (subtype.compare("SimpleTrackDriveline") == 0) {
        driveline = chrono_types::make_shared<SimpleTrackDriveline>(d);
    } else if (subtype.compare("TrackDrivelineBDS") == 0) {
        driveline = chrono_types::make_shared<TrackDrivelineBDS>(d);
    } else {
        throw std::invalid_argument("Driveline type not supported in ReadTrackDrivelineJSON.");
    }

    return driveline;
}

std::shared_ptr<ChTrackBrake> ReadTrackBrakeJSON(const std::string& filename) {
    std::shared_ptr<ChTrackBrake> brake;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a brake specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackBrake") == 0);

    // Extract brake type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the brake using the appropriate template.
    if (subtype.compare("TrackBrakeSimple") == 0) {
        brake = chrono_types::make_shared<TrackBrakeSimple>(d);
    } else if (subtype.compare("TrackBrakeShafts") == 0) {
        brake = chrono_types::make_shared<TrackBrakeShafts>(d);
    } else {
        throw std::invalid_argument("Brake type not supported in ReadTrackBrakeJSON.");
    }

    return brake;
}

std::shared_ptr<ChIdler> ReadIdlerJSON(const std::string& filename) {
    std::shared_ptr<ChIdler> idler;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is an idler specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Idler") == 0);

    // Extract idler type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the idler using the appropriate template.
    if (subtype.compare("TranslationalIdler") == 0) {
        idler = chrono_types::make_shared<TranslationalIdler>(d);
    } else if (subtype.compare("DistanceIdler") == 0) {
        idler = chrono_types::make_shared<DistanceIdler>(d);
    } else {
        throw std::invalid_argument("Idler type not supported in ReadIdlerJSON.");
    }

    return idler;
}

std::shared_ptr<ChTrackSuspension> ReadTrackSuspensionJSON(const std::string& filename, bool has_shock, bool lock_arm) {
    std::shared_ptr<ChTrackSuspension> suspension;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a track suspension specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackSuspension") == 0);

    // Extract track suspension type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the track suspension using the appropriate template.
    if (subtype.compare("TranslationalDamperSuspension") == 0) {
        suspension = chrono_types::make_shared<TranslationalDamperSuspension>(d, has_shock, lock_arm);
    } else if (subtype.compare("RotationalDamperSuspension") == 0) {
        suspension = chrono_types::make_shared<RotationalDamperSuspension>(d, has_shock, lock_arm);
    } else {
        throw std::invalid_argument("Suspension type not supported in ReadTrackSuspensionJSON.");
    }

    return suspension;
}

std::shared_ptr<ChTrackWheel> ReadTrackWheelJSON(const std::string& filename) {
    std::shared_ptr<ChTrackWheel> wheel;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a road-wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("TrackWheel") == 0);

    // Extract the road-wheel type
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the road-wheel using the appropriate template.
    if (subtype.compare("SingleTrackWheel") == 0) {
        wheel = chrono_types::make_shared<SingleTrackWheel>(d);
    } else if (subtype.compare("DoubleTrackWheel") == 0) {
        wheel = chrono_types::make_shared<DoubleTrackWheel>(d);
    } else {
        throw std::invalid_argument("Road-wheel type not supported in ReadTrackWheelJSON.");
    }

    return wheel;
}

}  // end namespace vehicle
}  // end namespace chrono
