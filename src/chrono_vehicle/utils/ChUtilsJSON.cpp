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

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/chassis/RigidChassis.h"
#include "chrono_vehicle/chassis/ChassisConnectorHitch.h"
#include "chrono_vehicle/chassis/ChassisConnectorArticulated.h"
#include "chrono_vehicle/chassis/ChassisConnectorTorsion.h"

#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"

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
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidThreeLinkAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/PacejkaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeSimple.h"
#include "chrono_vehicle/tracked_vehicle/brake/TrackBrakeShafts.h"
#include "chrono_vehicle/tracked_vehicle/driveline/SimpleTrackDriveline.h"
#include "chrono_vehicle/tracked_vehicle/driveline/TrackDrivelineBDS.h"
#include "chrono_vehicle/tracked_vehicle/idler/DoubleIdler.h"
#include "chrono_vehicle/tracked_vehicle/idler/SingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/DoubleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/SingleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/roller/DoubleRoller.h"
#include "chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"
#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperRWAssembly.h"
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

void ReadFileJSON(const std::string& filename, Document& d) {
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        GetLog() << "ERROR: Could not open JSON file: " << filename << "\n";
    } else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            GetLog() << "ERROR: Invalid JSON file: " << filename << "\n";
        }
    }
}

// -----------------------------------------------------------------------------

ChVector<> ReadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

ChQuaternion<> ReadQuaternionJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

ChColor ReadColorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChColor(a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat());
}

// -----------------------------------------------------------------------------

MaterialInfo ReadMaterialInfoJSON(const rapidjson::Value& mat) {
    MaterialInfo minfo;

    minfo.mu = mat["Coefficient of Friction"].GetFloat();
    minfo.cr = mat["Coefficient of Restitution"].GetFloat();
    if (mat.HasMember("Properties")) {
        minfo.Y = mat["Properties"]["Young Modulus"].GetFloat();
        minfo.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
    }
    if (mat.HasMember("Coefficients")) {
        minfo.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
        minfo.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
        minfo.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
        minfo.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
    }

    return minfo;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChChassis> ReadChassisJSON(const std::string& filename) {
    std::shared_ptr<ChChassis> chassis;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Chassis type not supported in ReadChassisJSON.");
    }

    return chassis;
}

std::shared_ptr<ChChassisRear> ReadChassisRearJSON(const std::string& filename) {
    std::shared_ptr<ChChassisRear> chassis;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Chassis type not supported in ReadChassisRearJSON.");
    }

    return chassis;
}

std::shared_ptr<ChChassisConnector> ReadChassisConnectorJSON(const std::string& filename) {
    std::shared_ptr<ChChassisConnector> connector;

    Document d;ReadFileJSON(filename, d);
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
    } else {
        throw ChException("Chassis type not supported in ReadChassisConnectorJSON.");
    }

    return connector;
}

std::shared_ptr<ChPowertrain> ReadPowertrainJSON(const std::string& filename) {
    std::shared_ptr<ChPowertrain> powertrain;

    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a powertrain specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Powertrain") == 0);

    // Extract the powertrain type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the powertrain using the appropriate template.
    if (subtype.compare("ShaftsPowertrain") == 0) {
        powertrain = chrono_types::make_shared<ShaftsPowertrain>(d);
    } else if (subtype.compare("SimpleCVTPowertrain") == 0) {
        powertrain = chrono_types::make_shared<SimpleCVTPowertrain>(d);
    } else if (subtype.compare("SimpleMapPowertrain") == 0) {
        powertrain = chrono_types::make_shared<SimpleMapPowertrain>(d);
    } else if (subtype.compare("SimplePowertrain") == 0) {
        powertrain = chrono_types::make_shared<SimplePowertrain>(d);
    } else {
        throw ChException("Powertrain type not supported in ReadChassisJSON.");
    }

    return powertrain;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChSuspension> ReadSuspensionJSON(const std::string& filename) {
    std::shared_ptr<ChSuspension> suspension;

    Document d;ReadFileJSON(filename, d);
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
    } else if (subtype.compare("SolidThreeLinkAxle") == 0) {
        suspension = chrono_types::make_shared<SolidThreeLinkAxle>(d);
    } else if (subtype.compare("SolidBellcrankThreeLinkAxle") == 0) {
        suspension = chrono_types::make_shared<SolidBellcrankThreeLinkAxle>(d);
    } else if (subtype.compare("RigidSuspension") == 0) {
        suspension = chrono_types::make_shared<RigidSuspension>(d);
    } else if (subtype.compare("RigidPinnedAxle") == 0) {
        suspension = chrono_types::make_shared<RigidPinnedAxle>(d);
    } else if (subtype.compare("HendricksonPRIMAXX") == 0) {
        suspension = chrono_types::make_shared<HendricksonPRIMAXX>(d);
    } else {
        throw ChException("Suspension type not supported in ReadSuspensionJSON.");
    }

    return suspension;
}

std::shared_ptr<ChSteering> ReadSteeringJSON(const std::string& filename) {
    std::shared_ptr<ChSteering> steering;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Steering type not supported in ReadSteeringJSON.");
    }

    return steering;
}

std::shared_ptr<ChDrivelineWV> ReadDrivelineWVJSON(const std::string& filename) {
    std::shared_ptr<ChDrivelineWV> driveline;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Driveline type not supported in ReadDrivelineWVJSON.");
    }

    return driveline;
}

std::shared_ptr<ChAntirollBar> ReadAntirollbarJSON(const std::string& filename) {
    std::shared_ptr<ChAntirollBar> antirollbar;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("AntirollBar type not supported in ReadAntirollbarJSON.");
    }

    return antirollbar;
}

std::shared_ptr<ChWheel> ReadWheelJSON(const std::string& filename) {
    std::shared_ptr<ChWheel> wheel;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Wheel type not supported in ReadWheelJSON.");
    }

    return wheel;
}

std::shared_ptr<ChBrake> ReadBrakeJSON(const std::string& filename) {
    std::shared_ptr<ChBrake> brake;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Brake type not supported in ReadBrakeJSON.");
    }

    return brake;
}

std::shared_ptr<ChTire> ReadTireJSON(const std::string& filename) {
    std::shared_ptr<ChTire> tire;

    Document d;ReadFileJSON(filename, d);
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
    } else if (subtype.compare("FialaTire") == 0) {
        tire = chrono_types::make_shared<FialaTire>(d);
    } else if (subtype.compare("LugreTire") == 0) {
        tire = chrono_types::make_shared<LugreTire>(d);
    } else if (subtype.compare("PacejkaTire") == 0) {
        tire = chrono_types::make_shared<PacejkaTire>(d);
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
        throw ChException("Tire type not supported in ReadTireJSON.");
    }

    return tire;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChTrackAssembly> ReadTrackAssemblySON(const std::string& filename) {
    std::shared_ptr<ChTrackAssembly> track;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("TrackAssembly type not supported in ReadTrackAssemblySON.");
    }

    return track;
}

std::shared_ptr<ChDrivelineTV> ReadDrivelineTVJSON(const std::string& filename) {
    std::shared_ptr<ChDrivelineTV> driveline;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Driveline type not supported in ReadTrackDrivelineJSON.");
    }

    return driveline;
}

std::shared_ptr<ChTrackBrake> ReadTrackBrakeJSON(const std::string& filename) {
    std::shared_ptr<ChTrackBrake> brake;

    Document d;ReadFileJSON(filename, d);
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
        throw ChException("Brake type not supported in ReadTrackBrakeJSON.");
    }

    return brake;
}

std::shared_ptr<ChIdler> ReadIdlerJSON(const std::string& filename) {
    std::shared_ptr<ChIdler> idler;

    Document d;ReadFileJSON(filename, d);
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
    if (subtype.compare("SingleIdler") == 0) {
        idler = chrono_types::make_shared<SingleIdler>(d);
    } else if (subtype.compare("DoubleIdler") == 0) {
        idler = chrono_types::make_shared<DoubleIdler>(d);
    } else {
        throw ChException("Idler type not supported in ReadIdlerJSON.");
    }

    return idler;
}

std::shared_ptr<ChRoadWheelAssembly> ReadRoadWheelAssemblyJSON(const std::string& filename, bool has_shock) {
    std::shared_ptr<ChRoadWheelAssembly> suspension;

    Document d;ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a road-wheel assembly specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("RoadWheelAssembly") == 0);

    // Extract road-wheel assembly type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the road-wheel assembly using the appropriate template.
    if (subtype.compare("LinearDamperRWAssembly") == 0) {
        suspension = chrono_types::make_shared<LinearDamperRWAssembly>(d, has_shock);
    } else if (subtype.compare("RotationalDamperRWAssembly") == 0) {
        suspension = chrono_types::make_shared<RotationalDamperRWAssembly>(d, has_shock);
    } else {
        throw ChException("Suspension type not supported in ReadRoadWheelAssemblyJSON.");
    }

    return suspension;
}

std::shared_ptr<ChRoller> ReadRollerJSON(const std::string& filename) {
    std::shared_ptr<ChRoller> roller;

    Document d;ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a roller specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Roller") == 0);

    // Extract roller type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the roller using the appropriate template.
    if (subtype.compare("DoubleRoller") == 0) {
        roller = chrono_types::make_shared<DoubleRoller>(d);
    } else {
        throw ChException("Roller type not supported in ReadRollerJSON.");
    }

    return roller;
}

std::shared_ptr<ChRoadWheel> ReadRoadWheelJSON(const std::string& filename) {
    std::shared_ptr<ChRoadWheel> wheel;

    Document d;ReadFileJSON(filename, d);
    if (d.IsNull())
        return nullptr;

    // Check that the given file is a road-wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("RoadWheel") == 0);

    // Extract the road-wheel type
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the road-wheel using the appropriate template.
    if (subtype.compare("SingleRoadWheel") == 0) {
        wheel = chrono_types::make_shared<SingleRoadWheel>(d);
    } else if (subtype.compare("DoubleRoadWheel") == 0) {
        wheel = chrono_types::make_shared<DoubleRoadWheel>(d);
    } else {
        throw ChException("Road-wheel type not supported in ReadRoadWheelJSON.");
    }

    return wheel;
}

}  // end namespace vehicle
}  // end namespace chrono
