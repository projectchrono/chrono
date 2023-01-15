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
#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
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

ChContactMaterialData ReadMaterialInfoJSON(const rapidjson::Value& mat) {
    ChContactMaterialData minfo;

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

std::shared_ptr<ChVehicleBushingData> ReadBushingDataJSON(const rapidjson::Value& bd) {
    auto bushing_data = chrono_types::make_shared<ChVehicleBushingData>();

    bushing_data->K_lin = bd["Stiffness Linear"].GetDouble();
    bushing_data->D_lin = bd["Damping Linear"].GetDouble();
    bushing_data->K_rot = bd["Stiffness Rotational"].GetDouble();
    bushing_data->D_rot = bd["Damping Rotational"].GetDouble();

    if (bd.HasMember("DOF")) {
        bushing_data->K_lin_dof = bd["DOF"]["Stiffness Linear"].GetDouble();
        bushing_data->D_lin_dof = bd["DOF"]["Damping Linear"].GetDouble();
        bushing_data->K_rot_dof = bd["DOF"]["Stiffness Rotational"].GetDouble();
        bushing_data->D_rot_dof = bd["DOF"]["Damping Rotational"].GetDouble();
    }

    return bushing_data;
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctorJSON(const rapidjson::Value& tsda, double& free_length) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        DegressiveDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        MapSpringDamper,
        Unknown
    };

    FunctorType type = FunctorType::Unknown;
    free_length = 0;

    assert(tsda.IsObject());

    if (tsda.HasMember("Spring Coefficient"))
        if (tsda.HasMember("Damping Coefficient"))
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    else if (tsda.HasMember("Damping Coefficient"))
        if (tsda.HasMember("Degressivity Compression") && tsda.HasMember("Degressivity Expansion"))
            type = FunctorType::DegressiveDamper;
        else
            type = FunctorType::LinearDamper;

    if (tsda.HasMember("Spring Curve Data"))
        if (tsda.HasMember("Damping Curve Data"))
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    else if (tsda.HasMember("Damping Curve Data"))
        type = FunctorType::NonlinearDamper;

    if (tsda.HasMember("Map Data"))
        type = FunctorType::MapSpringDamper;

    double preload = 0;
    if (tsda.HasMember("Preload"))
        preload = tsda["Preload"].GetDouble();

    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported TSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            double k = tsda["Spring Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearSpringForce>(k, preload);
        }

        case FunctorType::NonlinearSpring: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            auto forceCB = chrono_types::make_shared<NonlinearSpringForce>(preload);

            assert(tsda["Spring Curve Data"].IsArray() && tsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = tsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["Spring Curve Data"][i][0u].GetDouble();
                double force = tsda["Spring Curve Data"][i][1u].GetDouble();
                forceCB->add_pointK(def, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::LinearDamper: {
            double c = tsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearDamperForce>(c);
        }

        case FunctorType::DegressiveDamper: {
            double c = tsda["Damping Coefficient"].GetDouble();
            double dc = tsda["Degressivity Compression"].GetDouble();
            double de = tsda["Degressivity Expansion"].GetDouble();

            return chrono_types::make_shared<DegressiveDamperForce>(c, dc, de);
        }

        case FunctorType::NonlinearDamper: {
            auto forceCB = chrono_types::make_shared<NonlinearDamperForce>();

            assert(tsda["Damping Curve Data"].IsArray() && tsda["Damping Curve Data"][0u].Size() == 2);
            int num_speeds = tsda["Dumping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Damping Curve Data"][i][0u].GetDouble();
                double force = tsda["Damping Curve Data"][i][1u].GetDouble();
                forceCB->add_pointC(vel, force);
            }

            return forceCB;
        }

        case FunctorType::LinearSpringDamper: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            double k = tsda["Spring Coefficient"].GetDouble();
            double c = tsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearSpringDamperForce>(k, c, preload);
        }

        case FunctorType::NonlinearSpringDamper: {
            auto forceCB = chrono_types::make_shared<NonlinearSpringDamperForce>(preload);

            assert(tsda["Spring Curve Data"].IsArray() && tsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = tsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = tsda["Spring Curve Data"][i][0u].GetDouble();
                double force = tsda["Spring Curve Data"][i][1u].GetDouble();
                forceCB->add_pointK(def, force);
            }
            int num_speeds = tsda["Dumping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Damping Curve Data"][i][0u].GetDouble();
                double force = tsda["Damping Curve Data"][i][1u].GetDouble();
                forceCB->add_pointC(vel, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;
        }

        case FunctorType::MapSpringDamper: {
            assert(tsda.HasMember("Free Length"));
            free_length = tsda["Free Length"].GetDouble();

            auto forceCB = chrono_types::make_shared<MapSpringDamperForce>(preload);

            assert(tsda.HasMember("Deformation"));
            assert(tsda["Deformation"].IsArray());
            assert(tsda["Map Data"].IsArray() && tsda["Map Data"][0u].Size() == tsda["Deformation"].Size() + 1);
            int num_defs = tsda["Deformation"].Size();
            int num_speeds = tsda["Map Data"].Size();
            std::vector<double> defs(num_defs);
            for (int j = 0; j < num_defs; j++)
                defs[j] = tsda["Deformation"][j].GetDouble();
            forceCB->set_deformations(defs);
            for (int i = 0; i < num_speeds; i++) {
                double vel = tsda["Map Data"][i][0u].GetDouble();
                std::vector<double> force(num_defs);
                for (int j = 0; j < num_defs; j++)
                    force[j] = tsda["Map Data"][i][j + 1].GetDouble();
                forceCB->add_pointC(vel, force);
            }
            if (tsda.HasMember("Minimum Length") && tsda.HasMember("Maximum Length")) {
                forceCB->enable_stops(tsda["Minimum Length"].GetDouble(), tsda["Maximum Length"].GetDouble());
            }

            return forceCB;     
        }
    }
}

std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctorJSON(const rapidjson::Value& rsda, double& free_angle) {
    enum class FunctorType {
        LinearSpring,
        NonlinearSpring,
        LinearDamper,
        NonlinearDamper,
        LinearSpringDamper,
        NonlinearSpringDamper,
        Unknown
    };

    FunctorType type = FunctorType::Unknown;
    free_angle = 0;

    if (rsda.HasMember("Spring Coefficient"))
        if (rsda.HasMember("Damping Coefficient"))
            type = FunctorType::LinearSpringDamper;
        else
            type = FunctorType::LinearSpring;
    else if (rsda.HasMember("Damping Coefficient"))
        type = FunctorType::LinearDamper;

    if (rsda.HasMember("Spring Curve Data"))
        if (rsda.HasMember("Damping Curve Data"))
            type = FunctorType::NonlinearSpringDamper;
        else
            type = FunctorType::NonlinearSpring;
    else if (rsda.HasMember("Damping Curve Data"))
        type = FunctorType::NonlinearDamper;

    double preload = 0;
    if (rsda.HasMember("Preload"))
        preload = rsda["Preload"].GetDouble();

    switch (type) {
        default:
        case FunctorType::Unknown: {
            std::cout << "Unsupported RSDA element" << std::endl;
            return nullptr;
        }

        case FunctorType::LinearSpring: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            double k = rsda["Spring Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearSpringTorque>(k, preload);
        }

        case FunctorType::NonlinearSpring: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            auto torqueCB = chrono_types::make_shared<NonlinearSpringTorque>(preload);

            assert(rsda["Spring Curve Data"].IsArray() && rsda["Spring Curve Data"][0u].Size() == 2);
            int num_defs = rsda["Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double def = rsda["Spring Curve Data"][i][0u].GetDouble();
                double force = rsda["Spring Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointK(def, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearDamper: {
            double c = rsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearDamperTorque>(c);
        }

        case FunctorType::NonlinearDamper: {
            auto torqueCB = chrono_types::make_shared<NonlinearDamperTorque>();

            assert(rsda["Damping Curve Data"].IsArray() && rsda["Damping Curve Data"][0u].Size() == 2);
            int num_speeds = rsda["Dumping Curve Data"].Size();
            for (int i = 0; i < num_speeds; i++) {
                double vel = rsda["Damping Curve Data"][i][0u].GetDouble();
                double force = rsda["Damping Curve Data"][i][1u].GetDouble();
                torqueCB->add_pointC(vel, force);
            }

            return torqueCB;
        }

        case FunctorType::LinearSpringDamper: {
            assert(rsda.HasMember("Free Angle"));
            free_angle = rsda["Free Angle"].GetDouble();

            double k = rsda["Spring Coefficient"].GetDouble();
            double c = rsda["Damping Coefficient"].GetDouble();

            return chrono_types::make_shared<LinearSpringDamperTorque>(k, c, preload);
        }

    }
}

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
        throw ChException("Chassis type not supported in ReadChassisJSON.");
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
        throw ChException("Chassis type not supported in ReadChassisRearJSON.");
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
        throw ChException("Steering type not supported in ReadSteeringJSON.");
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
        throw ChException("Driveline type not supported in ReadDrivelineWVJSON.");
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
        throw ChException("AntirollBar type not supported in ReadAntirollbarJSON.");
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
        throw ChException("Wheel type not supported in ReadWheelJSON.");
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
        throw ChException("Subchassis type not supported in ReadSubchassisJSON.");
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
        throw ChException("Brake type not supported in ReadBrakeJSON.");
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
        throw ChException("Tire type not supported in ReadTireJSON.");
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
        throw ChException("TrackAssembly type not supported in ReadTrackAssemblyJSON.");
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
        throw ChException("Driveline type not supported in ReadTrackDrivelineJSON.");
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
        throw ChException("Brake type not supported in ReadTrackBrakeJSON.");
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
        throw ChException("Idler type not supported in ReadIdlerJSON.");
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
        throw ChException("Suspension type not supported in ReadTrackSuspensionJSON.");
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
        throw ChException("Road-wheel type not supported in ReadTrackWheelJSON.");
    }

    return wheel;
}

}  // end namespace vehicle
}  // end namespace chrono
