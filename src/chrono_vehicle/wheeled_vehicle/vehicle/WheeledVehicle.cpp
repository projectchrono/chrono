// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Wheeled vehicle model constructed from a JSON specification file
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/chassis/RigidChassis.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SemiTrailingArm.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS.h"

#include "chrono_vehicle/wheeled_vehicle/antirollbar/AntirollBarRSD.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/SimpleDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadChassis(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a chassis specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Chassis") == 0);

    // Extract the chassis type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("RigidChassis") == 0) {
        m_chassis = std::make_shared<RigidChassis>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadSteering(const std::string& filename, int which) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Steering") == 0);

    // Extract the steering type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    if (subtype.compare("PitmanArm") == 0) {
        m_steerings[which] = std::make_shared<PitmanArm>(d);
    } else if (subtype.compare("RackPinion") == 0) {
        m_steerings[which] = std::make_shared<RackPinion>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadDriveline(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a driveline specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Driveline") == 0);

    // Extract the driveline type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the driveline using the appropriate template.
    if (subtype.compare("ShaftsDriveline2WD") == 0) {
        m_driveline = std::make_shared<ShaftsDriveline2WD>(d);
    } else if (subtype.compare("ShaftsDriveline4WD") == 0) {
        m_driveline = std::make_shared<ShaftsDriveline4WD>(d);
    } else if (subtype.compare("SimpleDriveline") == 0) {
        m_driveline = std::make_shared<SimpleDriveline>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadSuspension(const std::string& filename, int axle) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a suspension specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Suspension") == 0);

    // Extract the suspension type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the suspension using the appropriate template.
    if (subtype.compare("DoubleWishbone") == 0) {
        m_suspensions[axle] = std::make_shared<DoubleWishbone>(d);
    } else if (subtype.compare("DoubleWishboneReduced") == 0) {
        m_suspensions[axle] = std::make_shared<DoubleWishboneReduced>(d);
    } else if (subtype.compare("SolidAxle") == 0) {
        m_suspensions[axle] = std::make_shared<SolidAxle>(d);
    } else if (subtype.compare("MultiLink") == 0) {
        m_suspensions[axle] = std::make_shared<MultiLink>(d);
    } else if (subtype.compare("MacPhersonStrut") == 0) {
        m_suspensions[axle] = std::make_shared<MacPhersonStrut>(d);
    } else if (subtype.compare("SemiTrailingArm") == 0) {
        m_suspensions[axle] = std::make_shared<SemiTrailingArm>(d);
    } else if (subtype.compare("ThreeLinkIRS") == 0) {
        m_suspensions[axle] = std::make_shared<ThreeLinkIRS>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadAntirollbar(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is an antirollbar specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Antirollbar") == 0);

    // Extract the antirollbar type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    if (subtype.compare("AntirollBarRSD") == 0) {
        m_antirollbars.push_back(std::make_shared<AntirollBarRSD>(d));
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadWheel(const std::string& filename, int axle, int side) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Wheel") == 0);

    // Extract the wheel type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the wheel using the appropriate template.
    if (subtype.compare("Wheel") == 0) {
        m_wheels[2 * axle + side] = std::make_shared<Wheel>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::LoadBrake(const std::string& filename, int axle, int side) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a brake specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Brake") == 0);

    // Extract the brake type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the brake using the appropriate template.
    if (subtype.compare("BrakeSimple") == 0) {
        m_brakes[2 * axle + side] = std::make_shared<BrakeSimple>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WheeledVehicle::WheeledVehicle(const std::string& filename, ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChWheeledVehicle(contact_method) {
    Create(filename);
}

WheeledVehicle::WheeledVehicle(ChSystem* system, const std::string& filename) : ChWheeledVehicle(system) {
    Create(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::Create(const std::string& filename) {
    // -------------------------------------------
    // Open and parse the input file
    // -------------------------------------------
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------

    assert(d.HasMember("Steering Subsystems"));
    assert(d.HasMember("Driveline"));
    assert(d.HasMember("Axles"));
    assert(d["Axles"].IsArray());
    assert(d["Steering Subsystems"].IsArray());

    // Extract the number of axles.
    m_num_axles = d["Axles"].Size();

    // Extract the number of steering subsystems
    m_num_strs = d["Steering Subsystems"].Size();

    // Resize arrays
    m_suspensions.resize(m_num_axles);
    m_suspLocations.resize(m_num_axles);
    m_suspSteering.resize(m_num_axles, -1);
    m_wheels.resize(2 * m_num_axles);
    m_brakes.resize(2 * m_num_axles);

    m_steerings.resize(m_num_strs);
    m_strLocations.resize(m_num_strs);
    m_strRotations.resize(m_num_strs);

    // -------------------------------------------
    // Create the chassis system
    // -------------------------------------------
    assert(d.HasMember("Chassis"));

    {
        std::string file_name = d["Chassis"]["Input File"].GetString();
        LoadChassis(vehicle::GetDataFile(file_name));
    }

    // ------------------------------
    // Create the steering subsystems
    // ------------------------------

    for (int i = 0; i < m_num_strs; i++) {
        std::string file_name = d["Steering Subsystems"][i]["Input File"].GetString();
        LoadSteering(vehicle::GetDataFile(file_name), i);
        m_strLocations[i] = loadVector(d["Steering Subsystems"][i]["Location"]);
        m_strRotations[i] = loadQuaternion(d["Steering Subsystems"][i]["Orientation"]);
    }

    // --------------------
    // Create the driveline
    // --------------------

    {
        std::string file_name = d["Driveline"]["Input File"].GetString();
        LoadDriveline(vehicle::GetDataFile(file_name));
        SizeType num_driven_susp = d["Driveline"]["Suspension Indexes"].Size();
        m_driven_susp.resize(num_driven_susp);
        for (SizeType i = 0; i < num_driven_susp; i++) {
            m_driven_susp[i] = d["Driveline"]["Suspension Indexes"][i].GetInt();
        }

        assert(num_driven_susp == GetDriveline()->GetNumDrivenAxles());
    }

    // ---------------------------------------------------
    // Create the suspension, wheel, and brake subsystems.
    // ---------------------------------------------------

    for (int i = 0; i < m_num_axles; i++) {
        // Suspension
        std::string file_name = d["Axles"][i]["Suspension Input File"].GetString();
        LoadSuspension(vehicle::GetDataFile(file_name), i);
        m_suspLocations[i] = loadVector(d["Axles"][i]["Suspension Location"]);

        // Index of steering subsystem (if applicable)
        if (d["Axles"][i].HasMember("Steering Index")) {
            m_suspSteering[i] = d["Axles"][i]["Steering Index"].GetInt();
        }

        // Antirollbar (if applicable)
        if (d["Axles"][i].HasMember("Antirollbar Input File")) {
            assert(m_suspensions[i]->IsIndependent());
            assert(d["Axles"][i].HasMember("Antirollbar Location"));
            file_name = d["Axles"][i]["Antirollbar Input File"].GetString();
            LoadAntirollbar(vehicle::GetDataFile(file_name));
            m_arbLocations.push_back(loadVector(d["Axles"][i]["Antirollbar Location"]));
            m_arbSuspension.push_back(i);
        }

        // Left and right wheels
        file_name = d["Axles"][i]["Left Wheel Input File"].GetString();
        LoadWheel(vehicle::GetDataFile(file_name), i, 0);
        file_name = d["Axles"][i]["Right Wheel Input File"].GetString();
        LoadWheel(vehicle::GetDataFile(file_name), i, 1);

        // Left and right brakes
        file_name = d["Axles"][i]["Left Brake Input File"].GetString();
        LoadBrake(vehicle::GetDataFile(file_name), i, 0);

        file_name = d["Axles"][i]["Right Brake Input File"].GetString();
        LoadBrake(vehicle::GetDataFile(file_name), i, 1);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel);

    // Initialize the steering subsystems.
    for (int i = 0; i < m_num_strs; i++) {
        m_steerings[i]->Initialize(m_chassis->GetBody(), m_strLocations[i], m_strRotations[i]);
    }

    // Initialize the suspension, wheel, and brake subsystems.
    for (int i = 0; i < m_num_axles; i++) {
        if (m_suspSteering[i] >= 0)
            m_suspensions[i]->Initialize(m_chassis->GetBody(), m_suspLocations[i],
                                         m_steerings[m_suspSteering[i]]->GetSteeringLink());
        else
            m_suspensions[i]->Initialize(m_chassis->GetBody(), m_suspLocations[i], m_chassis->GetBody());

        m_wheels[2 * i]->Initialize(m_suspensions[i]->GetSpindle(LEFT));
        m_wheels[2 * i + 1]->Initialize(m_suspensions[i]->GetSpindle(RIGHT));

        m_brakes[2 * i]->Initialize(m_suspensions[i]->GetRevolute(LEFT));
        m_brakes[2 * i + 1]->Initialize(m_suspensions[i]->GetRevolute(RIGHT));
    }

    // Initialize the antirollbar subsystems.
    for (unsigned int i = 0; i < m_antirollbars.size(); i++) {
        int j = m_arbSuspension[i];
        m_antirollbars[i]->Initialize(m_chassis->GetBody(), m_arbLocations[i], m_suspensions[j]->GetLeftBody(),
                                      m_suspensions[j]->GetRightBody());
    }

    // Initialize the driveline
    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, m_driven_susp);
}

}  // end namespace vehicle
}  // end namespace chrono
