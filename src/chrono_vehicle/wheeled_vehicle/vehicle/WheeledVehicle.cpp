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
// Wheeled vehicle model constructed from a JSON specification file
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
//
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WheeledVehicle::WheeledVehicle(const std::string& filename, ChMaterialSurface::ContactMethod contact_method)
    : ChWheeledVehicle("", contact_method) {
    Create(filename);
}

WheeledVehicle::WheeledVehicle(ChSystem* system, const std::string& filename) : ChWheeledVehicle("", system) {
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

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string subtype = d["Template"].GetString();
    assert(type.compare("Vehicle") == 0);
    assert(subtype.compare("WheeledVehicle") == 0);

    SetName(name);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------

    assert(d.HasMember("Chassis"));
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

    {
        std::string file_name = d["Chassis"]["Input File"].GetString();
        m_chassis = ReadChassisJSON(vehicle::GetDataFile(file_name));
        if (d["Chassis"].HasMember("Output")) {
            m_chassis->SetOutput(d["Chassis"]["Output"].GetBool());
        }
    }

    // ------------------------------
    // Create the steering subsystems
    // ------------------------------

    for (int i = 0; i < m_num_strs; i++) {
        std::string file_name = d["Steering Subsystems"][i]["Input File"].GetString();
        m_steerings[i] = ReadSteeringJSON(vehicle::GetDataFile(file_name));
        if (d["Steering Subsystems"][i].HasMember("Output")) {
            m_steerings[i]->SetOutput(d["Steering Subsystems"][i]["Output"].GetBool());
        }
        m_strLocations[i] = ReadVectorJSON(d["Steering Subsystems"][i]["Location"]);
        m_strRotations[i] = ReadQuaternionJSON(d["Steering Subsystems"][i]["Orientation"]);
    }

    // --------------------
    // Create the driveline
    // --------------------

    {
        std::string file_name = d["Driveline"]["Input File"].GetString();
        m_driveline = ReadDrivelineJSON(vehicle::GetDataFile(file_name));
        if (d["Driveline"].HasMember("Output")) {
            m_driveline->SetOutput(d["Driveline"]["Output"].GetBool());
        }
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
        m_suspensions[i] = ReadSuspensionJSON(vehicle::GetDataFile(file_name));
        m_suspLocations[i] = ReadVectorJSON(d["Axles"][i]["Suspension Location"]);

        // Index of steering subsystem (if applicable)
        if (d["Axles"][i].HasMember("Steering Index")) {
            m_suspSteering[i] = d["Axles"][i]["Steering Index"].GetInt();
        }

        // Antirollbar (if applicable)
        if (d["Axles"][i].HasMember("Antirollbar Input File")) {
            assert(m_suspensions[i]->IsIndependent());
            assert(d["Axles"][i].HasMember("Antirollbar Location"));
            file_name = d["Axles"][i]["Antirollbar Input File"].GetString();
            m_antirollbars.push_back(ReadAntirollbarJSON(vehicle::GetDataFile(file_name)));
            m_arbLocations.push_back(ReadVectorJSON(d["Axles"][i]["Antirollbar Location"]));
            m_arbSuspension.push_back(i);
        }

        // Left and right wheels
        file_name = d["Axles"][i]["Left Wheel Input File"].GetString();
        m_wheels[2 * i + VehicleSide::LEFT] = ReadWheelJSON(vehicle::GetDataFile(file_name));

        file_name = d["Axles"][i]["Right Wheel Input File"].GetString();
        m_wheels[2 * i + VehicleSide::RIGHT] = ReadWheelJSON(vehicle::GetDataFile(file_name));

        // Left and right brakes
        file_name = d["Axles"][i]["Left Brake Input File"].GetString();
        m_brakes[2 * i + VehicleSide::LEFT] = ReadBrakeJSON(vehicle::GetDataFile(file_name));

        file_name = d["Axles"][i]["Right Brake Input File"].GetString();
        m_brakes[2 * i + VehicleSide::RIGHT] = ReadBrakeJSON(vehicle::GetDataFile(file_name));

        if (d["Axles"][i].HasMember("Output")) {
            bool output = d["Axles"][i]["Output"].GetBool();
            m_suspensions[i]->SetOutput(output);
            m_wheels[2 * i + VehicleSide::LEFT]->SetOutput(output);
            m_wheels[2 * i + VehicleSide::RIGHT]->SetOutput(output);
            m_brakes[2 * i + VehicleSide::LEFT]->SetOutput(output);
            m_brakes[2 * i + VehicleSide::RIGHT]->SetOutput(output);
        }
    }

    // Get the wheelbase (if defined in JSON file).
    // Otherwise, approximate as distance between first and last suspensions.
    if (d.HasMember("Wheelbase")) {
        m_wheelbase = d["Wheelbase"].GetDouble();
    } else {
        m_wheelbase = m_suspLocations[0].x() - m_suspLocations[m_num_axles - 1].x();
    }
    assert(m_wheelbase > 0);

    // Get the minimum turning radius (if defined in JSON file).
    // Otherwise, use default value.
    if (d.HasMember("Minimum Turning Radius")) {
        m_turn_radius = d["Minimum Turning Radius"].GetDouble();
    } else {
        m_turn_radius = ChWheeledVehicle::GetMinTurningRadius();
    }

    // Set maximum steering angle. Use value from JSON file is provided.
    // Otherwise, use default estimate.
    if (d.HasMember("Maximum Steering Angle")) {
        m_steer_angle = d["Maximum Steering Angle"].GetDouble() * CH_C_DEG_TO_RAD;
    } else {
        m_steer_angle = ChWheeledVehicle::GetMaxSteeringAngle();
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the steering subsystems.
    for (int i = 0; i < m_num_strs; i++) {
        m_steerings[i]->Initialize(m_chassis->GetBody(), m_strLocations[i], m_strRotations[i]);
    }

    // Initialize the suspension, wheel, and brake subsystems.
    for (int i = 0; i < m_num_axles; i++) {
        if (m_suspSteering[i] >= 0)
            m_suspensions[i]->Initialize(m_chassis->GetBody(), m_suspLocations[i],
                                         m_steerings[m_suspSteering[i]]->GetSteeringLink(), m_suspSteering[i]);
        else
            m_suspensions[i]->Initialize(m_chassis->GetBody(), m_suspLocations[i], m_chassis->GetBody(), -1);

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
