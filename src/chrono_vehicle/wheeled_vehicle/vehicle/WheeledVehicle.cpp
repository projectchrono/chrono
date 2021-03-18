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

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WheeledVehicle::WheeledVehicle(const std::string& filename, ChContactMethod contact_method)
    : ChWheeledVehicle("", contact_method) {
    Create(filename);
}

WheeledVehicle::WheeledVehicle(ChSystem* system, const std::string& filename) : ChWheeledVehicle("", system) {
    Create(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WheeledVehicle::Create(const std::string& filename) {
    // Open and parse the input file
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

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
    m_axles.resize(m_num_axles);
    m_suspLocations.resize(m_num_axles);
    m_suspSteering.resize(m_num_axles, -1);
    m_arbLocations.resize(m_num_axles);

    m_steerings.resize(m_num_strs);
    m_strLocations.resize(m_num_strs);
    m_strRotations.resize(m_num_strs);

    m_wheelSeparations.resize(m_num_axles, 0.0);

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
        m_driveline = ReadDrivelineWVJSON(vehicle::GetDataFile(file_name));
        if (d["Driveline"].HasMember("Output")) {
            m_driveline->SetOutput(d["Driveline"]["Output"].GetBool());
        }
        SizeType num_driven_axles = d["Driveline"]["Suspension Indexes"].Size();
        m_driven_axles.resize(num_driven_axles);
        for (SizeType i = 0; i < num_driven_axles; i++) {
            m_driven_axles[i] = d["Driveline"]["Suspension Indexes"][i].GetInt();
        }
    }

    // ---------------------------------------------------
    // Create the suspension, wheel, and brake subsystems.
    // ---------------------------------------------------

    for (int i = 0; i < m_num_axles; i++) {
        m_axles[i] = chrono_types::make_shared<ChAxle>();

        // Suspension
        std::string file_name = d["Axles"][i]["Suspension Input File"].GetString();
        m_axles[i]->m_suspension = ReadSuspensionJSON(vehicle::GetDataFile(file_name));
        m_suspLocations[i] = ReadVectorJSON(d["Axles"][i]["Suspension Location"]);

        // Index of steering subsystem (if applicable)
        if (d["Axles"][i].HasMember("Steering Index")) {
            m_suspSteering[i] = d["Axles"][i]["Steering Index"].GetInt();
        }

        // Antirollbar (if applicable)
        if (d["Axles"][i].HasMember("Antirollbar Input File")) {
            assert(m_axles[i]->m_suspension->IsIndependent());
            assert(d["Axles"][i].HasMember("Antirollbar Location"));
            file_name = d["Axles"][i]["Antirollbar Input File"].GetString();
            m_axles[i]->m_antirollbar = ReadAntirollbarJSON(vehicle::GetDataFile(file_name));
            m_arbLocations[i] = ReadVectorJSON(d["Axles"][i]["Antirollbar Location"]);
        }

        // Check if there are double wheels
        // Otherwise, assume only two
        if (d["Axles"][i].HasMember("Wheel Separation")) {
            m_wheelSeparations[i] = d["Axles"][i]["Wheel Separation"].GetDouble();

            int num_wheels = 4;
            m_axles[i]->m_wheels.resize(num_wheels);
            file_name = d["Axles"][i]["Left Inside Wheel Input File"].GetString();
            m_axles[i]->m_wheels[0] = ReadWheelJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Right Inside Wheel Input File"].GetString();
            m_axles[i]->m_wheels[1] = ReadWheelJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Left Outside Wheel Input File"].GetString();
            m_axles[i]->m_wheels[2] = ReadWheelJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Right Outside Wheel Input File"].GetString();
            m_axles[i]->m_wheels[3] = ReadWheelJSON(vehicle::GetDataFile(file_name));
        } else {
            int num_wheels = 2;
            m_axles[i]->m_wheels.resize(num_wheels);
            file_name = d["Axles"][i]["Left Wheel Input File"].GetString();
            m_axles[i]->m_wheels[0] = ReadWheelJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Right Wheel Input File"].GetString();
            m_axles[i]->m_wheels[1] = ReadWheelJSON(vehicle::GetDataFile(file_name));
        }

        // Left and right brakes (may be absent)
        if (d["Axles"][i].HasMember("Left Brake Input File")) {
            file_name = d["Axles"][i]["Left Brake Input File"].GetString();
            m_axles[i]->m_brake_left = ReadBrakeJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Right Brake Input File"].GetString();
            m_axles[i]->m_brake_right = ReadBrakeJSON(vehicle::GetDataFile(file_name));
        }

        if (d["Axles"][i].HasMember("Output")) {
            bool output = d["Axles"][i]["Output"].GetBool();
            m_axles[i]->SetOutput(output);
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
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystems.
    for (int i = 0; i < m_num_strs; i++) {
        m_steerings[i]->Initialize(m_chassis, m_strLocations[i], m_strRotations[i]);
    }

    // Initialize the axles (suspension + brakes + wheels + antirollbar)
    for (int i = 0; i < m_num_axles; i++) {
        int str_index = m_suspSteering[i];
        std::shared_ptr<ChSteering> steering = (str_index == -1) ? nullptr : m_steerings[str_index];
        m_axles[i]->Initialize(m_chassis, nullptr, steering, m_suspLocations[i], m_arbLocations[i],
                               m_wheelSeparations[i]);
    }

    // Initialize the driveline
    m_driveline->Initialize(m_chassis, m_axles, m_driven_axles);

    // Sanity check: make sure the driveline can accommodate the number of driven axles.
    assert(m_driveline->GetNumDrivenAxles() == m_driven_axles.size());
}

}  // end namespace vehicle
}  // end namespace chrono
