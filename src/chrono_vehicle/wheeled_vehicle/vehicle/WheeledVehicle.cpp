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

WheeledVehicle::WheeledVehicle(const std::string& filename,
                               ChContactMethod contact_method,
                               bool create_powertrain,
                               bool create_tires)
    : ChWheeledVehicle("", contact_method) {
    Create(filename, create_powertrain, create_tires);
}

WheeledVehicle::WheeledVehicle(ChSystem* system, const std::string& filename, bool create_powertrain, bool create_tires)
    : ChWheeledVehicle("", system) {
    Create(filename, create_powertrain, create_tires);
}

// -----------------------------------------------------------------------------

void WheeledVehicle::Create(const std::string& filename, bool create_powertrain, bool create_tires) {
    // Open and parse the input file
    Document d;
    ReadFileJSON(filename, d);
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
    assert(d.HasMember("Axles"));
    assert(d["Axles"].IsArray());

    // Extract number of rear chassis subsystems
    if (d.HasMember("Rear Chassis")) {
        assert(d["Rear Chassis"].IsArray());
        m_num_rear_chassis = d["Rear Chassis"].Size();
    } else {
        m_num_rear_chassis = 0;
    }

    // Extract number of subchassis subsystems
    if (d.HasMember("Subchassis")) {
        assert(d["Subchassis"].IsArray());
        m_num_subch = d["Subchassis"].Size();
    } else {
        m_num_subch = 0;
    }

    // Extract the number of axles.
    m_num_axles = d["Axles"].Size();

    // Extract the number of steering subsystems
    if (d.HasMember("Steering Subsystems")) {
        assert(d["Steering Subsystems"].IsArray());
        m_num_strs = d["Steering Subsystems"].Size();
    } else {
        m_num_strs = 0;
    }

    // Resize arrays
    if (m_num_rear_chassis > 0) {
        m_chassis_rear.resize(m_num_rear_chassis);
        m_chassis_connectors.resize(m_num_rear_chassis);
        m_rearch_chassis_index.resize(m_num_rear_chassis);
    }

    if (m_num_subch > 0) {
        m_subchassis.resize(m_num_subch);
        m_subch_locations.resize(m_num_subch);
        m_subch_chassis_index.resize(m_num_subch);
    }

    m_axles.resize(m_num_axles);
    m_arb_locations.resize(m_num_axles);
    m_susp_locations.resize(m_num_axles);
    m_susp_steering_index.resize(m_num_axles, -1);    // default: non-steerbale
    m_susp_chassis_index.resize(m_num_axles, -1);     // default: attached to main chassis
    m_susp_subchassis_index.resize(m_num_axles, -1);  // default: no subchassis attachment

    if (m_num_strs > 0) {
        m_steerings.resize(m_num_strs);
        m_str_locations.resize(m_num_strs);
        m_str_rotations.resize(m_num_strs);
        m_str_chassis_index.resize(m_num_strs, -1);  // default: attach to main chassis
    }

    m_wheel_separations.resize(m_num_axles, 0.0);

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

    // ---------------------------------------------------------
    // Create rear chassis and corresponding connectors (if any)
    // ---------------------------------------------------------

    for (int i = 0; i < m_num_rear_chassis; i++) {
        std::string file_name = d["Rear Chassis"][i]["Input File"].GetString();
        m_chassis_rear[i] = ReadChassisRearJSON(vehicle::GetDataFile(file_name));
        if (d["Rear Chassis"][i].HasMember("Output")) {
            m_chassis->SetOutput(d["Rear Chassis"][i]["Output"].GetBool());
        }
        file_name = d["Rear Chassis"][i]["Connector Input File"].GetString();
        m_chassis_connectors[i] = ReadChassisConnectorJSON(vehicle::GetDataFile(file_name));
        m_rearch_chassis_index[i] = d["Rear Chassis"][i]["Chassis Index"].GetInt();
    }

    // -------------------------------------
    // Create subchassis subsystems (if any)
    // -------------------------------------

    for (int i = 0; i < m_num_subch; i++) {
        std::string file_name = d["Subchassis"][i]["Input File"].GetString();
        m_subchassis[i] = ReadSubchassisJSON(vehicle::GetDataFile(file_name));
        m_subch_locations[i] = ReadVectorJSON(d["Subchassis"][i]["Subchassis Location"]);
        m_subch_chassis_index[i] = d["Subchassis"][i]["Chassis Index"].GetInt();
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
        m_str_locations[i] = ReadVectorJSON(d["Steering Subsystems"][i]["Location"]);
        m_str_rotations[i] = ReadQuaternionJSON(d["Steering Subsystems"][i]["Orientation"]);

        // Index of chassis subsystem (if applicable)
        if (d["Steering Subsystems"][i].HasMember("Chassis Index")) {
            m_str_chassis_index[i] = d["Steering Subsystems"][i]["Chassis Index"].GetInt();
        }
    }

    // ------------------------------------
    // Create the powertrain (if specified)
    // ------------------------------------

    if (create_powertrain && d.HasMember("Powertrain")) {
        assert(d["Powertrain"].HasMember("Engine Input File"));
        assert(d["Powertrain"].HasMember("Transmission Input File"));

        std::string file_name_e = d["Powertrain"]["Engine Input File"].GetString();
        std::string file_name_t = d["Powertrain"]["Transmission Input File"].GetString();
        auto engine = ReadEngineJSON(vehicle::GetDataFile(file_name_e));
        auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(file_name_t));
        m_powertrain_assembly = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    }

    // --------------------
    // Create the driveline
    // --------------------

    if (d.HasMember("Driveline")) {
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
    // Create tires (if specified)
    // ---------------------------------------------------

    for (int i = 0; i < m_num_axles; i++) {
        m_axles[i] = chrono_types::make_shared<ChAxle>();

        // Suspension
        std::string file_name = d["Axles"][i]["Suspension Input File"].GetString();
        m_axles[i]->m_suspension = ReadSuspensionJSON(vehicle::GetDataFile(file_name));
        m_susp_locations[i] = ReadVectorJSON(d["Axles"][i]["Suspension Location"]);

        // Index of steering subsystem (if applicable)
        if (d["Axles"][i].HasMember("Steering Index")) {
            m_susp_steering_index[i] = d["Axles"][i]["Steering Index"].GetInt();
        }

        // Index of chassis subsystem (if applicable)
        if (d["Axles"][i].HasMember("Chassis Index")) {
            m_susp_chassis_index[i] = d["Axles"][i]["Chassis Index"].GetInt();
        }

        // Index of subchassis subsystem (if applicable)
        if (d["Axles"][i].HasMember("Subchassis Index")) {
            m_susp_subchassis_index[i] = d["Axles"][i]["Subchassis Index"].GetInt();
        }

        // Antirollbar (if applicable)
        if (d["Axles"][i].HasMember("Antirollbar Input File")) {
            assert(m_axles[i]->m_suspension->IsIndependent());
            assert(d["Axles"][i].HasMember("Antirollbar Location"));
            file_name = d["Axles"][i]["Antirollbar Input File"].GetString();
            m_axles[i]->m_antirollbar = ReadAntirollbarJSON(vehicle::GetDataFile(file_name));
            m_arb_locations[i] = ReadVectorJSON(d["Axles"][i]["Antirollbar Location"]);
        }

        // Check if there are double wheels
        // Otherwise, assume only two
        if (d["Axles"][i].HasMember("Wheel Separation")) {
            m_wheel_separations[i] = d["Axles"][i]["Wheel Separation"].GetDouble();

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

        // Left and right brakes (if specified)
        if (d["Axles"][i].HasMember("Left Brake Input File")) {
            file_name = d["Axles"][i]["Left Brake Input File"].GetString();
            m_axles[i]->m_brake_left = ReadBrakeJSON(vehicle::GetDataFile(file_name));

            file_name = d["Axles"][i]["Right Brake Input File"].GetString();
            m_axles[i]->m_brake_right = ReadBrakeJSON(vehicle::GetDataFile(file_name));
        }

        // Create tires (if specified)
        if (create_tires && d["Axles"][i].HasMember("Tire Input File")) {
            file_name = d["Axles"][i]["Tire Input File"].GetString();
            for (auto& wheel : m_axles[i]->GetWheels()) {
                wheel->SetTire(ReadTireJSON(vehicle::GetDataFile(file_name)));
            }
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
        m_wheelbase = m_susp_locations[0].x() - m_susp_locations[m_num_axles - 1].x();
    }
    assert(m_wheelbase >= 0);

    // Get the minimum turning radius (if defined in JSON file).
    // Otherwise, use default value.
    if (d.HasMember("Minimum Turning Radius")) {
        m_turn_radius = d["Minimum Turning Radius"].GetDouble();
    } else {
        m_turn_radius = ChWheeledVehicle::GetMinTurningRadius();
    }

    // Set maximum steering angle. Use value from JSON file is provided.
    // Otherwise, use default estimate.
    if (d.HasMember("Maximum Steering Angle (deg)")) {
        m_steer_angle = d["Maximum Steering Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    } else {
        m_steer_angle = ChWheeledVehicle::GetMaxSteeringAngle();
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------

void WheeledVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize any rear chassis subsystems and their connectors.
    for (int i = 0; i < m_num_rear_chassis; i++) {
        int front_index = m_rearch_chassis_index[i];
        std::shared_ptr<ChChassis> front = (front_index == -1) ? m_chassis : m_chassis_rear[front_index];
        m_chassis_rear[i]->Initialize(front, WheeledCollisionFamily::CHASSIS);
        m_chassis_connectors[i]->Initialize(front, m_chassis_rear[i]);
    }

    // Initialize any subchassis subsystems.
    for (int i = 0; i < m_num_subch; i++) {
        int chassis_index = m_subch_chassis_index[i];
        std::shared_ptr<ChChassis> chassis = (chassis_index == -1) ? m_chassis : m_chassis_rear[chassis_index];
        m_subchassis[i]->Initialize(chassis, m_subch_locations[i]);
    }

    // Initialize the steering subsystems.
    for (int i = 0; i < m_num_strs; i++) {
        int ch_index = m_str_chassis_index[i];
        std::shared_ptr<ChChassis> chassis = (ch_index == -1) ? m_chassis : m_chassis_rear[ch_index];
        m_steerings[i]->Initialize(chassis, m_str_locations[i], m_str_rotations[i]);
    }

    // Initialize the axles (suspension + brakes + wheels + antirollbar)
    for (int i = 0; i < m_num_axles; i++) {
        int ch_index = m_susp_chassis_index[i];
        int subch_index = m_susp_subchassis_index[i];
        int str_index = m_susp_steering_index[i];
        std::shared_ptr<ChChassis> chassis = (ch_index == -1) ? m_chassis : m_chassis_rear[ch_index];
        std::shared_ptr<ChSubchassis> subchassis = (subch_index == -1) ? nullptr : m_subchassis[subch_index];
        std::shared_ptr<ChSteering> steering = (str_index == -1) ? nullptr : m_steerings[str_index];
        m_axles[i]->Initialize(chassis, subchassis, steering, m_susp_locations[i], m_arb_locations[i],
                               m_wheel_separations[i]);
        // Initialize tires (if present)
        for (auto& wheel : m_axles[i]->GetWheels()) {
            if (wheel->GetTire()) {
                InitializeTire(wheel->GetTire(), wheel);
            }
        }
    }

    // Initialize the driveline
    if (m_driveline) {
        m_driveline->Initialize(m_chassis, m_axles, m_driven_axles);

        // Sanity check: make sure the driveline can accommodate the number of driven axles.
        assert(m_driveline->GetNumDrivenAxles() == m_driven_axles.size());
    }

    // Initialize the powertain (if present)
    if (m_powertrain_assembly) {
        InitializePowertrain(m_powertrain_assembly);
    }

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

}  // end namespace vehicle
}  // end namespace chrono
