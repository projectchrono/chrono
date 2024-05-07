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

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

WheeledTrailer::WheeledTrailer(ChSystem* system, const std::string& filename, bool create_tires)
    : ChWheeledTrailer("", system) {
    Create(filename, create_tires);
}

void WheeledTrailer::Create(const std::string& filename, bool create_tires) {
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
    assert(type.compare("Trailer") == 0);
    assert(subtype.compare("WheeledTrailer") == 0);

    SetName(name);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------

    assert(d.HasMember("Chassis"));
    assert(d.HasMember("Axles"));
    assert(d["Axles"].IsArray());

    // Extract the number of axles.
    m_num_axles = d["Axles"].Size();

    // Resize arrays
    m_axles.resize(m_num_axles);
    m_suspLocations.resize(m_num_axles);

    m_wheelSeparations.resize(m_num_axles, 0.0);

    // -----------------------------------------------
    // Create the (rear) chassis and connector systems
    // -----------------------------------------------

    {
        std::string file_name = d["Chassis"]["Input File"].GetString();
        m_chassis = ReadChassisRearJSON(vehicle::GetDataFile(file_name));
        if (d["Chassis"].HasMember("Output")) {
            m_chassis->SetOutput(d["Chassis"]["Output"].GetBool());
        }
    }

    {
        std::string file_name = d["Connector"]["Input File"].GetString();
        auto connector = ReadChassisConnectorJSON(vehicle::GetDataFile(file_name));
        m_connector = std::dynamic_pointer_cast<ChChassisConnectorHitch>(connector);
        assert(m_connector);
        if (d["Connector"].HasMember("Output")) {
            m_connector->SetOutput(d["Chassis"]["Output"].GetBool());
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

    std::cout << "Loaded JSON " << filename << std::endl;
}

void WheeledTrailer::Initialize(std::shared_ptr<ChChassis> frontChassis) {
    // Initialize the trailer chassis and its connector
    ChWheeledTrailer::Initialize(frontChassis);

    // Initialize the axles (suspension + brakes + wheels + antirollbar)
    for (int i = 0; i < m_num_axles; i++) {
        m_axles[i]->Initialize(m_chassis, nullptr, nullptr, m_suspLocations[i], ChVector3d(0), m_wheelSeparations[i]);
        // Initialize tires (if present)
        for (auto& wheel : m_axles[i]->GetWheels()) {
            if (wheel->GetTire()) {
                InitializeTire(wheel->GetTire(), wheel);
            }
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
