// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

//// TODO
////    For now, we assume that the YAML file and the vehicle JSON data files reside in the Chrono data directory.
////    Relax this constraint.

//// TODO - support tracked vehicles

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_parsers/yaml/ChParserVehicleYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserVehicleYAML::ChParserVehicleYAML(const std::string& yaml_model_filename,
                                         const std::string& yaml_sim_filename,
                                         bool verbose)
    : m_name("YAML model"),
      m_verbose(verbose),
      m_init_position(VNULL),
      m_init_yaw(0.0),
      m_chassis_point(VNULL),
      m_chase_distance(5.0),
      m_chase_height(0.5),
      m_vis_chassis(VisualizationType::MESH),
      m_vis_trailer(VisualizationType::MESH),
      m_vis_subchassis(VisualizationType::PRIMITIVES),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::MESH),
      m_vis_tire(VisualizationType::MESH) {
    m_parserMBS = chrono_types::make_shared<ChParserMbsYAML>();
    m_parserMBS->LoadSimulationFile(yaml_sim_filename);

    LoadModelFile(yaml_model_filename);
}

ChParserVehicleYAML::~ChParserVehicleYAML() {}

// -----------------------------------------------------------------------------

static std::string ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

static void CheckVersion(const YAML::Node& a) {
    std::string chrono_version = a.as<std::string>();

    auto first = chrono_version.find(".");
    ChAssertAlways(first != std::string::npos);
    std::string chrono_major = chrono_version.substr(0, first);

    ChAssertAlways(first < chrono_version.size() - 1);
    chrono_version = &chrono_version[first + 1];

    auto second = chrono_version.find(".");
    if (second == std::string::npos)
        second = chrono_version.size();
    std::string chrono_minor = chrono_version.substr(0, second);

    ChAssertAlways(chrono_major == CHRONO_VERSION_MAJOR);
    ChAssertAlways(chrono_minor == CHRONO_VERSION_MINOR);
}

ChParserVehicleYAML::VehicleType ChParserVehicleYAML::ReadVehicleType(const std::string& vehicle_json) {
    // Peek in vehicle JSON file and infer type
    rapidjson::Document d;
    vehicle::ReadFileJSON(vehicle_json, d);
    ChAssertAlways(!d.IsNull());
    ChAssertAlways(d.HasMember("Type"));
    ChAssertAlways(d.HasMember("Template"));

    std::string type = d["Type"].GetString();
    ChAssertAlways(type.compare("Vehicle") == 0);
    std::string subtype = d["Template"].GetString();

    if (subtype == "WheeledVehicle")
        return VehicleType::WHEELED;
    return VehicleType::TRACKED;
}

std::string ChParserVehicleYAML::GetVehicleTypeAsString() const {
    if (m_vehicle_type == VehicleType::WHEELED)
        return "Wheeled";
    return "Tracked";
}

void ChParserVehicleYAML::LoadModelFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    std::string script_dir = path.parent_path().str();

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Read the model
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    ChAssertAlways(model["vehicle_json"]);
    ChAssertAlways(model["engine_json"]);
    ChAssertAlways(model["transmission_json"]);
    auto vehicle_json = model["vehicle_json"].as<std::string>();
    auto engine_json = model["engine_json"].as<std::string>();
    auto transmission_json = model["transmission_json"].as<std::string>();

    m_vehicle_json = script_dir + "/" + vehicle_json;
    m_engine_json = script_dir + "/" + engine_json;
    m_transmission_json = script_dir + "/" + transmission_json;

    m_vehicle_type = ReadVehicleType(GetChronoDataFile(m_vehicle_json));

    if (m_vehicle_type == VehicleType::WHEELED) {
        ChAssertAlways(model["tire_json"]);
        auto tire_json = model["tire_json"].as<std::string>();
        m_tire_json = script_dir + "/" + tire_json;
    }

    if (yaml["terrain_json"]) {
        auto terrain_json = yaml["terrain_json"].as<std::string>();
        m_terrain_json = script_dir + "/" + terrain_json;
    }

    if (yaml["initial_position"])
        m_init_position = ChParserMbsYAML::ReadVector(yaml["initial_position"]);
    if (yaml["initial_yaw"])
        m_init_yaw = CH_DEG_TO_RAD * yaml["initial_yaw"].as<double>();

    if (yaml["chase_camera"]) {
        auto a = yaml["chase_camera"];
        ChAssertAlways(a["chassis_point"]);
        ChAssertAlways(a["chase_distance"]);
        ChAssertAlways(a["chase_height"]);
        m_chassis_point = ChParserMbsYAML::ReadVector(a["chassis_point"]);
        m_chase_distance = a["chase_distance"].as<double>();
        m_chase_height = a["chase_height"].as<double>();
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nLoading Chrono::Vehicle specification from: " << yaml_filename << "\n" << endl;
        cout << "Model name: '" << m_name << "'" << endl;
        cout << "JSON specification files" << endl;
        cout << "   Vehicle:      " << m_vehicle_json << endl;
        cout << "   Engine:       " << m_engine_json << endl;
        cout << "   Transmission: " << m_transmission_json << endl;
        cout << "   Tire:         " << m_tire_json << endl;
        if (!m_terrain_json.empty())
            cout << "   Terrain:      " << m_terrain_json << endl;
    }

    m_model_loaded = true;
}

void ChParserVehicleYAML::CreateVehicle(ChSystem& sys) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nCreate vehicle (" << GetVehicleTypeAsString() << ")\n" << endl;
    }

    // Create a wheeled vehicle system
    std::shared_ptr<vehicle::WheeledVehicle> vehicleW;
    std::shared_ptr<vehicle::TrackedVehicle> vehicleT;

    switch (m_vehicle_type) {
        case VehicleType::WHEELED:
            vehicleW = chrono_types::make_shared<vehicle::WheeledVehicle>(&sys, GetChronoDataFile(m_vehicle_json),
                                                                          false, false);
            vehicleW->Initialize(ChCoordsys<>(m_init_position, QuatFromAngleZ(m_init_yaw)));
            vehicleW->GetChassis()->SetFixed(false);

            vehicleW->SetChassisVisualizationType(m_vis_chassis);
            vehicleW->SetChassisRearVisualizationType(m_vis_trailer);
            vehicleW->SetSubchassisVisualizationType(m_vis_subchassis);
            vehicleW->SetSuspensionVisualizationType(m_vis_suspension);
            vehicleW->SetSteeringVisualizationType(m_vis_steering);
            vehicleW->SetWheelVisualizationType(m_vis_wheel);

            m_vehicle = vehicleW;

            break;

        case VehicleType::TRACKED:
            vehicleT = chrono_types::make_shared<vehicle::TrackedVehicle>(&sys, GetChronoDataFile(m_vehicle_json));
            vehicleT->Initialize(ChCoordsys<>(m_init_position, QuatFromAngleZ(m_init_yaw)));
            vehicleT->GetChassis()->SetFixed(false);

            vehicleT->SetChassisVisualizationType(m_vis_chassis);
            vehicleT->SetSuspensionVisualizationType(m_vis_suspension);
            vehicleT->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetRollerVisualizationType(VisualizationType::PRIMITIVES);
            vehicleT->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

            m_vehicle = vehicleT;

            break;
    }

    // Create and initialize the powertrain system
    auto engine = vehicle::ReadEngineJSON(GetChronoDataFile(m_engine_json));
    auto transmission = vehicle::ReadTransmissionJSON(GetChronoDataFile(m_transmission_json));
    auto powertrain = chrono_types::make_shared<vehicle::ChPowertrainAssembly>(engine, transmission);
    m_vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    if (m_vehicle_type == VehicleType::WHEELED) {
        for (unsigned int i = 0; i < vehicleW->GetNumberAxles(); i++) {
            for (auto& wheel : vehicleW->GetAxle(i)->GetWheels()) {
                auto tire = vehicle::ReadTireJSON(GetChronoDataFile(m_tire_json));
                vehicleW->InitializeTire(tire, wheel, m_vis_tire);
            }
        }
    }

    // Create a rigid terrain (if specified)
    if (!m_terrain_json.empty()) {
        m_terrain = chrono_types::make_shared<vehicle::RigidTerrain>(&sys, m_terrain_json);
        m_terrain->Initialize();
    }
}

}  // namespace parsers
}  // namespace chrono
