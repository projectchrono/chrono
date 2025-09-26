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

//// TODO - support tracked vehicles

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

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
    ChAssertAlways(model["tire_json"]);
    auto vehicle_json = model["vehicle_json"].as<std::string>();
    auto engine_json = model["engine_json"].as<std::string>();
    auto transmission_json = model["transmission_json"].as<std::string>();
    auto tire_json = model["tire_json"].as<std::string>();

    m_vehicle_json = script_dir + "/" + vehicle_json;
    m_engine_json = script_dir + "/" + engine_json;
    m_transmission_json = script_dir + "/" + transmission_json;
    m_tire_json = script_dir + "/" + tire_json;

    if (model["terrain_json"]) {
        auto terrain_json = model["terrain_json"].as<std::string>();
        m_terrain_json = script_dir + "/" + terrain_json;
    }

    if (model["initial_position"])
        m_init_position = ChParserMbsYAML::ReadVector(model["initial_position"]);
    if (model["initial_yaw"])
        m_init_yaw = CH_DEG_TO_RAD * model["initial_yaw"].as<double>();

    if (model["chase_camera"]) {
        auto a = model["chase_camera"];
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
    // Create a wheeled vehicle system
    auto vehicle =
        chrono_types::make_shared<vehicle::WheeledVehicle>(&sys, GetChronoDataFile(m_vehicle_json), false, false);
    vehicle->Initialize(ChCoordsys<>(m_init_position, QuatFromAngleZ(m_init_yaw)));
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(m_vis_chassis);
    vehicle->SetChassisRearVisualizationType(m_vis_trailer);
    vehicle->SetSubchassisVisualizationType(m_vis_subchassis);
    vehicle->SetSuspensionVisualizationType(m_vis_suspension);
    vehicle->SetSteeringVisualizationType(m_vis_steering);
    vehicle->SetWheelVisualizationType(m_vis_wheel);

    // Create and initialize the powertrain system
    auto engine = vehicle::ReadEngineJSON(GetChronoDataFile(m_engine_json));
    auto transmission = vehicle::ReadTransmissionJSON(GetChronoDataFile(m_transmission_json));
    auto powertrain = chrono_types::make_shared<vehicle::ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (unsigned int i = 0; i < vehicle->GetNumberAxles(); i++) {
        for (auto& wheel : vehicle->GetAxle(i)->GetWheels()) {
            auto tire = vehicle::ReadTireJSON(GetChronoDataFile(m_tire_json));
            vehicle->InitializeTire(tire, wheel, m_vis_tire);
        }
    }

    // Create a rigid terrain (if specified)
    if (!m_terrain_json.empty()) {
        m_terrain = chrono_types::make_shared<vehicle::RigidTerrain>(&sys, m_terrain_json);
        m_terrain->Initialize();
    }

    // Cache the vehicle object
    m_vehicle = vehicle;
}

}  // namespace parsers
}  // namespace chrono
