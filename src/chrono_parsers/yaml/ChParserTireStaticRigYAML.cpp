// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <filesystem>

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

#include "chrono_parsers/yaml/ChParserTireStaticRigYAML.h"
#include "chrono_parsers/yaml/ChParserWheelRigYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

// -----------------------------------------------------------------------------

// Return the full path of a file specified relative to the directory of a reference file.
static std::string GetRelativeFile(const std::string& reference_file, const std::string& filename) {
    auto dir = std::filesystem::path(reference_file).parent_path();
    auto path = dir.empty() ? std::filesystem::path(filename) : dir / filename;
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
        cerr << "Error: file '" << path.string() << "' not found." << endl;
        throw std::runtime_error("File not found");
    }
    return path.string();
}

// Load a YAML file and check its Chrono version.
static YAML::Node LoadYamlFile(const std::string& filename) {
    auto yaml = YAML::LoadFile(filename);
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);
    return yaml;
}

// -----------------------------------------------------------------------------

ChParserTireStaticRigYAML::ChParserTireStaticRigYAML(const std::string& yaml_filename, bool verbose)
    : ChParserMbsYAML(verbose),
      m_set_collision_type(false),
      m_collision_type(vehicle::ChTire::CollisionType::SINGLE_POINT),
      m_set_contact_surface(false),
      m_contact_surface_type(vehicle::ChTire::ContactSurfaceType::NODE_CLOUD),
      m_contact_surface_dim(0.01),
      m_collision_family(13),
      m_plate_friction(0.8),
      m_plate_restitution(0),
      m_plate_Young_modulus(2e7),
      m_mode(vehicle::ChTireStaticTestRig::Mode::TEST_R),
      m_radial_load(5000),
      m_drop_speed(1),
      m_radial_speed(1e-3),
      m_longitudinal_speed(1e-3),
      m_lateral_speed(1e-3),
      m_torsional_speed(1e-2),
      m_transition_delay(0.1),
      m_camera_offset(1.0, 2.5, 0.5),
      m_camera_target_offset(0, 0, 0) {
    LoadFile(yaml_filename);
}

ChParserTireStaticRigYAML::~ChParserTireStaticRigYAML() {}

// -----------------------------------------------------------------------------

void ChParserTireStaticRigYAML::LoadFile(const std::string& yaml_filename) {
    auto yaml = LoadYamlFile(yaml_filename);

    // Check the YAML file is of type "TIRE_STATIC_RIG"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::TIRE_STATIC_RIG);

    // Load simulation, output, and run-time visualization data
    LoadSimData(yaml);
    if (yaml["visualization"] && yaml["visualization"]["camera"]) {
        auto camera = yaml["visualization"]["camera"];
        if (camera["offset"])
            m_camera_offset = ReadVector(camera["offset"]);
        if (camera["target_offset"])
            m_camera_target_offset = ReadVector(camera["target_offset"]);
    }

    // The test rig assumes a vertical Z axis
    const auto& gravity = GetGravitationalAcceleration();
    if (std::abs(gravity.x()) > 1e-12 || std::abs(gravity.y()) > 1e-12 || gravity.z() > 0) {
        cerr << "Error: the gravitational acceleration must be of the form [0, 0, -g]." << endl;
        throw std::runtime_error("Incorrect gravitational acceleration");
    }

    // Load the rig model file
    ChAssertAlways(yaml["rig"]);
    LoadRigFile(GetRelativeFile(yaml_filename, yaml["rig"].as<std::string>()));

    // Load the rig plate properties
    if (yaml["plate"]) {
        auto plate = yaml["plate"];
        if (plate["friction"])
            m_plate_friction = plate["friction"].as<double>();
        if (plate["restitution"])
            m_plate_restitution = plate["restitution"].as<double>();
        if (plate["Young_modulus"])
            m_plate_Young_modulus = plate["Young_modulus"].as<double>();
    }

    // Load the test scenario
    ChAssertAlways(yaml["test"]);
    LoadTestData(yaml["test"]);

    // Load plot specifications
    if (yaml["plots"])
        m_plots.Load(yaml["plots"]);

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nStatic tire test rig: " << m_name << endl;
        cout << "  wheel:        " << m_wheel_json << endl;
        cout << "  tire:         " << m_tire_json << endl;
        cout << "  radial load:  " << m_radial_load << endl;
        cout << "  step size:    " << GetTimestep() << endl;
        cout << "  plots:        " << m_plots.GetNumFigures() << endl;
        cout << endl;
        m_vis_settings.PrintInfo();
    }

    m_loaded = true;
}

void ChParserTireStaticRigYAML::LoadRigFile(const std::string& filename) {
    if (m_verbose)
        cout << "\nLoading static tire test rig model from: " << filename << endl;

    auto yaml = LoadYamlFile(filename);

    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    // Data files are located relative to the rig file
    m_file_handler.SetReferenceDirectory(filename);
    m_file_handler.Read(model);

    // Only Chrono::Vehicle wheel and tire subsystems can be tested
    if (model["type"] && ChToUpper(model["type"].as<std::string>()) != "VEHICLE") {
        cerr << "Error: the static tire test rig requires a rig model of VEHICLE type." << endl;
        throw std::runtime_error("Unsupported rig model type");
    }

    ChAssertAlways(model["wheel_json"]);
    ChAssertAlways(model["tire_json"]);
    m_wheel_json = m_file_handler.GetFilename(model["wheel_json"].as<std::string>());
    m_tire_json = m_file_handler.GetFilename(model["tire_json"].as<std::string>());
    if (model["suspension_json"])
        cout << "Note: the rig model 'suspension_json' is ignored for a static tire test rig." << endl;

    if (model["tire"] && model["tire"]["collision_type"]) {
        m_set_collision_type = true;
        m_collision_type = ChParserWheelRigYAML::ReadTireCollisionType(model["tire"]["collision_type"]);
    }
    if (model["tire"] && model["tire"]["contact_surface"]) {
        auto surface = model["tire"]["contact_surface"];
        ChAssertAlways(surface["type"]);
        m_set_contact_surface = true;
        m_contact_surface_type = ChParserWheelRigYAML::ReadTireContactSurfaceType(surface["type"]);
        if (surface["dimension"])
            m_contact_surface_dim = surface["dimension"].as<double>();
        if (surface["collision_family"])
            m_collision_family = surface["collision_family"].as<int>();
    }

    // The rig normal load is the default nominal radial load
    if (yaml["rig"] && yaml["rig"]["normal_load"])
        m_radial_load = yaml["rig"]["normal_load"].as<double>();

    // MBS solver (relative to the rig file)
    ChAssertAlways(yaml["solver"]);
    auto solver_filename = GetRelativeFile(filename, yaml["solver"].as<std::string>());
    if (m_verbose)
        cout << "Loading MBS solver specification from: " << solver_filename << endl;
    LoadSolverData(LoadYamlFile(solver_filename));
}

void ChParserTireStaticRigYAML::LoadTestData(const YAML::Node& a) {
    ChAssertAlways(a["mode"]);
    m_mode = ReadMode(a["mode"]);

    if (a["radial_load"])
        m_radial_load = a["radial_load"].as<double>();  // overrides the normal load from the rig model file
    if (a["drop_speed"])
        m_drop_speed = a["drop_speed"].as<double>();
    if (a["radial_speed"])
        m_radial_speed = a["radial_speed"].as<double>();
    if (a["longitudinal_speed"])
        m_longitudinal_speed = a["longitudinal_speed"].as<double>();
    if (a["lateral_speed"])
        m_lateral_speed = a["lateral_speed"].as<double>();
    if (a["torsional_speed"])
        m_torsional_speed = a["torsional_speed"].as<double>();
    if (a["transition_delay"])
        m_transition_delay = a["transition_delay"].as<double>();
}

// -----------------------------------------------------------------------------

void ChParserTireStaticRigYAML::WriteOutput(int frame, double time) {
    if (!m_rig)
        return;

    auto channels = m_rig->GetOutputChannels();

    // Write to the output database (created if needed; null if no output database or format not available)
    ChParserYAML::WriteOutput(frame, time);
    if (m_output_db)
        m_output_db->Write(frame, time, {}, channels);

    // Record channels for plotting
    m_plots.Record(time, channels);
}

void ChParserTireStaticRigYAML::GeneratePlots() {
    m_plots.Generate(m_output_dir);
}

std::shared_ptr<vehicle::ChTireStaticTestRig> ChParserTireStaticRigYAML::CreateRig(ChSystem& sys) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nCreate static tire test rig\n" << endl;
    }

    // Create the wheel and tire subsystems
    auto wheel = vehicle::ReadWheelJSON(m_wheel_json);
    auto tire = vehicle::ReadTireJSON(m_tire_json);

    // Handling tires have no collision geometry and cannot be tested on the rig plate
    if (std::dynamic_pointer_cast<vehicle::ChForceElementTire>(tire)) {
        cerr << "Error: handling tire models cannot be used with the static tire test rig." << endl;
        throw std::runtime_error("Incompatible tire model");
    }

    if (m_set_contact_surface)
        tire->SetContactSurfaceType(m_contact_surface_type, m_contact_surface_dim, m_collision_family);

    // Create the test rig (which sets a default tire collision type)
    m_rig = chrono_types::make_shared<vehicle::ChTireStaticTestRig>(wheel, tire, &sys);

    // Rig and tire properties
    m_rig->SetGravitationalAcceleration(-GetGravitationalAcceleration().z());
    m_rig->SetTireStepsize(GetTimestep());
    m_rig->SetTireVisualizationType(GetVisualizationType());
    if (m_set_collision_type)
        m_rig->SetTireCollisionType(m_collision_type);
    m_rig->SetPlateMaterialProperties(m_plate_friction, m_plate_restitution, m_plate_Young_modulus);

    // Test scenario
    m_rig->SetNominalRadialLoad(m_radial_load);
    m_rig->SetDropSpeed(m_drop_speed);
    m_rig->SetRadialLoadSpeed(m_radial_speed);
    m_rig->SetLongitudinalLoadSpeed(m_longitudinal_speed);
    m_rig->SetLateralLoadSpeed(m_lateral_speed);
    m_rig->SetTorsionalLoadSpeed(m_torsional_speed);
    m_rig->SetStateTransitionDelay(m_transition_delay);

    m_rig->Initialize(m_mode);

    return m_rig;
}

// -----------------------------------------------------------------------------

vehicle::ChTireStaticTestRig::Mode ChParserTireStaticRigYAML::ReadMode(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "SUSPEND")
        return vehicle::ChTireStaticTestRig::Mode::SUSPEND;
    if (val == "TEST_R")
        return vehicle::ChTireStaticTestRig::Mode::TEST_R;
    if (val == "TEST_X")
        return vehicle::ChTireStaticTestRig::Mode::TEST_X;
    if (val == "TEST_Y")
        return vehicle::ChTireStaticTestRig::Mode::TEST_Y;
    if (val == "TEST_Z")
        return vehicle::ChTireStaticTestRig::Mode::TEST_Z;
    cerr << "Error: unknown static tire test rig mode '" << a.as<std::string>() << "'." << endl;
    throw std::runtime_error("Unknown static tire test rig mode");
}

}  // namespace parsers
}  // namespace chrono
