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

#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/yaml/ChParserFsiYAML.h"
#ifdef CHRONO_FSI_SPH
    #include "chrono_parsers/yaml/ChParserSphYAML.h"
#endif
#ifdef CHRONO_FSI_TDPF
    #include "chrono_parsers/yaml/ChParserTdpfYAML.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserFsiYAML::ChParserFsiYAML(const std::string& yaml_filename, bool verbose) : ChParserYAML() {
    SetVerbose(verbose);
    LoadFile(yaml_filename);
}

ChParserFsiYAML::~ChParserFsiYAML() {}

// -----------------------------------------------------------------------------

std::shared_ptr<utils::ChBodyGeometry> ChParserFsiYAML::ReadCollisionGeometry(const YAML::Node& a) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();

    size_t num_shapes = a.size();

    for (size_t i = 0; i < num_shapes; i++) {
        const YAML::Node& shape = a[i];
        ChAssertAlways(shape["type"]);
        std::string type = ToUpper(shape["type"].as<std::string>());

        if (type == "SPHERE") {
            ChAssertAlways(shape["location"]);
            ChAssertAlways(shape["radius"]);
            ChVector3d pos = ReadVector(shape["location"]);
            double radius = shape["radius"].as<double>();
            geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius, -1));
        } else if (type == "BOX") {
            ChAssertAlways(shape["location"]);
            ChAssertAlways(shape["orientation"]);
            ChAssertAlways(shape["dimensions"]);
            ChVector3d pos = ReadVector(shape["location"]);
            ChQuaterniond rot = ReadRotation(shape["orientation"], m_use_degrees);
            ChVector3d dims = ReadVector(shape["dimensions"]);
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(pos, rot, dims, -1));
        } else if (type == "CYLINDER") {
            ChAssertAlways(shape["location"]);
            ChAssertAlways(shape["axis"]);
            ChAssertAlways(shape["radius"]);
            ChAssertAlways(shape["length"]);
            ChVector3d pos = ReadVector(shape["location"]);
            ChVector3d axis = ReadVector(shape["axis"]);
            double radius = shape["radius"].as<double>();
            double length = shape["length"].as<double>();
            geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(pos, axis, radius, length, -1));
        } else if (type == "HULL") {
            ChAssertAlways(shape["filename"]);
            std::string filename = shape["filename"].as<std::string>();
            geometry->coll_hulls.push_back(utils::ChBodyGeometry::ConvexHullsShape(GetDatafilePath(filename), -1));
        } else if (type == "MESH") {
            ChAssertAlways(shape["filename"]);
            std::string filename = shape["filename"].as<std::string>();
            ChVector3d pos = VNULL;
            ChQuaterniond rot = QUNIT;
            double scale = 1;
            double radius = 0;
            if (shape["location"])
                pos = ReadVector(shape["location"]);
            if (shape["orientation"])
                rot = ReadRotation(shape["orientation"], m_use_degrees);
            if (shape["scale"])
                scale = shape["scale"].as<double>();
            if (shape["contact_radius"])
                radius = shape["contact_radius"].as<double>();
            geometry->coll_meshes.push_back(
                utils::ChBodyGeometry::TrimeshShape(pos, rot, GetDatafilePath(filename), scale, radius, -1));
        }
    }

    return geometry;
}

void ChParserFsiYAML::LoadFile(const std::string& yaml_filename) {
    YAML::Node yaml;

    // Load FSI YAML file
    {
        auto path = filesystem::path(yaml_filename);
        if (!path.exists() || !path.is_file()) {
            cerr << "Error: file '" << yaml_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        m_script_directory = path.parent_path().str();
        yaml = YAML::LoadFile(yaml_filename);
    }

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check the YAML file if of type "FSI"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::FSI);

    // Read the MBS and fluid specification files
    ChAssertAlways(yaml["mbs"]);
    ChAssertAlways(yaml["fluid"]);
    auto mbs_fname = yaml["mbs"].as<std::string>();
    auto fluid_fname = yaml["fluid"].as<std::string>();
    auto mbs_filename = m_script_directory + "/" + mbs_fname;
    auto fluid_filename = m_script_directory + "/" + fluid_fname;
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserFsiYAML] Loading Chrono::FSI specification from: " << yaml_filename << "\n" << endl;
        cout << "specification files" << endl;
        cout << "   multibody specification file:      " << mbs_filename << endl;
        cout << "   fluid specification file:          " << fluid_filename << endl;
    }

    // Load the MBS problem
    m_parserMBS = chrono_types::make_shared<ChParserMbsYAML>(mbs_filename, m_verbose);

    // Load the CFD problem
    m_sysCFD_type = ChParserCfdYAML::ReadFluidSystemType(fluid_filename);
    switch (m_sysCFD_type) {
        case ChParserCfdYAML::FluidSystemType::SPH:
#ifdef CHRONO_FSI_SPH
            m_parserCFD = chrono_types::make_shared<ChParserSphYAML>(fluid_filename, m_verbose);
#else
            throw std::runtime_error("Chrono::FSI-SPH not enabled");
#endif
            break;
        case ChParserCfdYAML::FluidSystemType::TDPF:
#ifdef CHRONO_FSI_TDPF
            m_parserCFD = chrono_types::make_shared<ChParserTdpfYAML>(fluid_filename, m_verbose);
#else
            throw std::runtime_error("Chrono::FSI-TDPF not enabled");
#endif
            break;
    }

    // Load the FSI problem
    ChAssertAlways(yaml["fsi"]);
    auto fsi = yaml["fsi"];
    LoadFsiData(fsi);

    // Load simulation and run-time visualization settings
    LoadSimData(yaml);
}

void ChParserFsiYAML::LoadFsiData(const YAML::Node& yaml) {
    if (yaml["name"])
        m_name = yaml["name"].as<std::string>();

    if (yaml["angle_degrees"])
        m_use_degrees = yaml["angle_degrees"].as<bool>();

    if (yaml["data_path"]) {
        ChAssertAlways(yaml["data_path"]["type"]);
        m_data_path = ReadDataPathType(yaml["data_path"]["type"]);
        if (yaml["data_path"]["root"])
            m_rel_path = yaml["data_path"]["root"].as<std::string>();
    }

    // Read FSI bodies
    if (yaml["fsi_bodies"]) {
        auto fsi_bodies = yaml["fsi_bodies"];
        ChAssertAlways(fsi_bodies.IsSequence());
        for (int i = 0; i < fsi_bodies.size(); i++) {
            FsiBody fsi_body;
            fsi_body.name = fsi_bodies[i]["name"].as<std::string>();
            fsi_body.geometry = ReadCollisionGeometry(fsi_bodies[i]["shapes"]);
            m_fsi_bodies.push_back(fsi_body);
        }
    }

    if (m_verbose) {
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        switch (m_data_path) {
            case ChParserYAML::DataPathType::ABS:
                cout << "using absolute file paths" << endl;
                break;
            case ChParserYAML::DataPathType::REL:
                cout << "using file paths relative to: '" << m_rel_path << "'" << endl;
                break;
        }
    }
}

void ChParserFsiYAML::LoadSimData(const YAML::Node& yaml) {
    // Simulation settings
    ChAssertAlways(yaml["simulation"]);
    auto sim = yaml["simulation"];
    ChAssertAlways(sim["time_step"]);
    m_sim.step = sim["time_step"].as<double>();
    if (sim["end_time"])
        m_sim.end_time = sim["end_time"].as<double>();
    if (sim["gravity"])
        m_sim.gravity = ReadVector(sim["gravity"]);

    // Run-time visualization (optional)
    if (yaml["visualization"]) {
        m_vis.render = true;
        auto vis = yaml["visualization"];
        ChAssertAlways(vis["render_fps"]);
        m_vis.render_fps = vis["render_fps"].as<double>();
        if (vis["enable_shadows"])
            m_vis.enable_shadows = vis["enable_shadows"].as<bool>();
        if (vis["camera"]) {
            if (vis["camera"]["vertical"]) {
                auto camera_vertical = ToUpper(vis["camera"]["vertical"].as<std::string>());
                if (camera_vertical == "Y")
                    m_vis.camera_vertical = CameraVerticalDir::Y;
                else if (camera_vertical == "Z")
                    m_vis.camera_vertical = CameraVerticalDir::Z;
                else {
                    cerr << "Incorrect camera vertical " << vis["camera"]["vertical"].as<std::string>() << endl;
                    throw std::runtime_error("Incorrect camera vertical");
                }
            }
            if (vis["camera"]["location"])
                m_vis.camera_location = ReadVector(vis["camera"]["location"]);
            if (vis["camera"]["target"])
                m_vis.camera_target = ReadVector(vis["camera"]["target"]);
        }

    } else {
        m_vis.render = false;
    }

    if (m_verbose) {
        m_sim.PrintInfo();
        cout << endl;
        m_vis.PrintInfo();
    }
}

void ChParserFsiYAML::CreateFsiSystem() {
    // Create and populate MBS system
    m_sysMBS = m_parserMBS->CreateSystem();
    m_sysMBS->SetGravitationalAcceleration(m_sim.gravity);
    m_parserMBS->Populate(*m_sysMBS);

    // Parse the fluid YAML files, create FSI system, and associate FSI solids
    switch (m_sysCFD_type) {
        case ChParserCfdYAML::FluidSystemType::SPH: {
#ifdef CHRONO_FSI_SPH
            // Create an SPH YAML parser and the underlying FSI problem, but do not initialize the FSI problem
            auto parserSPH = std::static_pointer_cast<ChParserSphYAML>(m_parserCFD);
            auto problemSPH = parserSPH->CreateFsiProblemSPH(false);

            // Cache the underlying FSI and CFD systems
            m_sysFSI = parserSPH->GetFsiSystem();
            m_sysCFD = parserSPH->GetFluidSystem();
            m_sysCFD->SetGravitationalAcceleration(m_sim.gravity);

            // Create the FSI problem and attach the MBS system
            problemSPH->AttachMultibodySystem(m_sysMBS.get());

            // Create FSI solids
            for (const auto& fsi_body : m_fsi_bodies) {
                auto bodies = m_parserMBS->FindBodiesByName(fsi_body.name);
                if (bodies.empty())
                    cerr << "  Warning: No body with name '" << fsi_body.name << "' was found. Ignoring." << endl;
                for (const auto& body : bodies)
                    problemSPH->AddRigidBody(body, fsi_body.geometry, true);
            }

            // Initialize the FSI problem (now that an MBS system and FSI solids are specified)
            problemSPH->Initialize();

            m_parserCFD = parserSPH;

            if (m_verbose) {
                cout << "\n-------------------------------------------------" << endl;
                cout << "\n[ChParserFsiYAML] Created FSI-SPH system" << endl;
                cout << "  Attached MBS system" << endl;
                if (!m_fsi_bodies.empty())
                    cout << "  Associated FSI rigid bodies" << endl;
                cout << "  Initialized FSI problem" << endl;
                cout << endl;
            }
#else
            throw std::runtime_error("Chrono::FSI-SPH not enabled");
#endif
            break;
        }
        case ChParserCfdYAML::FluidSystemType::TDPF: {
#ifdef CHRONO_FSI_TDPF
            // Create  a TDPF YAML parser and the underlying FSI problem, but do not initialize the FSI problem
            auto parserTDPF = std::static_pointer_cast<ChParserTdpfYAML>(m_parserCFD);
            auto problemTDPF = parserTDPF->CreateFsiSystemTDPF(false);

            // Cache the underlying FSI and CFD systems
            m_sysFSI = parserTDPF->GetFsiSystem();
            m_sysCFD = parserTDPF->GetFluidSystem();
            m_sysCFD->SetGravitationalAcceleration(m_sim.gravity);

            // Create the FSI problem and attach the MBS system
            problemTDPF->AttachMultibodySystem(m_sysMBS.get());

            // Specify FSI bodies
            for (const auto& fsi_body : m_fsi_bodies) {
                auto bodies = m_parserMBS->FindBodiesByName(fsi_body.name);
                if (bodies.empty())
                    cerr << "  Warning: No body with name '" << fsi_body.name << "' was found. Ignoring." << endl;
                for (const auto& body : bodies)
                    problemTDPF->AddFsiBody(body, fsi_body.geometry, true);
            }

            // Initialize the FSI problem (now that an MBS system and FSI solids are specified)
            problemTDPF->Initialize();

            m_parserCFD = parserTDPF;

            if (m_verbose) {
                cout << "\n-------------------------------------------------" << endl;
                cout << "\n[ChParserFsiYAML] Created FSI-TDPF system" << endl;
                cout << "  Attached MBS system" << endl;
                if (!m_fsi_bodies.empty())
                    cout << "  Associated FSI rigid bodies" << endl;
                cout << "  Initialized FSI problem" << endl;
                cout << endl;
            }

#else
            throw std::runtime_error("Chrono::FSI-TDPF not enabled");
#endif
            break;
        }
    }
}

// -----------------------------------------------------------------------------

bool ChParserFsiYAML::Output() const {
    if (m_parserMBS && m_parserCFD)
        return m_parserMBS->Output() || m_parserCFD->Output();
    return false;
}

void ChParserFsiYAML::SetOutputDir(const std::string& out_dir) {
    ChParserYAML::SetOutputDir(out_dir);

    if (m_parserMBS) {
        std::string out_dir_MBS = out_dir + "/mbs";
        if (filesystem::create_directory(filesystem::path(out_dir_MBS))) {
            m_parserMBS->SetOutputDir(out_dir_MBS);
        } else {
            std::cerr << "Error creating directory " << out_dir_MBS << std::endl;
            throw std::runtime_error("Could not create output directory");
        }
    }

    if (m_parserCFD) {
        std::string out_dir_CFD = out_dir + "/fluid";
        if (filesystem::create_directory(filesystem::path(out_dir_CFD))) {
            m_parserCFD->SetOutputDir(out_dir_CFD);
        } else {
            std::cerr << "Error creating directory " << out_dir_CFD << std::endl;
            throw std::runtime_error("Could not create output directory");
        }
    }
}

// -----------------------------------------------------------------------------

ChParserFsiYAML::SimParams::SimParams() : end_time(-1), gravity({0, 0, -9.8}) {}

void ChParserFsiYAML::SimParams::PrintInfo() {
    cout << "co-simulation parameters" << endl;
    cout << "   co-sim meta-step:  " << step << endl;
    cout << "   end time:          " << (end_time > 0 ? std::to_string(end_time) : "undefined") << endl;
    cout << "   gravitational acc: " << gravity << endl;
}

ChParserFsiYAML::VisParams::VisParams()
    : render(false),
      render_fps(120),
      camera_vertical(CameraVerticalDir::Z),
      camera_location({0, -1, 0}),
      camera_target({0, 0, 0}),
      enable_shadows(true) {}

void ChParserFsiYAML::VisParams::PrintInfo() {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

    cout << "run-time visualization" << endl;
    cout << "  render FPS:           " << render_fps << endl;
    cout << "  enable shadows?       " << std::boolalpha << enable_shadows << endl;
    cout << "  camera vertical dir:  " << (camera_vertical == CameraVerticalDir::Y ? "Y" : "Z") << endl;
    cout << "  camera location:      " << camera_location << endl;
    cout << "  camera target:        " << camera_target << endl;
}

}  // namespace parsers
}  // namespace chrono
