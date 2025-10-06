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
#include "chrono_parsers/yaml/ChParserSphYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserFsiYAML::ChParserFsiYAML(const std::string& yaml_filename, bool verbose)
    : ChParserYAML(), m_end_time(-1), m_render(false), m_output(false) {
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

    if (model["angle_degrees"])
        m_use_degrees = model["angle_degrees"].as<bool>();

    if (model["data_path"]) {
        ChAssertAlways(model["data_path"]["type"]);
        m_data_path = ReadDataPathType(model["data_path"]["type"]);
        if (model["data_path"]["root"])
            m_rel_path = model["data_path"]["root"].as<std::string>();
    }

    auto modelMBS = model["multibody_model"].as<std::string>();
    auto simMBS = model["multibody_simulation"].as<std::string>();
    auto modelCFD = model["fluid_model"].as<std::string>();
    auto simCFD = model["fluid_simulation"].as<std::string>();

    m_file_modelMBS = script_dir + "/" + modelMBS;
    m_file_simMBS = script_dir + "/" + simMBS;
    m_file_modelCFD = script_dir + "/" + modelCFD;
    m_file_simCFD = script_dir + "/" + simCFD;

    // Read FSI bodies
    if (model["fsi_bodies"]) {
        auto fsi_bodies = model["fsi_bodies"];
        ChAssertAlways(fsi_bodies.IsSequence());
        for (int i = 0; i < fsi_bodies.size(); i++) {
            FsiBody fsi_body;
            fsi_body.name = fsi_bodies[i]["name"].as<std::string>();
            fsi_body.geometry = ReadCollisionGeometry(fsi_bodies[i]["shapes"]);
            m_fsi_bodies.push_back(fsi_body);
        }
    }

    // Read meta-step and simulation end time
    ChAssertAlways(yaml["simulation"]);
    auto sim = yaml["simulation"];
    ChAssertAlways(sim["time_step"]);
    m_step = sim["time_step"].as<double>();
    if (sim["end_time"])
        m_end_time = sim["end_time"].as<double>();

    // Read visualization settings
    if (yaml["visualization"]) {
        m_render = true;
        auto vis = yaml["visualization"];
        ChAssertAlways(vis["render_fps"]);
        m_render_fps = vis["render_fps"].as<double>();
    }

    // Read output settings
    if (yaml["output"]) {
        m_output = true;
        auto out = yaml["output"];
        ChAssertAlways(out["output_fps"]);
        m_output_fps = out["output_fps"].as<double>();
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserFsiYAML] Loading Chrono::FSI specification from: " << yaml_filename << "\n" << endl;
        cout << "    Model name: '" << m_name << "'" << endl;
        cout << "    Specification files" << endl;
        cout << "       Multibody model specification file:      " << m_file_modelMBS << endl;
        cout << "       Multibody simulation specification file: " << m_file_simMBS << endl;
        cout << "       Fluid model specification file:          " << m_file_modelCFD << endl;
        cout << "       Fluid simulation specification file:     " << m_file_simCFD << endl;
        cout << "    Angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        switch (m_data_path) {
            case ChParserYAML::DataPathType::ABS:
                cout << "    Using absolute file paths" << endl;
                break;
            case ChParserYAML::DataPathType::REL:
                cout << "    Using file paths relative to: '" << m_rel_path << "'" << endl;
                break;
        }
    }
}

void ChParserFsiYAML::CreateFsiSystem() {
    // Parse the multibody YAML files, create and populate MBS system
    m_parserMBS = chrono_types::make_shared<ChParserMbsYAML>(m_file_modelMBS, m_file_simMBS, m_verbose);
    m_sysMBS = m_parserMBS->CreateSystem();
    m_parserMBS->Populate(*m_sysMBS);

    // Peek in fluid simulation YAML file and extract fluid solver type
    m_sysCFD_type = ChParserCfdYAML::ReadFluidSystemType(m_file_simCFD);

    // Parse the fluid YAML files, create FSI system, and associate FSI solids
    switch (m_sysCFD_type) {
        case ChParserCfdYAML::FluidSystemType::SPH: {
            // Create an SPH YAML parser
            auto parserSPH = chrono_types::make_shared<ChParserSphYAML>(m_file_modelCFD, m_file_simCFD, m_verbose);

            // Access the underlying FSI problem and attach the MBS system
            auto problemSPH = parserSPH->CreateFsiProblemSPH(false);
            if (m_verbose)
                cout << "Attach MBS system" << endl;
            problemSPH->AttachMultibodySystem(m_sysMBS.get());

            // Cache the underlying FSI and CFD systems
            m_sysFSI = problemSPH->GetFsiSystemSPH();
            m_sysCFD = problemSPH->GetFluidSystemSPH();

            // Create FSI solids
            if (m_verbose && !m_fsi_bodies.empty())
                cout << "Associate FSI rigid bodies" << endl;
            for (const auto& fsi_body : m_fsi_bodies) {
                auto bodies = m_parserMBS->FindBodiesByName(fsi_body.name);
                if (bodies.empty())
                    cerr << "  Warning: No body with name '" << fsi_body.name << "' was found. Ignoring." << endl;
                for (auto body : bodies)
                    problemSPH->AddRigidBody(body, fsi_body.geometry, true);
            }

            // Initialize the FSI problem (now that an MBS system and FSI solids are specified)
            if (m_verbose)
                cout << "Initialize FSI problem" << endl;
            problemSPH->Initialize();

            m_parserCFD = parserSPH;
            break;
        }
        case ChParserCfdYAML::FluidSystemType::BEM: {
            throw std::runtime_error("BEM fluid system not yet supported");
            ////auto parserBEM = chrono_types::make_shared<ChParserBemYAML>(m_file_modelCFD, m_file_simCFD, m_verbose);
            ////m_parserCFD = parserBEM;
            ////break;
        }
    }
}

}  // namespace parsers
}  // namespace chrono
