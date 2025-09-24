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

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
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
    : m_name("YAML model"), m_verbose(false), m_end_time(-1), m_render(false), m_output(false) {
    LoadFile(yaml_filename);
}

ChParserFsiYAML::~ChParserFsiYAML() {}

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
    auto modelMBS = model["multibody_model"].as<std::string>();
    auto simMBS = model["multibody_simulation"].as<std::string>();
    auto modelCFD = model["fluid_model"].as<std::string>();
    auto simCFD = model["fluid_simulation"].as<std::string>();
    if (model["name"])
        m_name = model["name"].as<std::string>();

    m_file_modelMBS = script_dir + "/" + modelMBS;
    m_file_simMBS = script_dir + "/" + simMBS;
    m_file_modelCFD = script_dir + "/" + modelCFD;
    m_file_simCFD = script_dir + "/" + simCFD;

    // Read FSI bodies
    if (yaml["fsi_bodies"]) {
        auto fsi_bodies = yaml["fsi_bodies"];
        ChAssertAlways(fsi_bodies.IsSequence());
        for (int i = 0; i < fsi_bodies.size(); i++) {
            m_fsi_bodies.push_back(fsi_bodies[i].as<std::string>());
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
        cout << "\nLoading Chrono::FSI specification from: " << yaml_filename << "\n" << endl;
        cout << "    Model name: '" << m_name << "'" << endl;
        cout << "    Specification files" << endl;
        cout << "       Multibody model specification file:      " << m_file_modelMBS << endl;
        cout << "       Multibody simulation specification file: " << m_file_simMBS << endl;
        cout << "       Fluid model specification file:          " << m_file_modelCFD << endl;
        cout << "       Fluid simulation specification file:     " << m_file_simCFD << endl;
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
                if (m_parserMBS->HasBodyParams(fsi_body)) {
                    const auto& body_params = m_parserMBS->FindBodyParams(fsi_body);
                    for (const auto& body : body_params.body)
                        problemSPH->AddRigidBody(body, body_params.geometry, true);
                } else {
                    cerr << "  Warning: No body with name '" << fsi_body << "' was found. Ignoring." << endl;
                }
            }

            // Initialize the FSI problem (now that an MBS system and FSI solids are specified)
            if (m_verbose)
                cout << "Initialize FSI problem" << endl;
            problemSPH->Initialize();

            if (m_verbose) {
                auto domain_aabb = problemSPH->GetComputationalDomain();
                cout << "Computational domain: " << endl;
                cout << "   min: " << domain_aabb.min << endl;
                cout << "   max: " << domain_aabb.max << endl;
            }

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
