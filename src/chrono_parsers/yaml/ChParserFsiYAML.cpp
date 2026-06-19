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

#include <filesystem>

#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/yaml/ChParserFsiYAML.h"
#ifdef CHRONO_FSI_SPH
    #include "chrono_parsers/yaml/ChParserSphYAML.h"
#endif
#ifdef CHRONO_FSI_TDPF
    #include "chrono_parsers/yaml/ChParserTdpfYAML.h"
#endif

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

void ChParserFsiYAML::LoadFile(const std::string& yaml_filename) {
    YAML::Node yaml;

    // Load FSI YAML file
    yaml = YAML::LoadFile(yaml_filename);
    m_file_handler.SetReferenceDirectory(yaml_filename);

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
    auto mbs_filename = m_file_handler.GetReferenceDirectory() + "/" + mbs_fname;
    auto fluid_filename = m_file_handler.GetReferenceDirectory() + "/" + fluid_fname;
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

    m_file_handler.Read(yaml);

    // Read FSI bodies
    if (yaml["fsi_bodies"]) {
        auto fsi_bodies = yaml["fsi_bodies"];
        ChAssertAlways(fsi_bodies.IsSequence());
        for (int i = 0; i < fsi_bodies.size(); i++) {
            FsiBody fsi_body;
            fsi_body.name = fsi_bodies[i]["name"].as<std::string>();
            fsi_body.geometry = ReadCollisionGeometry(fsi_bodies[i]["shapes"], m_file_handler, m_use_degrees);
            m_fsi_bodies.push_back(fsi_body);
        }
    }

    if (m_verbose) {
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        m_file_handler.PrintInfo();
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
        m_vis_settings = ChVisualSystem::Settings::Read(yaml["visualization"]);
    }

    if (m_verbose) {
        m_sim.PrintInfo();
        cout << endl;
        m_vis_settings.PrintInfo();
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

bool ChParserFsiYAML::Render() const {
    if (!ChParserYAML::Render())
        return false;

    if (m_parserMBS && m_parserMBS->Render())
        return true;

    if (m_parserCFD && m_parserCFD->Render())
        return true;

    return false;
}

bool ChParserFsiYAML::Output() const {
    if (!ChParserYAML::Output())
        return false;

    if (m_parserMBS && m_parserMBS->Output())
        return true;

    if (m_parserCFD && m_parserCFD->Output())
        return true;

    return false;
}

void ChParserFsiYAML::SetOutputDir(const std::string& out_dir) {
    ChParserYAML::SetOutputDir(out_dir);

    if (m_parserMBS) {
        std::string out_dir_MBS = out_dir + "/mbs";
        if (CreateOutputDirectory(std::filesystem::path(out_dir_MBS))) {
            m_parserMBS->SetOutputDir(out_dir_MBS);
        } else {
            std::cerr << "Error creating directory " << out_dir_MBS << std::endl;
            throw std::runtime_error("Could not create output directory");
        }
    }

    if (m_parserCFD) {
        std::string out_dir_CFD = out_dir + "/fluid";
        if (CreateOutputDirectory(std::filesystem::path(out_dir_CFD))) {
            m_parserCFD->SetOutputDir(out_dir_CFD);
        } else {
            std::cerr << "Error creating directory " << out_dir_CFD << std::endl;
            throw std::runtime_error("Could not create output directory");
        }
    }
}

// -----------------------------------------------------------------------------

ChParserFsiYAML::SimParams::SimParams() : end_time(-1), gravity({0, 0, -9.8}) {}

void ChParserFsiYAML::SimParams::PrintInfo() const {
    cout << "co-simulation parameters" << endl;
    cout << "   co-sim meta-step:  " << step << endl;
    cout << "   end time:          " << (end_time > 0 ? std::to_string(end_time) : "undefined") << endl;
    cout << "   gravitational acc: " << gravity << endl;
}

}  // namespace parsers
}  // namespace chrono
