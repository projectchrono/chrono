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
//// - output

#include <algorithm>
#include <filesystem>

#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserTdpfYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserTdpfYAML::ChParserTdpfYAML(const std::string& yaml_filename, bool verbose)
    : ChParserCfdYAML(verbose),
      m_gravity({0, 0, -9.8}),
      m_ramp_duration(0),
      m_loaded(false),
      m_solver_loaded(false),
      m_model_loaded(false) {
    SetVerbose(verbose);
    LoadFile(yaml_filename);
}

ChParserTdpfYAML::~ChParserTdpfYAML() {}

// -----------------------------------------------------------------------------

void ChParserTdpfYAML::LoadFile(const std::string& yaml_filename) {
    YAML::Node yaml;

    // Load SPH YAML file
    yaml = YAML::LoadFile(yaml_filename);
    m_file_handler.SetReferenceDirectory(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check the YAML file if of type "TDPF"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::TDPF);

    // Load simulation, output, and run-time visualization data
    LoadSimData(yaml);

    // Load TDPF model YAML file
    {
        ChAssertAlways(yaml["model"]);
        auto model_fname = yaml["model"].as<std::string>();
        auto model_filename = m_file_handler.GetReferenceDirectory() + "/" + model_fname;
        auto path = std::filesystem::path(model_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << model_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserTdpfYAML] Loading Chrono TDPF model from: '" << yaml_filename << "'\n" << endl;
        }
        auto model = YAML::LoadFile(model_filename);
        ChAssertAlways(model["chrono-version"]);
        CheckVersion(model["chrono-version"]);
        LoadModelData(model);
    }

    // Load solver YAML file
    {
        ChAssertAlways(yaml["solver"]);
        auto solver_fname = yaml["solver"].as<std::string>();
        auto solver_filename = m_file_handler.GetReferenceDirectory() + "/" + solver_fname;
        auto path = std::filesystem::path(solver_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << solver_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserTdpfYAML] Loading Chrono TDPF solver from: " << solver_filename << "\n" << endl;
        }
        auto solver = YAML::LoadFile(solver_filename);
        ChAssertAlways(solver["chrono-version"]);
        CheckVersion(solver["chrono-version"]);
        LoadSolverData(solver);
    }

    if (m_verbose) {
        m_vis_settings.PrintInfo();
#ifdef CHRONO_VSG
        m_visTDPF_settings.PrintInfo();
#endif
        cout << endl;
        m_output_settings.PrintInfo();
    }

    m_loaded = true;
}

void ChParserTdpfYAML::LoadSimData(const YAML::Node& yaml) {
    // Read common simulation settings
    ChParserYAML::LoadSimData(yaml);

    // TDPF-specific run-time visualization (optional)
    if (yaml["visualization"]) {
#ifdef CHRONO_VSG
        m_visTDPF_settings = fsi::tdpf::ChTdpfVisualizationVSG::Settings::Read(yaml["visualization"]);
#else
        m_vis.render = false;
#endif
    }
}

void ChParserTdpfYAML::LoadSolverData(const YAML::Node& yaml) {
    // Nothing to do here
    m_solver_loaded = true;
}

void ChParserTdpfYAML::LoadModelData(const YAML::Node& yaml) {
    // Check a model object exists
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    if (model["angle_degrees"])
        m_use_degrees = model["angle_degrees"].as<bool>();

    m_file_handler.Read(model);

    if (m_verbose) {
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        m_file_handler.PrintInfo();
    }

    // Read HDF5 hydrodynamic filename
    ChAssertAlways(model["h5_file"]);
    m_h5_file = model["h5_file"].as<std::string>();

    // Read wave information
    if (model["waves"]) {
        auto waves = model["waves"];
        ChAssertAlways(waves["type"]);
        m_wave_type = ReadWaveType(waves["type"]);
        switch (m_wave_type) {
            case WaveType::NONE:
                m_sea_state.type = "none";
                break;
            case WaveType::REGULAR: {
                ChAssertAlways(waves["height"]);
                ChAssertAlways(waves["period"]);
                m_sea_state.type = "regular";
                m_sea_state.amplitude = 0.5 * waves["height"].as<double>();
                m_sea_state.omega = CH_2PI / waves["period"].as<double>();
                if (waves["phase"])
                    m_sea_state.phase_rad = waves["phase"].as<double>();
                if (waves["direction"]) {
                    double dir = waves["direction"].as<double>();
                    m_sea_state.direction_deg = m_use_degrees ? dir : dir * CH_RAD_TO_DEG;
                }
                break;
            }
            case WaveType::IRREGULAR: {
                ChAssertAlways(waves["height"]);
                ChAssertAlways(waves["period"]);
                m_sea_state.type = "irregular";

                fsi::tdpf::ChTdpfSeaStatePartition partition;
                partition.spectrum.Hs = waves["height"].as<double>();
                partition.spectrum.Tp = waves["period"].as<double>();
                if (waves["spectrum"])
                    partition.spectrum.type = waves["spectrum"].as<std::string>();
                if (waves["gamma"])
                    partition.spectrum.gamma = waves["gamma"].as<double>();
                if (waves["direction"]) {
                    double dir = waves["direction"].as<double>();
                    partition.spreading.mean_direction_deg = m_use_degrees ? dir : dir * CH_RAD_TO_DEG;
                }
                if (waves["spreading"]) {
                    auto spreading = waves["spreading"];
                    if (spreading["type"])
                        partition.spreading.type = spreading["type"].as<std::string>();
                    if (spreading["s"])
                        partition.spreading.s = spreading["s"].as<double>();
                }
                m_sea_state.partitions.push_back(partition);

                // Frequency range and discretization. YAML carries frequencies in
                // Hz; the TDPF API uses angular frequencies.
                if (waves["frequency_min"])
                    m_sea_state.omega_min = CH_2PI * waves["frequency_min"].as<double>();
                if (waves["frequency_max"])
                    m_sea_state.omega_max = CH_2PI * waves["frequency_max"].as<double>();
                if (waves["nfrequencies"])
                    m_sea_state.n_omega = waves["nfrequencies"].as<int>();
                if (waves["discretization"]) {
                    auto discretization = waves["discretization"];
                    if (discretization["n_omega"])
                        m_sea_state.n_omega = discretization["n_omega"].as<int>();
                    if (discretization["n_theta"])
                        m_sea_state.n_theta = discretization["n_theta"].as<int>();
                }
                if (waves["seed"])
                    m_sea_state.seed = waves["seed"].as<int>();
                if (waves["eta_file"])
                    m_sea_state.eta_file_path = waves["eta_file"].as<std::string>();
                break;
            }
        }

        if (waves["depth"])
            m_sea_state.depth = waves["depth"].as<double>();
        if (waves["ramp_duration"])
            m_ramp_duration = waves["ramp_duration"].as<double>();
    }

    m_model_loaded = true;
}

// -----------------------------------------------------------------------------

std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> ChParserTdpfYAML::CreateFsiSystemTDPF(bool initialize) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserTdpfYAML] Create ChFSISystemTDPF\n" << endl;
    }

    if (!m_model_loaded) {
        cerr << "[ChParserTdpfYAML::CreateFsiSystemTDPF] Error: no YAML model file loaded." << endl;
        throw std::runtime_error("No YAML model file loaded");
    }

    if (!m_solver_loaded) {
        cerr << "[ChParserTdpfYAML::CreateFsiSystemTDPF] Error: no YAML simulation file loaded." << endl;
        throw std::runtime_error("No YAML simulation file loaded");
    }

    // Create a TDPF fluid system and associate the HDF5 file
    auto h5_file = m_file_handler.GetFilename(m_h5_file);
    if (m_verbose)
        cout << "HDF5 hydro file: " << h5_file << endl;
    m_sysTDPF = chrono_types::make_unique<fsi::tdpf::ChFsiFluidSystemTDPF>();
    m_sysTDPF->SetHydroFilename(h5_file);
    m_sysTDPF->SetGravitationalAcceleration(m_gravity);

    // Set the sea state (note that the number of bodies is set during initialization
    // of the TDPF system)
    m_sysTDPF->SetSeaState(m_sea_state);
    if (m_ramp_duration > 0)
        m_sysTDPF->SetRampDuration(m_ramp_duration);

    // Create a Chrono::FSI-TDPF system with no MBS attached
    m_sysFSI = chrono_types::make_shared<fsi::tdpf::ChFsiSystemTDPF>(nullptr, m_sysTDPF.get());
    m_sysFSI->SetVerbose(m_verbose);

    // Set a dummy time step (not needed by TDPF)
    m_sysFSI->SetStepSizeCFD(1);

    // Initialize FSI problem
    if (initialize)
        m_sysFSI->Initialize();

    return m_sysFSI;
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG

const fsi::tdpf::ChTdpfVisualizationVSG::Settings& ChParserTdpfYAML::GetTdpfVisualizationSettings() const {
    return m_visTDPF_settings;
}

std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> ChParserTdpfYAML::GetVisualizationPlugin() const {
    auto vis = chrono_types::make_shared<fsi::tdpf::ChTdpfVisualizationVSG>(m_sysFSI.get());

    vis->SetWaveMeshVisibility(true);
    vis->SetWaveMeshColormap(m_visTDPF_settings.colormap, 0.95f);
    vis->SetWaveMeshColorMode(m_visTDPF_settings.mode, m_visTDPF_settings.range);
    vis->SetWaveMeshUpdateFrequency(m_visTDPF_settings.update_fps);

    return vis;
}

#endif

// -----------------------------------------------------------------------------

void ChParserTdpfYAML::WriteOutput(int frame, double time) {
    ChParserYAML::WriteOutput(frame, time);

    //// TODO
}

// =============================================================================

ChParserTdpfYAML::WaveType ChParserTdpfYAML::ReadWaveType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "REGULAR")
        return WaveType::REGULAR;
    if (val == "IRREGULAR")
        return WaveType::IRREGULAR;
    return WaveType::NONE;
}

}  // namespace parsers
}  // namespace chrono
