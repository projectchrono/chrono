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

#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserTdpfYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserTdpfYAML::ChParserTdpfYAML(const std::string& yaml_filename, bool verbose)
    : ChParserCfdYAML(verbose),
      m_gravity({0, 0, -9.8}),
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
        auto model_filename = m_script_directory + "/" + model_fname;
        auto path = filesystem::path(model_filename);
        if (!path.exists() || !path.is_file()) {
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
        auto solver_filename = m_script_directory + "/" + solver_fname;
        auto path = filesystem::path(solver_filename);
        if (!path.exists() || !path.is_file()) {
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
        m_vis.PrintInfo();
        cout << endl;
        m_output.PrintInfo();
    }

    m_loaded = true;
}

void ChParserTdpfYAML::LoadSimData(const YAML::Node& yaml) {
    // Run-time visualization (optional)
    if (yaml["visualization"]) {
#ifdef CHRONO_VSG
        auto a = yaml["visualization"];
        m_vis.render = true;

        if (a["update_fps"]) {
            m_vis.update_fps = a["update_fps"].as<double>();
        }

        if (a["color_map"]) {
            ChAssertAlways(a["color_map"]["type"]);
            m_vis.mode = ReadWaveColoringMode(a["color_map"]["type"]);
            if (a["color_map"]["map"])
                m_vis.colormap = ReadColorMapType(a["color_map"]["map"]);
            if (a["color_map"]["min"])
                m_vis.range[0] = a["color_map"]["min"].as<double>();
            if (a["color_map"]["max"])
                m_vis.range[1] = a["color_map"]["max"].as<double>();
        }

        if (a["output"]) {
            ChAssertAlways(a["output"]["output_directory"]);
            m_vis.image_dir = a["output"]["output_directory"].as<std::string>();
            if (a["output"]["save_images"])
                m_vis.write_images = a["output"]["save_images"].as<bool>();
        }
#endif
    }

    // Output (optional)
    if (yaml["output"])
        ReadOutputParams(yaml["output"]);
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

    if (model["data_path"]) {
        ChAssertAlways(model["data_path"]["type"]);
        m_data_path = ReadDataPathType(model["data_path"]["type"]);
        if (model["data_path"]["root"])
            m_rel_path = model["data_path"]["root"].as<std::string>();
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
                break;
            case WaveType::REGULAR:
                ChAssertAlways(waves["height"]);
                ChAssertAlways(waves["period"]);
                m_reg_wave_params.regular_wave_amplitude = 0.5 * waves["height"].as<double>();
                m_reg_wave_params.regular_wave_omega = CH_2PI / waves["period"].as<double>();
                if (waves["phase"])
                    m_reg_wave_params.regular_wave_phase = waves["phase"].as<double>();
                else
                    m_reg_wave_params.regular_wave_phase = 0;
                if (waves["stretching"])
                    m_reg_wave_params.wave_stretching = waves["stretching"].as<bool>();
                else
                    m_reg_wave_params.wave_stretching = true;
                break;
            case WaveType::IRREGULAR:
                //// TODO
                break;
        }
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
    auto h5_file = GetDatafilePath(m_h5_file);
    if (m_verbose)
        cout << "HDF5 hydro file: " << h5_file << endl;
    m_sysTDPF = chrono_types::make_unique<fsi::tdpf::ChFsiFluidSystemTDPF>();
    m_sysTDPF->SetHydroFilename(h5_file);
    m_sysTDPF->SetGravitationalAcceleration(m_gravity);

    // Add waves (note that the number of bodies is set during initialization of the TDPF system)
    switch (m_wave_type) {
        case WaveType::NONE:
            break;
        case WaveType::REGULAR: {
            m_sysTDPF->AddWaves(m_reg_wave_params);
            break;
        }
        case WaveType::IRREGULAR:
            m_sysTDPF->AddWaves(m_irreg_wave_params);
            break;
    }

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
std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> ChParserTdpfYAML::GetVisualizationPlugin() const {
    auto vis = chrono_types::make_shared<fsi::tdpf::ChTdpfVisualizationVSG>(m_sysFSI.get());

    vis->SetWaveMeshVisibility(true);
    vis->SetWaveMeshColormap(m_vis.colormap, 0.95f);
    vis->SetWaveMeshColorMode(m_vis.mode, m_vis.range);
    vis->SetWaveMeshUpdateFrequency(m_vis.update_fps);

    return vis;
}
#endif

// -----------------------------------------------------------------------------

void ChParserTdpfYAML::SaveOutput(int frame) {
    ChParserYAML::SaveOutput(frame);

    //// TODO
}

ChParserTdpfYAML::VisParams::VisParams()
    : render(false),
#ifdef CHRONO_VSG
      mode(fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::NONE),
#endif
      colormap(ChColormap::Type::FAST),
      range({-1, 1}),
      update_fps(30),
      write_images(false),
      image_dir(".") {
}

void ChParserTdpfYAML::VisParams::PrintInfo() {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

#ifdef CHRONO_VSGF
    cout << "run-time visualization" << endl;
    cout << "  wave color mode:       " << fsi::tdpf::ChTdpfVisualizationVSG::GetWaveMeshColorModeAsString(mode)
         << endl;
    cout << "  colormap:              " << ChColormap::GetTypeAsString(colormap) << endl;
    cout << "  color data range:      " << range << endl;
    cout << "  mesh update frequency: " << update_fps << endl;
#endif
}

// =============================================================================

ChColormap::Type ChParserTdpfYAML::ReadColorMapType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "BLUE")
        return ChColormap::Type::BLUE;
    if (val == "BROWN")
        return ChColormap::Type::BROWN;
    if (val == "COPPER")
        return ChColormap::Type::COPPER;
    if (val == "FAST")
        return ChColormap::Type::FAST;
    if (val == "INFERNO")
        return ChColormap::Type::INFERNO;
    if (val == "JET")
        return ChColormap::Type::JET;
    if (val == "KINDLMANN")
        return ChColormap::Type::KINDLMANN;
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "PLASMA")
        return ChColormap::Type::PLASMA;
    if (val == "RED_BLUE")
        return ChColormap::Type::RED_BLUE;
    return ChColormap::Type::JET;
}

ChParserTdpfYAML::WaveType ChParserTdpfYAML::ReadWaveType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "REGULAR")
        return WaveType::REGULAR;
    if (val == "IRREGULAR")
        return WaveType::IRREGULAR;
    return WaveType::NONE;
}

#ifdef CHRONO_VSG
fsi::tdpf::ChTdpfVisualizationVSG::ColorMode ChParserTdpfYAML::ReadWaveColoringMode(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "HEIGHT")
        return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::HEIGHT;
    if (val == "VELOCITY")
        return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::VELOCITY_MAG;
    return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::NONE;
}
#endif

}  // namespace parsers
}  // namespace chrono
