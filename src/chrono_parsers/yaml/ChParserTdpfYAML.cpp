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

ChParserTdpfYAML::ChParserTdpfYAML(const std::string& yaml_model_filename,
                                   const std::string& yaml_sim_filename,
                                   bool verbose)
    : ChParserCfdYAML(verbose), m_gravity({0, 0, -9.8}), m_sim_loaded(false), m_model_loaded(false) {
    SetVerbose(verbose);
    LoadModelFile(yaml_model_filename);
    LoadSimulationFile(yaml_sim_filename);
}

ChParserTdpfYAML::~ChParserTdpfYAML() {}

// -----------------------------------------------------------------------------

void ChParserTdpfYAML::LoadSimulationFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check that the file is an TDPF specification
    ChAssertAlways(yaml["fluid_dynamics_solver"]);
    if (ToUpper(yaml["fluid_dynamics_solver"].as<std::string>()) != "TDPF") {
        cerr << "Error: file '" << yaml_filename << "' is not a TDPF specification file." << endl;
        throw std::runtime_error("Not a TDPF specification file");
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserTdpfYAML] Loading Chrono::TDPF simulation specification from: " << yaml_filename << "\n"
             << endl;
    }

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check a simulation object exists
    ChAssertAlways(yaml["simulation"]);
    auto sim = yaml["simulation"];

    // Run-time visualization (optional)
    if (sim["visualization"]) {
#ifdef CHRONO_VSG
        m_vis.render = true;
        auto a = sim["visualization"];

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
    if (sim["output"]) {
        ChAssertAlways(sim["output"]["type"]);
        m_output.type = ReadOutputType(sim["output"]["type"]);
        if (sim["output"]["mode"])
            m_output.mode = ReadOutputMode(sim["output"]["mode"]);
        if (sim["output"]["fps"])
            m_output.fps = sim["output"]["fps"].as<double>();
        if (sim["output"]["output_directory"])
            m_output.dir = sim["output"]["output_directory"].as<std::string>();
    }

    if (m_verbose) {
        m_vis.PrintInfo();
        cout << endl;
        m_output.PrintInfo();
    }

    m_sim_loaded = true;
}

void ChParserTdpfYAML::LoadModelFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    m_script_directory = path.parent_path().str();

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

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
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserTdpfYAML] Loading Chrono::TDPF model specification from: '" << yaml_filename << "'\n"
             << endl;
        cout << "model name: '" << m_name << "'" << endl;
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
                m_reg_wave_params.regular_wave_amplitude_ = 0.5 * waves["height"].as<double>();
                m_reg_wave_params.regular_wave_omega_ = CH_2PI / waves["period"].as<double>();
                if (waves["phase"])
                    m_reg_wave_params.regular_wave_phase_ = waves["phase"].as<double>();
                else
                    m_reg_wave_params.regular_wave_phase_ = 0;
                if (waves["stretching"])
                    m_reg_wave_params.wave_stretching_ = waves["stretching"].as<bool>();
                else
                    m_reg_wave_params.wave_stretching_ = true;
                break;
            case WaveType::IRREGULAR:
                //// TODO
                break;
        }
    }

    if (m_verbose) {
        cout << endl;
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

    if (!m_sim_loaded) {
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
            m_sysTDPF->AddWaves(NoWaveParams());
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
      mode(fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::NONE),
      colormap(ChColormap::Type::FAST),
      range({-1, 1}),
      update_fps(30),
      write_images(false),
      image_dir(".") {}

void ChParserTdpfYAML::VisParams::PrintInfo() {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

    cout << "run-time visualization" << endl;
    cout << "  wave color mode:       " << fsi::tdpf::ChTdpfVisualizationVSG::GetWaveMeshColorModeAsString(mode)
         << endl;
    cout << "  colormap:              " << ChColormap::GetTypeAsString(colormap) << endl;
    cout << "  color data range:      " << range << endl;
    cout << "  mesh update frequency: " << update_fps << endl;
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

fsi::tdpf::ChTdpfVisualizationVSG::ColorMode ChParserTdpfYAML::ReadWaveColoringMode(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "HEIGHT")
        return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::HEIGHT;
    if (val == "VELOCITY")
        return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::VELOCITY_MAG;
    return fsi::tdpf::ChTdpfVisualizationVSG::ColorMode::NONE;
}

}  // namespace parsers
}  // namespace chrono
