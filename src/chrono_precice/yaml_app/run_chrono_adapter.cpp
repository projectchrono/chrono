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
//
// Application for running a Chrono preCICE adapter specified through YAML files.
//
// =============================================================================

#include <filesystem>

#include "chrono/ChConfig.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

bool ParseArgs(int argc,
               char** argv,
               std::string& yaml_filename,
               std::string& precice_filename,
               std::string& out_dir,
               bool& disable_verbose,
               bool& disable_output,
               bool& disable_vis);
bool RunMBS(const std::string& yaml_filename, const std::string& precice_filename, const std::string& out_dir, bool disable_verbose, bool disable_output, bool& disable_vis);

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Process command line arguments
    bool disable_verbose = false;
    bool disable_output = false;
    bool disable_vis = false;
    std::string yaml_filename = "";
    std::string precice_filename = "";
    std::string out_dir = GetChronoOutputPath() + "YAML_PRECICE_CHRONO/";
    if (!ParseArgs(argc, argv, yaml_filename, precice_filename, out_dir, disable_verbose, disable_output, disable_vis))
        return 1;

    cout << endl;
    cout << "Chrono YAML specification file: " << yaml_filename << endl;
    cout << "preCICE configuration file:     " << precice_filename << endl;
    cout << "Verbose?          " << (disable_verbose ? "no" : "yes") << endl;
    cout << "Visualization?    " << (disable_vis ? "no" : "yes") << endl;
    cout << "Output?           " << (disable_output ? "no" : "yes") << endl;
    if (!disable_output)
        cout << "Output directory: " << out_dir << endl;

    // Peek in file, read type, and call appropriate function for processing the YAML file
    auto type = ChParserYAML::ReadYamlFileType(yaml_filename);
    switch (type) {
        case ChParserYAML::YamlFileType::MBS:
            RunMBS(yaml_filename, precice_filename, out_dir, disable_verbose, disable_output, disable_vis);
            break;
        case ChParserYAML::YamlFileType::VEHICLE:
            cerr << "Chrono::Vehicle preCICE parser not yet available." << endl;
            break;
        case ChParserYAML::YamlFileType::FSI:
            cerr << "Chrono::FSI preCICE parser not yet available." << endl;
            break;

        default:
            cerr << "\nError: Unsupported YAML file type.\n" << endl;
    }

    return 0;
}

// -----------------------------------------------------------------------------

bool ParseArgs(int argc,
               char** argv,
               std::string& yaml_filename,
               std::string& precice_filename,
               std::string& out_dir,
               bool& disable_verbose,
               bool& disable_output,
               bool& disable_vis) {
    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "s,sim_file", "preCICE simulation specification file (YAML format)");
    cli.AddOption<std::string>("", "p,precice_file", "preCICE configuration file (XML format)");
    cli.AddOption<std::string>("", "o,out_dir", "Output directory", out_dir);
    cli.AddOption<bool>("", "quiet", "Disable terminal output");
    cli.AddOption<bool>("", "no_output", "Disable output");
    cli.AddOption<bool>("", "no_visualization", "Disable run-time visualization");

    if (!cli.Parse(argc, argv, true))
        return false;

    try {
        yaml_filename = cli.Get("sim_file").as<std::string>();
    } catch (std::domain_error&) {
        cerr << "\nError: Missing YAML specification file." << endl;
        cli.Help();
        return false;
    }

    try {
        precice_filename = cli.Get("precice_file").as<std::string>();
    } catch (std::domain_error&) {
        cerr << "\nError: Missing XML configuration file." << endl;
        cli.Help();
        return false;
    }

    disable_verbose = cli.GetAsType<bool>("quiet");
    disable_output = cli.GetAsType<bool>("no_output");
    disable_vis = cli.GetAsType<bool>("no_visualization");

    out_dir = cli.Get("out_dir").as<std::string>();

    return true;
}

// -----------------------------------------------------------------------------

bool RunMBS(const std::string& yaml_filename, const std::string& precice_filename, const std::string& out_dir, bool disable_verbose, bool disable_output, bool& disable_vis) {
    // Create the preCICE Chrono MBS participant
    ChPreciceAdapterMbs participant(yaml_filename, !disable_verbose);

    // Create and set output directories
    const auto& model_name = participant.GetModelName();
    if (!disable_output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        auto mbs_out_dir = out_dir + "/" + model_name;
        if (!CreateOutputDirectory(std::filesystem::path(mbs_out_dir))) {
            std::cout << "Error creating directory " << mbs_out_dir << std::endl;
            return false;
        }
        participant.SetOutputDir(mbs_out_dir);
    }

    // Enable/disable run-time visualization and simulation output
    participant.EnableVisualization(!disable_vis);
    participant.EnableOutput(!disable_output);

    // Register participant with preCICE
    participant.RegisterParticipant(precice_filename);

    // Initialize, run, and finalize participant simulation
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();

    return true;
}
