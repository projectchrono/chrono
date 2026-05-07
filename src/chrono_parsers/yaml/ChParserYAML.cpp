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

#include "chrono/physics/ChSystem.h"

#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserYAML::ChParserYAML()
    : m_name("model"),
      m_verbose(false),
      m_use_degrees(true),
      m_output_dir(".") {}

// -----------------------------------------------------------------------------

ChParserYAML::YamlFileType ChParserYAML::ReadYamlFileType(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }
    YAML::Node yaml = YAML::LoadFile(yaml_filename);
    ChAssertAlways(yaml["type"]);
    return ReadYamlFileType(yaml["type"]);
}

ChParserYAML::YamlFileType ChParserYAML::ReadYamlFileType(const YAML::Node& a) {
    auto type = ChToUpper(a.as<std::string>());
    if (type == "MBS")
        return YamlFileType::MBS;
    if (type == "SPH")
        return YamlFileType::SPH;
    if (type == "TDPF")
        return YamlFileType::TDPF;
    if (type == "FSI")
        return YamlFileType::FSI;
    if (type == "VEHICLE")
        return YamlFileType::VEHICLE;
    return YamlFileType::UNKNOWN;
}

// -----------------------------------------------------------------------------

ChParserYAML::OutputParameters::OutputParameters()
    : type(ChOutput::Type::NONE), mode(ChOutput::Mode::FRAMES), fps(100) {}

void ChParserYAML::OutputParameters::PrintInfo() {
    if (type == ChOutput::Type::NONE) {
        cout << "no output" << endl;
        return;
    }

    cout << "output" << endl;
    cout << "  type:                 " << ChOutput::GetOutputTypeAsString(type) << endl;
    cout << "  mode:                 " << ChOutput::GetOutputModeAsString(mode) << endl;
    cout << "  output FPS:           " << fps << endl;
}

bool ChParserYAML::Output() const {
    return m_output.type != ChOutput::Type::NONE;
}

void ChParserYAML::SetOutputDir(const std::string& out_dir) {
    auto p = filesystem::path(out_dir);
    if (!p.exists() || !p.is_directory()) {
        std::cerr << "The specified path " << out_dir << " is not a valid directory." << std::endl;
        throw std::runtime_error("Invalid directory");
    }

    m_output_dir = out_dir;

    if (m_verbose) {
        auto filename = m_output_dir + "/" + m_name;
        switch (m_output.type) {
            case ChOutput::Type::ASCII:
                filename += ".txt";
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                filename += ".h5";
                break;
#else
                return;
#endif
        }
        cout << "Output file: " << filename << endl;
    }
}

void ChParserYAML::ReadOutputParams(const YAML::Node& a) {
    ChAssertAlways(a["type"]);
    m_output.type = ReadOutputType(a["type"]);
#ifndef CHRONO_HAS_HDF5
    if (m_output.type == ChOutput::Type::HDF5) {
        std::cerr << "HDF5 output support not available.\nOutput disabled." << std::endl;
        m_output.type = ChOutput::Type::NONE;
        return;
    }
#endif

    if (a["mode"])
        m_output.mode = ReadOutputMode(a["mode"]);
    if (a["fps"])
        m_output.fps = a["fps"].as<double>();
}

void ChParserYAML::SaveOutput(int frame) {
    if (m_output.type == ChOutput::Type::NONE)
        return;

    // Create the output DB if needed
    if (!m_output_db) {
        auto filename = m_output_dir + "/" + m_name;
        switch (m_output.type) {
            case ChOutput::Type::ASCII:
                filename += ".txt";
                m_output_db = chrono_types::make_shared<ChOutputASCII>(filename);
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                filename += ".h5";
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(filename, m_output.mode);
                break;
#else
                return;
#endif
        }

        m_output_db->Initialize();
    }
}

}  // end namespace parsers
}  // namespace chrono
