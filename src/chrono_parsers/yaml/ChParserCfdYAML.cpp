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

#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserCfdYAML::ChParserCfdYAML(bool verbose) : m_verbose(verbose), m_output_dir("") {}

std::string ChParserCfdYAML::ToUpper(std::string in) {
    std::transform(in.begin(), in.end(), in.begin(), ::toupper);
    return in;
}

void ChParserCfdYAML::CheckVersion(const YAML::Node& a) {
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

ChParserCfdYAML::FluidSystemType ChParserCfdYAML::ReadFluidSystemType(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    if (yaml["fluid_dynamics_solver"]) {
        auto sysCFD_type = yaml["fluid_dynamics_solver"].as<std::string>();

        if (sysCFD_type == "SPH")
            return FluidSystemType::SPH;
        if (sysCFD_type == "BEM")
            return FluidSystemType::BEM;

        cerr << "Error: unknown fluid system type '" << sysCFD_type << "'." << endl;
        throw std::runtime_error("Unknown fluid system type");
    } else {
        cerr << "Error: file '" << yaml_filename << "' is not a fluid simulation YAML file." << endl;
        throw std::runtime_error("Invalid file");
    }
}

}  // namespace parsers
}  // namespace chrono
