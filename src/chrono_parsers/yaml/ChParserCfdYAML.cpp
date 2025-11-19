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

#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserCfdYAML::ChParserCfdYAML(bool verbose) : ChParserYAML() {}

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
