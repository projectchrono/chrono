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

#include "chrono_parsers/yaml/ChParserCfdYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserCfdYAML::ChParserCfdYAML(bool verbose) : ChParserYAML() {}

ChParserCfdYAML::FluidSystemType ChParserCfdYAML::ReadFluidSystemType(const std::string& yaml_filename) {
    auto path = std::filesystem::path(yaml_filename);
    if (!exists(path) || !is_regular_file(path)) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    if (yaml["type"]) {
        auto sysCFD_type = yaml["type"].as<std::string>();
        if (sysCFD_type == "SPH")
            return FluidSystemType::SPH;
        if (sysCFD_type == "TDPF")
            return FluidSystemType::TDPF;
    }

    cerr << "Error: file '" << yaml_filename << "' is not a known fluid YAML specification." << endl;
    throw std::runtime_error("Invalid fluid YAML specification file");
}

}  // namespace parsers
}  // namespace chrono
