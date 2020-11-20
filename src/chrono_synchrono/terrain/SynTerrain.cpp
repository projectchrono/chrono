// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Base class for all SynChrono terrain wrappers. Wraps normal ChTerrain
// functions but also includes functions for managing messages (at the moment
// only implemented by SCMTerrain as rigid terrain has nothing to synchronize)
//
// =============================================================================

#include "chrono_synchrono/terrain/SynTerrain.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

Document SynTerrain::ParseTerrainFileJSON(const std::string& filename) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Vehicle file not read properly in ParseTerrainFileJSON.");

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));

    std::string type = d["Type"].GetString();
    assert(type.compare("Terrain") == 0);

    return d;
}

}  // namespace synchrono
}  // namespace chrono
