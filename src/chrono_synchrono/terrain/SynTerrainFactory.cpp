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
// Factory class to read a json file and attach a corresponding ChTerrain to
// the pased in ChSystem. Intended for simulation initialization.
//
// =============================================================================

#include "chrono_synchrono/terrain/SynTerrainFactory.h"

#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/terrain/SynSCMTerrain.h"

#include "chrono_synchrono/utils/SynUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

std::shared_ptr<SynTerrain> SynTerrainFactory::CreateTerrain(ChSystem* system, const std::string& filename) {
    // Create the terrain
    std::shared_ptr<SynTerrain> terrain;

    // Parse JSON file to get the terrain type
    Document d = SynTerrain::ParseTerrainFileJSON(filename);
    std::string type = d["Template"].GetString();

    if (type.compare("RigidTerrain") == 0) {
        terrain = chrono_types::make_shared<SynRigidTerrain>(system, filename);
    } else if (type.compare("SCMDeformableTerrain") == 0) {
        terrain = chrono_types::make_shared<SynSCMTerrain>(system, filename);
    } else {
        std::string message = "Terrain type \"" + type + "\" not recognized.";
        throw ChException(message);
    }

    return terrain;
}

}  // namespace synchrono
}  // namespace chrono
