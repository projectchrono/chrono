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

#ifndef SYN_TERRAIN_FACTORY_H
#define SYN_TERRAIN_FACTORY_H

#include "chrono_synchrono/terrain/SynTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_terrain
/// @{

/// Generates SynTerrain's from JSON files
class SYN_API SynTerrainFactory {
  public:
    /// Generate the corresponding SynTerrain from a JSON specification file
    static std::shared_ptr<SynTerrain> CreateTerrain(ChSystem* system, const std::string& filename);
};

/// @} synchrono_terrain

}  // namespace synchrono
}  // namespace chrono

#endif
