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

#ifndef SYN_TERRAIN_H
#define SYN_TERRAIN_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_terrain
/// @{

/// Base class for all terrain wrappers. Must handle messages and Advance and Synchronize the state of their terrain
class SYN_API SynTerrain {
  public:
    // Constructor
    SynTerrain() {}

    // Destructor
    ~SynTerrain() {}

    /// @brief Synchronize the state of this terrain based on info in a received SynMessage
    virtual void ProcessMessage(SynMessage* message) = 0;

    /// @brief Add any messages that this terrain would like to send out to the vector of SynMessages
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) = 0;

    /// Get the terrain
    virtual std::shared_ptr<vehicle::ChTerrain> GetTerrain() = 0;

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    /// Update the state of the terrain system at the specified time.
    virtual void Advance(double step) { GetTerrain()->Advance(step); }

    /// Advance the state of the terrain system by the specified duration.
    virtual void Synchronize(double time) { GetTerrain()->Synchronize(time); }

    /// Get the terrain height below the specified location.
    double GetHeight(const ChVector<>& loc) { return GetTerrain()->GetHeight(loc); }

    static rapidjson::Document ParseTerrainFileJSON(const std::string& filename);
};

/// @} synchrono_terrain

}  // namespace synchrono
}  // namespace chrono

#endif  // SYNTERRAIN_H
