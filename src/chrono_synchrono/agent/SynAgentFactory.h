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
// Class that uses either json files or a flatbuffer description message to
// construct a SynAgent that will be of the corresponding underlying type.
// - Communication managers use the construct-from-message functionality in the
//      setup phase when they receive description messages from other ranks
// - Demos use the json functionality to construct agents
//
// =============================================================================

#ifndef SYN_AGENT_FACTORY_H
#define SYN_AGENT_FACTORY_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"


namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

// TODO: Look into ChArchive related files


/// Generates SynTerrain's from JSON files
/// Used to improve generality in Agent classes
class SYN_API SynAgentFactory {
  public:
    /// Generate the corresponding SynAgent from a description message
    static std::shared_ptr<SynAgent> CreateAgent(std::shared_ptr<SynMessage> description);
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif
