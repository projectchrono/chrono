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
// Class that acts as the primary wrapper for an underlying Chrono object. This
// base class provides the functionality to synchronize between different agents
// on different ranks. Message generation and processing is done through a agent.
// Dynamic updates of the underlying Chrono objects are also performed by each
// agent on the object they actually wrap.
//
// =============================================================================

#include "chrono_synchrono/agent/SynAgent.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynAgent::SynAgent(unsigned int rank, SynAgentType type, ChSystem* system)
    : m_rank(rank),
      m_type(type),
      m_step_size(1e-3),
      m_system(system),
      m_vis_manager(chrono_types::make_shared<SynVisualizationManager>()) {}

Document SynAgent::ParseAgentFileJSON(const std::string& filename) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Agent file not read properly in ParseAgentFileJSON.");

    // Read top-level data
    assert(d.HasMember("Name"));
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string suptype = d["Template"].GetString();
    assert(type.compare("Agent") == 0);

    return d;
}

void SynAgent::SetVisualizationManager(std::shared_ptr<SynVisualizationManager> manager) {
    m_vis_manager = manager;
}

}  // namespace synchrono
}  // namespace chrono
