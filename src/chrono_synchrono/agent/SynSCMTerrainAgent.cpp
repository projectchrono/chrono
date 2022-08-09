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
// Authors: Jay Taves
// =============================================================================
//
// Class that wraps and synchronizes deformable terrain between Chrono Systems
// See chrono_vehicle/terrain/SCMDeformableTerrain for the physics
//
// We have a square grid of points that get depressed as an object contacts
// them. SCMDeformableTerrain computes what their deformation should be (and
// what forces get applied) at each time step. Every time step this class saves
// the changes to each node, then at the SynChrono heartbeat sends those changes
// (which span several physics timesteps) to all other ranks.
//
// =============================================================================

#include "chrono_synchrono/agent/SynSCMTerrainAgent.h"

#include "chrono/assets/ChVisualShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynSCMTerrainAgent::SynSCMTerrainAgent(std::shared_ptr<vehicle::SCMDeformableTerrain> terrain)
    : SynAgent(), m_terrain(terrain) {
    m_message = chrono_types::make_shared<SynSCMMessage>();
}

SynSCMTerrainAgent::~SynSCMTerrainAgent() {}

void SynSCMTerrainAgent::InitializeZombie(ChSystem* system) {}

void SynSCMTerrainAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynSCMMessage>(message))
        if (m_terrain)
            m_terrain->SetModifiedNodes(state->modified_nodes);
}

void SynSCMTerrainAgent::Update() {
    // Use [] because we want to update if it is there but otherwise insert
    auto modded = m_terrain->GetModifiedNodes();
    for (const auto& h : modded)
        m_modified_nodes[h.first] = h.second;

    m_message->modified_nodes.clear();
    m_message->modified_nodes.reserve(m_modified_nodes.size());
    for (const auto& v : m_modified_nodes)
        m_message->modified_nodes.push_back(std::make_pair(v.first, v.second));
}

void SynSCMTerrainAgent::GatherMessages(SynMessageList& messages) {
    messages.push_back(m_message);

    // After we send this message and get updates from others (ProcessMessage) our terrain state should be the same as
    // everyone else's. So we only keep track of what we change after that point
    m_modified_nodes.clear();
}

void SynSCMTerrainAgent::RegisterZombie(std::shared_ptr<SynAgent> zombie) {
    if (auto terrain_zombie = std::dynamic_pointer_cast<SynSCMTerrainAgent>(zombie))
        if (m_terrain)
            terrain_zombie->SetTerrain(m_terrain);
}

// ------------------------------------------------------------------------

void SynSCMTerrainAgent::SetSoilParametersFromStruct(SCMParameters* params) {
    m_terrain->SetSoilParameters(params->m_Bekker_Kphi, params->m_Bekker_Kc, params->m_Bekker_n,
                                 params->m_Mohr_cohesion, params->m_Mohr_friction, params->m_Janosi_shear,
                                 params->m_elastic_K, params->m_damping_R);
}

void SynSCMTerrainAgent::SetKey(AgentKey agent_key) {
    m_message->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

}  // namespace synchrono
}  // namespace chrono