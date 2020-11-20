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

#include "chrono_synchrono/terrain/SynSCMTerrain.h"

#include "chrono/core/ChLog.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynSCMTerrain::SynSCMTerrain(ChSystem* system, const std::string& filename) {
    GetLog() << "SCM Terrain initialization from json file not yet supported. Initialization call did nothing."
             << "\n";
}

void SynSCMTerrain::Advance(double step) {
    GetTerrain()->Advance(step);

    // Use [] because we want to update if it is there but otherwise insert
    for (auto h : m_scm_terrain->GetModifiedNodes())
        m_modified_nodes[h.first] = h.second;
}

void SynSCMTerrain::GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) {
    std::vector<SCMDeformableTerrain::NodeLevel> modified_nodes;

    for (auto v : m_modified_nodes)
        modified_nodes.push_back(std::make_pair(v.first, v.second));

    auto state = chrono_types::make_shared<SynSCMTerrainState>(m_system->GetChTime(), modified_nodes);

    messages.push_back(new SynSCMMessage(rank, state));

    // After we send this message and get updates from others (ProcessMessage) our terrain state should be the same as
    // everyone else's. So we only keep track of what we change after that point
    m_modified_nodes.clear();
}

void SynSCMTerrain::ProcessMessage(SynMessage* msg) {
    auto state = ((SynSCMMessage*)msg)->GetSCMState();

    m_scm_terrain->SetModifiedNodes(state->modified_nodes);
}

void SynSCMTerrain::SetSoilParametersFromStruct(SCMParameters* params) {
    m_scm_terrain->SetSoilParameters(params->m_Bekker_Kphi, params->m_Bekker_Kc, params->m_Bekker_n,
                                     params->m_Mohr_cohesion, params->m_Mohr_friction, params->m_Janosi_shear,
                                     params->m_elastic_K, params->m_damping_R);
}

}  // namespace synchrono
}  // namespace chrono
