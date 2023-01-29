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
// See chrono_vehicle/terrain/SCMTerrain for the physics
//
// We have a square grid of points that get depressed as an object contacts
// them. SCMTerrain computes what their deformation should be (and
// what forces get applied) at each time step. Every time step this class saves
// the changes to each node, then at the SynChrono heartbeat sends those changes
// (which span several physics timesteps) to all other ranks.
//
// =============================================================================

#ifndef SYN_SCM_TERRAIN_AGENT_H
#define SYN_SCM_TERRAIN_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynSCMMessage.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

struct SCMParameters;

/// Class that wraps and synchronizes deformable terrain between Chrono Systems
class SYN_API SynSCMTerrainAgent : public SynAgent {
  public:
    ///@brief Construct a scm terrain agent with optionally a terrain
    ///
    ///@param terrain the terrain this agent is responsible for (will be null if agent's a zombie)
    SynSCMTerrainAgent(std::shared_ptr<vehicle::SCMTerrain> terrain = nullptr);

    ///@brief Destructor.
    virtual ~SynSCMTerrainAgent();

    ///@brief Initialize this agents zombie representation
    /// Bodies are added and represented in the lead agent's world.
    ///
    ///@param system the ChSystem used to initialize the zombie
    virtual void InitializeZombie(ChSystem* system) override;

    ///@brief Synchronize this agents zombie with the rest of the simulation.
    /// Updates agent based on the passed message.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent.
    ///
    ///@param message the message to process and is used to update the position of the zombie
    virtual void SynchronizeZombie(std::shared_ptr<SynMessage> message) override;

    ///@brief Update this agent
    /// Typically used to update the state representation of the agent to be distributed to other agents
    ///
    virtual void Update() override;

    ///@brief Generates messages to be sent to other nodes
    /// Will create or get messages and pass them into the referenced message vector
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherMessages(SynMessageList& messages) override;

    ///@brief Get the description messages for this agent
    /// A single agent may have multiple description messages
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherDescriptionMessages(SynMessageList& messages) override { messages.push_back(m_message); }

    ///@brief Register a new zombie
    ///
    ///@param zombie the new zombie
    virtual void RegisterZombie(std::shared_ptr<SynAgent> zombie) override;

    // -----------------------------------------------------------------------------

    /// @brief Utility to set terrain's soil parameters from the passed struct
    ///
    /// @param params Physical parameters for terrain, see SCMParameters struct
    void SetSoilParametersFromStruct(SCMParameters* params);

    ///@brief Set the underlying terrain
    ///
    void SetTerrain(std::shared_ptr<vehicle::SCMTerrain> terrain) { m_terrain = terrain; }

    ///@brief Set the Agent ID
    ///
    virtual void SetKey(AgentKey agent_key) override;

  private:
    /// There is no STL default for hashing a pair of ints, but the SCM grid is indexed with integers, so we store diffs
    /// using a map of that format.
    /// This hash function has in no way been tested for performance (it should not matter), 31 is just a decently-sized
    /// prime number to reduce bucket collisions
    struct CoordHash {
      public:
        std::size_t operator()(const ChVector2<int>& p) const { return p.x() * 31 + p.y(); }
    };

    // ------------------------------------------------------------------------

    std::shared_ptr<vehicle::SCMTerrain> m_terrain;  ///< Underlying terrain we manage

    std::shared_ptr<SynSCMMessage> m_message;                                ///< The message passed between nodes
    std::unordered_map<ChVector2<int>, double, CoordHash> m_modified_nodes;  ///< Where we store changes to our terrain
};

/// Groups SCM parameters into a struct, defines some useful defaults
/// See SCMTerrain::SetSoilParameters and SoilParametersCallback for more details on these
struct SCMParameters {
    double m_Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
    double m_Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
    double m_Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double m_Mohr_cohesion;  ///< Cohesion for shear failure [Pa]
    double m_Mohr_friction;  ///< Friction angle for shear failure [degree]
    double m_Janosi_shear;   ///< Shear parameter in Janosi-Hanamoto formula [m]
    double m_elastic_K;      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
    double m_damping_R;      ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)

    void SetParameters(double Bekker_Kphi,
                       double Bekker_Kc,
                       double Bekker_n,
                       double Mohr_cohesion,
                       double Mohr_friction,
                       double Janosi_shear,
                       double elastic_K,
                       double damping_R) {
        m_Bekker_Kphi = Bekker_Kphi;
        m_Bekker_Kc = Bekker_Kc;
        m_Bekker_n = Bekker_n;
        m_Mohr_cohesion = Mohr_cohesion;
        m_Mohr_friction = Mohr_friction;
        m_Janosi_shear = Janosi_shear;
        m_elastic_K = ChMax(elastic_K, Bekker_Kphi);
        m_damping_R = damping_R;
    }

    void InitializeParametersAsSoft() {
        m_Bekker_Kphi = 0.2e6;
        m_Bekker_Kc = 0;
        m_Bekker_n = 1.1;
        m_Mohr_cohesion = 0;
        m_Mohr_friction = 30;
        m_Janosi_shear = 0.01;
        m_elastic_K = 4e7;
        m_damping_R = 3e4;
    }

    void InitializeParametersAsMid() {
        m_Bekker_Kphi = 2e6;
        m_Bekker_Kc = 0;
        m_Bekker_n = 1.1;
        m_Mohr_cohesion = 0;
        m_Mohr_friction = 30;
        m_Janosi_shear = 0.01;
        m_elastic_K = 2e8;
        m_damping_R = 3e4;
    }

    void InitializeParametersAsHard() {
        m_Bekker_Kphi = 5301e3;
        m_Bekker_Kc = 102e3;
        m_Bekker_n = 0.793;
        m_Mohr_cohesion = 1.3e3;
        m_Mohr_friction = 31.1;
        m_Janosi_shear = 1.2e-2;
        m_elastic_K = 4e8;
        m_damping_R = 3e4;
    }
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif