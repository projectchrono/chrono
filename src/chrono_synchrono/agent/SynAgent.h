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

#ifndef SYN_AGENT_H
#define SYN_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/brain/SynBrain.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"
#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"
#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent types
enum class SynAgentType { VEHICLE, ENVIRONMENT };

/// Base class for SynChrono agents.
class SYN_API SynAgent {
  public:
    ///@brief Construct a agent with the specified rank and type
    ///
    ///@param rank the rank this agent is on
    ///@param type the type of the underlying agent
    ///@param system optional argument to set the ChSystem of the agent through the constructor
    SynAgent(unsigned int rank, SynAgentType type, ChSystem* system = nullptr);

    ///@brief Destructor.
    virtual ~SynAgent() {}

    ///@brief Advance the state of this agent until agent time syncs with passed time.
    ///
    ///@param time_of_next_sync time at which this agent should stop advancing
    virtual void Advance(double time_of_next_sync) = 0;

    ///@brief Initialize this agents zombie representation
    /// In order to be visualized, the lead agent on this rank should pass in a ChSystem.
    /// With the passed system, bodies can be added and represented in the lead agent's world.
    ///
    ///@param system the ChSystem used to initialize the zombie
    virtual void InitializeZombie(ChSystem* system = 0) = 0;

    ///@brief Synchronoize this agents zombie with the rest of the simulation.
    /// Updates agent based on the passed message.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent
    ///
    ///@param message the message to process and is used to update the position of the zombie
    virtual void SynchronizeZombie(SynMessage* message) = 0;

    ///@brief Get the state of this agent
    ///
    ///@return std::shared_ptr<SynMessageState>
    virtual std::shared_ptr<SynMessageState> GetState() = 0;

    ///@brief Get the this agent's message to be pass to other ranks
    ///
    ///@return std::shared_ptr<SynMessageState>
    virtual std::shared_ptr<SynAgentMessage> GetMessage() = 0;

    ///@brief Generates messages to be sent to other ranks
    /// Will create or get messages and pass them into the referenced message vector
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) = 0;

    ///@brief Process an incoming message. Will pass the message directly to this agent's brain.
    ///
    ///@param msg the received message to be processed
    virtual void ProcessMessage(SynMessage* msg) { m_brain->ProcessMessage(msg); }

    // ------------------------------------------------------------------------

    /// Set the VisualizationManager for this agent
    void SetVisualizationManager(std::shared_ptr<SynVisualizationManager> vis_manager);

    /// Get/Set this agent's brain
    std::shared_ptr<SynBrain> GetBrain() { return m_brain; }
    void SetBrain(std::shared_ptr<SynBrain> brain) { m_brain = brain; }

    /// Get/Set this agent's rank
    unsigned int GetRank() { return m_rank; }
    void SetRank(unsigned int rank) { m_rank = rank; }

    /// Get/Set the Chrono system associated with this agent
    ChSystem* GetSystem() { return m_system; }
    void SetSystem(ChSystem* system) { m_system = system; }

    /// Get/Set this agent's step size
    double GetStepSize() { return m_step_size; }
    void SetStepSize(double step_size) { m_step_size = step_size; }

    /// Get the type of this agent
    SynAgentType GetType() const { return m_type; }

    ///@brief Parse an agent json specification file
    ///
    ///@param filename the json specification file
    ///@return rapidjson::Document
    static rapidjson::Document ParseAgentFileJSON(const std::string& filename);

  protected:
    unsigned int m_rank;  ///< agent rank
    SynAgentType m_type;  ///< agent type

    double m_step_size;  ///< Step size of the underlying agent

    std::shared_ptr<SynBrain> m_brain;                       ///< handle to this agent's brain
    std::shared_ptr<SynVisualizationManager> m_vis_manager;  ///< handle to this agent's visualization manager

    ChSystem* m_system;  ///< pointer to the Chrono system
};

/// Vector of handles to agents.
typedef std::vector<std::shared_ptr<SynAgent>> SynAgentList;

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_AGENT_H
