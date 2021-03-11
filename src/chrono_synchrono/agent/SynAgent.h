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
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

#include "chrono/physics/ChSystem.h"

#include <functional>

/// TODO: Create a class with utility functions
#define SynAgentID uint32_t

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Base class for SynChrono agents.
class SYN_API SynAgent {
  public:
    ///@brief Construct a agent with the specified node_id
    ///
    ///@param aid the agent id associated with this agent (defaults to uninitialized)
    SynAgent(SynAgentID aid = 0);

    ///@brief Destructor.
    virtual ~SynAgent();

    ///@brief Initialize this agents zombie representation
    /// Bodies are added and represented in the lead agent's world.
    ///
    ///@param system the ChSystem used to initialize the zombie
    virtual void InitializeZombie(ChSystem* system) = 0;

    ///@brief Synchronize this agents zombie with the rest of the simulation.
    /// Updates agent based on the passed message.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent.
    ///
    ///@param message the message to process and is used to update the position of the zombie
    virtual void SynchronizeZombie(std::shared_ptr<SynMessage> message) = 0;

    ///@brief Update this agent
    /// Typically used to update the state representation of the agent to be distributed to other agents
    ///
    virtual void Update() = 0;

    ///@brief Generates messages to be sent to other nodes
    /// Should create or get messages and pass them into the referenced message vector
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherMessages(SynMessageList& messages) = 0;

    ///@brief Get the description messages for this agent
    /// A single agent may have multiple description messages
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherDescriptionMessages(SynMessageList& messages) = 0;

    ///@brief Process an incoming message.
    ///
    ///@param msg the received message to be processed
    virtual void ProcessMessage(std::shared_ptr<SynMessage> msg);

    ///@brief Register a new zombie
    ///
    ///@param zombie the new zombie
    virtual void RegisterZombie(std::shared_ptr<SynAgent> zombie) {}

    // -------------------------------------------------------------------------

    void SetProcessMessageCallback(std::function<void(std::shared_ptr<SynMessage>)> callback);

    // -------------------------------------------------------------------------

    SynAgentID GetID() const { return m_aid; }
    virtual void SetID(SynAgentID aid) { m_aid = aid; }

  protected:
    SynAgentID m_aid;

    std::function<void(std::shared_ptr<SynMessage>)> m_process_message_callback;
};

/// Vector of handles to agents.
typedef std::vector<std::shared_ptr<SynAgent>> SynAgentList;

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif