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
// Authors: Simone Benatti
// =============================================================================
//
// Class for an agent that wraps a Chrono::Copter hexacopter. The
// underlying dynamics are those of a copter robot, state data consists of
// the position and orientation of the COM and the propellers of the copter
//
// =============================================================================

#ifndef SYN_COPTER_AGENT_H
#define SYN_COPTER_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynCopterMessage.h"

#include "chrono_models/robot/copters/Copter.h"

#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent wrapper of a copter model, sends out SynCopterMessage-s to synchronize its state
class SYN_API SynCopterAgent : public SynAgent {
  public:
    ///@brief Construct a copter agent with an optional copter object
    ///
    ///@param copter the copter this agent is responsible for (will be null if the agent is a zombie)
    SynCopterAgent(chrono::copter::Copter<6>* copter = nullptr);

    ///@brief Destructor.
    virtual ~SynCopterAgent();

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
    virtual void GatherMessages(SynMessageList& messages) override { messages.push_back(m_state); }

    ///@brief Get the description messages for this agent
    /// A single agent may have multiple description messages
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherDescriptionMessages(SynMessageList& messages) override { messages.push_back(m_description); }

    // ------------------------------------------------------------------------

    ///@brief Set the zombie visualization files
    ///
    ///@param chassis_vis_file the file used for chassis visualization
    ///@param propeller_vis_file the file used for propeller visualization
    void SetZombieVisualizationFiles(std::string chassis_vis_file, std::string propeller_vis_file) {
        m_description->SetVisualizationFiles(chassis_vis_file, propeller_vis_file);
    }

    ///@brief Set the number of propellers of the underlying copter
    ///
    ///@param num_props number of propellers of the underlying copter
    void SetNumProps(int num_props) { m_description->SetNumProps(num_props); }

    ///@brief Set the Agent ID
    ///
    virtual void SetKey(AgentKey agent_key) override;

    // ------------------------------------------------------------------------

  protected:
    ///@brief Helper method used to create a ChVisualShapeTriangleMesh to be used on as a zombie body
    ///
    ///@param filename the file to generate a ChVisualShapeTriangleMesh from
    ///@return std::shared_ptr<ChVisualShapeTriangleMesh>
    std::shared_ptr<ChVisualShapeTriangleMesh> CreateMeshZombieComponent(const std::string& filename);

    ///@brief Create a zombie chassis body.
    ///
    ///@param filename the filename that describes the ChVisualShapeTriangleMesh that should represent the chassis
    ///@param system the system to add the body to
    std::shared_ptr<ChBody> CreateChassisZombieBody(const std::string& filename, ChSystem* system);

    // ------------------------------------------------------------------------

    chrono::copter::Copter<6>* m_copter;  ///< Pointer to the ChCopter this class wraps

    std::shared_ptr<SynCopterStateMessage> m_state;              ///< State of the copter (See SynCopterMessage)
    std::shared_ptr<SynCopterDescriptionMessage> m_description;  ///< Description for zombie creation on discovery

    std::shared_ptr<ChBody> m_zombie_body;             ///< agent's zombie body
    std::vector<std::shared_ptr<ChBody>> m_prop_list;  ///< vector of this agent's zombie propellers
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif