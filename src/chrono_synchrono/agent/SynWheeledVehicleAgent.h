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
// Class for an agent that wraps a Chrono::Vehicle wheeled vehicle. The
// underlying dynamics are those of a wheeled vehicle, state data consists of
// the position and orientation of the COM and the wheels of the vehicle
//
// =============================================================================

#ifndef SYN_WHEELED_VEHICLE_AGENT_H
#define SYN_WHEELED_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent wrapper of a wheeled vehicle, in particular holds a pointer to a ChWheeledVehicle and sends out
/// SynWheeledVehicleMessage-s to synchronize its state
class SYN_API SynWheeledVehicleAgent : public SynAgent {
  public:
    ///@brief Construct a wheeled vehicle agent with optionally a vehicle
    ///
    ///@param vehicle the vehicle this agent is responsible for (will be null if agent's a zombie)
    ///@param filename json specification file for zombie visualization (will query vehicle if not passed)
    SynWheeledVehicleAgent(chrono::vehicle::ChWheeledVehicle* vehicle = nullptr, const std::string& filename = "");

    ///@brief Destructor.
    virtual ~SynWheeledVehicleAgent();

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

    ///@brief Set the zombie visualization files from a JSON specification file
    ///
    ///@param filename the json specification file
    void SetZombieVisualizationFilesFromJSON(const std::string& filename);

    ///@brief Set the zombie visualization files
    ///
    ///@param chassis_vis_file the file used for chassis visualization
    ///@param wheel_vis_file the file used for wheel visualization
    ///@param tire_vis_file the file used for tire visualization
    void SetZombieVisualizationFiles(std::string chassis_vis_file,
                                     std::string wheel_vis_file,
                                     std::string tire_vis_file) {
        m_description->SetVisualizationFiles(chassis_vis_file, wheel_vis_file, tire_vis_file);
    }

    ///@brief Set the number of wheels of the underlying vehicle
    ///
    ///@param num_wheels number of wheels of the underlying vehicle
    void SetNumWheels(int num_wheels) { m_description->SetNumWheels(num_wheels); }

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

    ///@brief Create a zombie chassis body. All ChVehicles have a chassis, so this can be defined here
    ///
    ///@param filename the filename that describes the ChVisualShapeTriangleMesh that should represent the chassis
    ///@param system the system to add the body to
    std::shared_ptr<ChBodyAuxRef> CreateChassisZombieBody(const std::string& filename, ChSystem* system);

    // ------------------------------------------------------------------------

    chrono::vehicle::ChWheeledVehicle* m_vehicle;  ///< Pointer to the ChWheeledVehicle this class wraps

    std::shared_ptr<SynWheeledVehicleStateMessage> m_state;  ///< State of the vehicle (See SynWheeledVehicleMessage)
    std::shared_ptr<SynWheeledVehicleDescriptionMessage>
        m_description;  ///< Description for zombie creation on discovery

    std::shared_ptr<ChBodyAuxRef> m_zombie_body;              ///< agent's zombie body reference
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheel_list;  ///< vector of this agent's zombie wheels
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif