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
// Class for an agent that wraps a Chrono::Vehicle tracked vehicle. The
// underlying dynamics are those of a tracked vehicle, state data consists of
// the position and orientation of the chassis and the various track components
// of the vehicle
//
// =============================================================================

#ifndef SYN_TRACKED_VEHICLE_AGENT_H
#define SYN_TRACKED_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent wrapper of a tracked vehicle, in particular holds a pointer to a ChTrackedVehicle and sends out
/// SynTrackedVehicleMessage-s to synchronize its state
class SYN_API SynTrackedVehicleAgent : public SynAgent {
  public:
    ///@brief Construct a tracked vehicle agent with the specified node_id
    ///
    ///@param vehicle the vehicle this agent is responsible for (will be null if agent's a zombie)
    ///@param filename json specification file for zombie visualization
    SynTrackedVehicleAgent(chrono::vehicle::ChTrackedVehicle* vehicle = nullptr, const std::string& filename = "");

    ///@brief Destructor.
    virtual ~SynTrackedVehicleAgent();

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
    ///@param track_shoe_vis_file the file used for track shoe visualization
    ///@param left_sprocket_vis_file the file used for left sprocket visualization
    ///@param right_sprocket_vis_file the file used for right sprocket visualization
    ///@param left_idler_vis_file the file used for left idler visualization
    ///@param right_idler_vis_file the file used for right idler visualization
    ///@param left_road_wheel_vis_file the file used for left road_wheel visualization
    ///@param right_road_wheel_vis_file the file used for right road_wheel visualization
    void SetZombieVisualizationFiles(const std::string& chassis_vis_file,
                                     const std::string& track_shoe_vis_file,
                                     const std::string& left_sprocket_vis_file,
                                     const std::string& right_sprocket_vis_file,
                                     const std::string& left_idler_vis_file,
                                     const std::string& right_idler_vis_file,
                                     const std::string& left_road_wheel_vis_file,
                                     const std::string& right_road_wheel_vis_file) {
        m_description->SetVisualizationFiles(chassis_vis_file, track_shoe_vis_file, left_sprocket_vis_file,
                                             right_sprocket_vis_file, left_idler_vis_file, right_idler_vis_file,
                                             left_road_wheel_vis_file, right_road_wheel_vis_file);
    }

    ///@brief Set the number of assembly components
    ///
    ///@param num_track_shoes number of track shoes
    ///@param num_sprockets number of sprockets
    ///@param num_idlers number of idlers
    ///@param num_road_wheels number of road_wheels
    void SetNumAssemblyComponents(int num_track_shoes, int num_sprockets, int num_idlers, int num_road_wheels) {
        m_description->SetNumAssemblyComponents(num_track_shoes, num_sprockets, num_idlers, num_road_wheels);
    }

    ///@brief Set the Agent ID
    ///
    virtual void SetKey(AgentKey agent_key) override;

    // ------------------------------------------------------------------------

  private:
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

    ///@brief Helper function for adding multiple trimeshes to a vector
    /// Will essentially instantiate a ChBodyAuxRef and set the trimesh as the asset for each
    ///
    ///@param trimesh the original trimesh
    ///@param ref_list the ref list to add the created bodies to
    ///@param system the system that each body must be added to so it can be visualized
    void AddMeshToVector(std::shared_ptr<ChVisualShapeTriangleMesh> trimesh,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

    ///@brief Helper function for adding multiple left and right trimeshes to a vector
    /// Will essentially instantiate a ChBodyAuxRef and set the trimesh as the asset for each
    ///
    ///@param left the original left trimesh
    ///@param right the original right trimesh
    ///@param ref_list the ref list to add the created bodies to
    ///@param system the system that each body must be added to so it can be visualized
    void AddMeshToVector(std::shared_ptr<ChVisualShapeTriangleMesh> left,
                         std::shared_ptr<ChVisualShapeTriangleMesh> right,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

    // ------------------------------------------------------------------------

    chrono::vehicle::ChTrackedVehicle* m_vehicle;  ///< Pointer to the ChTrackedVehicle this class wraps

    std::shared_ptr<SynTrackedVehicleStateMessage> m_state;  ///< State of the vehicle (See SynTrackedVehicleMessage)
    std::shared_ptr<SynTrackedVehicleDescriptionMessage>
        m_description;  ///< Description for zombie creation on discovery

    std::shared_ptr<ChBodyAuxRef> m_zombie_body;                   ///< agent's zombie body reference
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_track_shoe_list;  ///< vector of this agent's zombie track shoes
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_sprocket_list;    ///< vector of this agent's zombie sprockets
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_idler_list;       ///< vector of this agent's zombie idlers
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_road_wheel_list;  ///< vector of this agent's zombie road wheels
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif