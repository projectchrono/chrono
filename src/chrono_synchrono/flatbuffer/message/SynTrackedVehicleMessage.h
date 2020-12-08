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
// Wraps data received from a tracked vehicle flatbuffer state message, into a
// corresponding C++ object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#ifndef SYN_TRACKED_VEHICLE_MESSAGE_H
#define SYN_TRACKED_VEHICLE_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// State struct that holds state information for a SynTrackedVehicleAgent
struct SynTrackedVehicleState : SynMessageState {
    SynPose chassis;                   ///< vehicle's chassis pose
    std::vector<SynPose> track_shoes;  ///< vector of vehicle's track shoes
    std::vector<SynPose> sprockets;    ///< vector of vehicle's sprockets
    std::vector<SynPose> idlers;       ///< vector of vehicle's idlers
    std::vector<SynPose> road_wheels;  ///< vector of vehicle's road wheels

    /// Default Constructor
    SynTrackedVehicleState() : SynMessageState(0.0) {}

    /// Creates state with specified chassis and wheels
    SynTrackedVehicleState(double time,
                           SynPose chassis,
                           std::vector<SynPose> track_shoes,
                           std::vector<SynPose> sprockets,
                           std::vector<SynPose> idlers,
                           std::vector<SynPose> road_wheels)
        : SynMessageState(time),
          chassis(chassis),
          track_shoes(track_shoes),
          sprockets(sprockets),
          idlers(idlers),
          road_wheels(road_wheels) {}

    void SetState(double time,
                  SynPose chassis,
                  std::vector<SynPose> track_shoes,
                  std::vector<SynPose> sprockets,
                  std::vector<SynPose> idlers,
                  std::vector<SynPose> road_wheels) {
        this->time = time;
        this->chassis = chassis;
        this->track_shoes = track_shoes;
        this->sprockets = sprockets;
        this->idlers = idlers;
        this->road_wheels = road_wheels;
    }
};

///@brief The agent description struct
/// Describes how to visualize a zombie agent
struct SynTrackedVehicleDescription : public SynAgentDescription {
    std::string m_chassis_vis_file = "";           ///< file name for chassis zombie visualization
    std::string m_track_shoe_vis_file = "";        ///< file name for track shoe zombie visualization
    std::string m_left_sprocket_vis_file = "";     ///< file name for the left sprocket zombie visualization
    std::string m_right_sprocket_vis_file = "";    ///< file name for the right sprocket zombie visualization
    std::string m_left_idler_vis_file = "";        ///< file name for the left idler zombie visualization
    std::string m_right_idler_vis_file = "";       ///< file name for the right idler zombie visualization
    std::string m_left_road_wheel_vis_file = "";   ///< file name for the left road wheel zombie visualization
    std::string m_right_road_wheel_vis_file = "";  ///< file name for the right road wheel zombie visualization

    int m_num_track_shoes;  ///< number of track shoes the zombie vehicle has
    int m_num_sprockets;    ///< number of sprockets the zombie vehicle has
    int m_num_idlers;       ///< number of idlers the zombie vehicle has
    int m_num_road_wheels;  ///< number of road wheels the zombie vehicle has

    ///@brief Construct a new SynTrackedVehicleDescription object
    ///
    ///@param json the json specification file used to create an agent
    SynTrackedVehicleDescription(std::string json = "") : SynAgentDescription(json) {}

    void SetVisualizationFiles(std::string chassis_vis_file,
                               std::string track_shoe_vis_file,
                               std::string left_sprocket_vis_file,
                               std::string right_sprocket_vis_file,
                               std::string left_idler_vis_file,
                               std::string right_idler_vis_file,
                               std::string left_road_wheel_vis_file,
                               std::string right_road_wheel_vis_file) {
        m_chassis_vis_file = chassis_vis_file;
        m_track_shoe_vis_file = track_shoe_vis_file;
        m_left_sprocket_vis_file = left_sprocket_vis_file;
        m_right_sprocket_vis_file = right_sprocket_vis_file;
        m_left_idler_vis_file = left_idler_vis_file;
        m_right_idler_vis_file = right_idler_vis_file;
        m_left_road_wheel_vis_file = left_road_wheel_vis_file;
        m_right_road_wheel_vis_file = right_road_wheel_vis_file;
    }

    void SetNumAssemblyComponents(int num_track_shoes, int num_sprockets, int num_idlers, int num_road_wheels) {
        m_num_track_shoes = num_track_shoes;
        m_num_sprockets = num_sprockets;
        m_num_idlers = num_idlers;
        m_num_road_wheels = num_road_wheels;
    }
};

/// Wraps data from a tracked vehicle state message into a corresponding C++ object.
class SYN_API SynTrackedVehicleMessage : public SynAgentMessage {
  public:
    ///@brief Construct a new SynTrackedVehicleMessage object
    ///
    ///@param rank the rank of this message
    SynTrackedVehicleMessage(int rank,
                             std::string json = "",
                             std::shared_ptr<SynTrackedVehicleState> state = nullptr,
                             std::shared_ptr<SynTrackedVehicleDescription> description = nullptr);

    ///@brief Construct a new SynTrackedVehicleMessage object
    /// Initialize with the passed state and description
    ///
    ///@param rank the rank of this message
    SynTrackedVehicleMessage(int rank,
                             std::shared_ptr<SynTrackedVehicleState> state,
                             std::shared_ptr<SynTrackedVehicleDescription> description = nullptr);

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    ///@brief Get the SynMessageState object
    ///
    ///@return std::shared_ptr<SynMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    ///@brief Get the SynTrackedVehicleState object
    ///
    ///@return std::shared_ptr<SynTrackedVehicleState> the state associated with this message
    std::shared_ptr<SynTrackedVehicleState> GetTrackedState() { return m_state; }

    // ---------------------------------------------------------------------------------------------------------------

    ///@brief Generates and sets the agent description from a flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a SynAgentDescription object
    virtual void DescriptionFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the agent description
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) override;

    ///@brief Get the SynTrackedVehicleDescription object
    ///
    ///@return std::shared_ptr<SynTrackedVehicleDescription> the description associated with this message
    std::shared_ptr<SynTrackedVehicleDescription> GetTrackedDescription() { return m_vehicle_description; }

    // ---------------------------------------------------------------------------------------------------------------

  private:
    std::shared_ptr<SynTrackedVehicleState> m_state;                      ///< handle to the message state
    std::shared_ptr<SynTrackedVehicleDescription> m_vehicle_description;  ///< handle to the agent description
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
