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
// Wraps data from a wheeled vehicle state message into a corresponding C++
// object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#ifndef SYN_WHEELED_VEHICLE_MESSAGE_H
#define SYN_WHEELED_VEHICLE_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// State struct that holds state information for a SynWheeledVehicleAgent
struct SynWheeledVehicleState : SynMessageState {
    SynPose chassis;              ///< vehicle's chassis pose
    std::vector<SynPose> wheels;  ///< vector of vehicle's wheels

    /// Default Constructor
    SynWheeledVehicleState() : SynMessageState(0.0) {}

    /// Creates state with specified chassis and wheels
    SynWheeledVehicleState(double time, SynPose chassis, std::vector<SynPose> wheels)
        : SynMessageState(time), chassis(chassis), wheels(wheels) {}

    void SetState(double time, SynPose chassis, std::vector<SynPose> wheels) {
        this->time = time;
        this->chassis = chassis;
        this->wheels = wheels;
    }
};

///@brief The agent description struct
/// Describes how to visualize a zombie agent
struct SynWheeledVehicleDescription : public SynAgentDescription {
    std::string m_chassis_vis_file = "";  ///< file name for chassis zombie visualization
    std::string m_wheel_vis_file = "";    ///< file name for wheel zombie visualization
    std::string m_tire_vis_file = "";     ///< file name for tire zombie visualization

    int m_num_wheels = -1;  ///< number of wheels the zombie vehicle has

    ///@brief Construct a new SynWheeledVehicleDescription object
    ///
    ///@param json the json specification file used to create an agent
    SynWheeledVehicleDescription(std::string json = "") : SynAgentDescription(json) {}

    void SetVisualizationFiles(const std::string& chassis_vis_file,
                               const std::string& wheel_vis_file,
                               const std::string& tire_vis_file) {
        m_chassis_vis_file = chassis_vis_file;
        m_wheel_vis_file = wheel_vis_file;
        m_tire_vis_file = tire_vis_file;
    }

    void SetNumWheels(int num_wheels) { m_num_wheels = num_wheels; }
};

/// Wraps data from a wheeled vehicle state message into a corresponding C++ object.
class SYN_API SynWheeledVehicleMessage : public SynAgentMessage {
  public:
    ///@brief Construct a new SynWheeledVehicleMessage object
    ///
    ///@param rank the rank of this message
    SynWheeledVehicleMessage(int rank,
                             std::string json = "",
                             std::shared_ptr<SynWheeledVehicleState> state = nullptr,
                             std::shared_ptr<SynWheeledVehicleDescription> description = nullptr);

    ///@brief Construct a new SynWheeledVehicleMessage object
    ///       Initialize with the passed state and agent description
    ///
    ///@param rank the rank of this message
    ///@param state the state of the agent
    ///@param description the agent description
    SynWheeledVehicleMessage(int rank,
                             std::shared_ptr<SynWheeledVehicleState> state,
                             std::shared_ptr<SynWheeledVehicleDescription> description = nullptr);

    // ---------------------------------------------------------------------------------------------------------------

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

    ///@brief Get the SynWheeledVehicleState object
    ///
    ///@return std::shared_ptr<SynWheeledVehicleState> the state associated with this message
    std::shared_ptr<SynWheeledVehicleState> GetWheeledState() { return m_state; }

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

    ///@brief Get the SynWheeledVehicleDescription object
    ///
    ///@return std::shared_ptr<SynWheeledVehicleDescription> the description associated with this message
    std::shared_ptr<SynWheeledVehicleDescription> GetWheeledDescription() { return m_vehicle_description; }

  private:
    std::shared_ptr<SynWheeledVehicleState> m_state;                      ///< handle to the message state
    std::shared_ptr<SynWheeledVehicleDescription> m_vehicle_description;  ///< handle to the agent description
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
