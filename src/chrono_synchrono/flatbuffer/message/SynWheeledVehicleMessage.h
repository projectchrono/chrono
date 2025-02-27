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

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// State class that holds state information for a SynWheeledVehicle
class SYN_API SynWheeledVehicleStateMessage : public SynMessage {
  public:
    SynWheeledVehicleStateMessage();

    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    SynWheeledVehicleStateMessage(AgentKey source_key, AgentKey destination_key);

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    // -------------------------------------------------------------------------------

    ///@brief Set the state variables
    ///
    ///@param t simulation time
    ///@param chassis vehicle's chassis pose
    ///@param wheels vector of the vehicle's wheel poses
    void SetState(double t, SynPose chassis_pose, std::vector<SynPose> wheel_poses);

    // -------------------------------------------------------------------------------

    SynPose chassis;              ///< vehicle's chassis pose
    std::vector<SynPose> wheels;  ///< vector of vehicle's wheels
};

// ------------------------------------------------------------------------------------

/// Description class that holds description information for a SynWheeledVehicle
class SYN_API SynWheeledVehicleDescriptionMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    ///@param json the json specification file used to create an agent
    SynWheeledVehicleDescriptionMessage(AgentKey source_key = AgentKey(),
                                        AgentKey destination_key = AgentKey(),
                                        const std::string& json = "");

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    // -------------------------------------------------------------------------------

    ///@brief Set the visualization files from a JSON specification file
    ///
    ///@param filename the json specification file
    void SetZombieVisualizationFilesFromJSON(const std::string& filename);

    ///@brief Set the visualization files used for zombie visualization
    ///
    ///@param chassis_visualization_file filename for the chassis zombie visualization
    ///@param wheel_visualization_file filename for the wheel zombie visualization
    ///@param tire_visualization_file filename for the tire zombie visualization
    void SetVisualizationFiles(const std::string& chassis_visualization_file,
                               const std::string& wheel_visualization_file,
                               const std::string& tire_visualization_file);

    ///@brief Set the total number of wheels for this vehicle
    ///
    ///@param number_wheels the total number of wheels on the vehicle
    void SetNumWheels(int number_wheels);

    // -------------------------------------------------------------------------------

    std::string json = "";  ///< the json specification file that is used to create an agent

    std::string chassis_vis_file = "";  ///< file name for chassis zombie visualization
    std::string wheel_vis_file = "";    ///< file name for wheel zombie visualization
    std::string tire_vis_file = "";     ///< file name for tire zombie visualization

    int num_wheels = 0;  ///< number of wheels the zombie vehicle has
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
