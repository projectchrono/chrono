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

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// State class that holds state information for a SynTrackedVehicleAgent
class SynTrackedVehicleStateMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    SynTrackedVehicleStateMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

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
    ///@param chassis_pose vehicle's chassis pose
    ///@param track_shoe_poses vehicle's track shoe poses
    ///@param sprocket_poses vehicle's sprocket poses
    ///@param idler_poses vehicle's idler poses
    ///@param road_wheels vehicle's road wheel poses
    void SetState(double t,
                  SynPose chassis_pose,
                  std::vector<SynPose> track_shoe_poses,
                  std::vector<SynPose> sprocket_poses,
                  std::vector<SynPose> idler_poses,
                  std::vector<SynPose> road_wheel_poses);

    // -------------------------------------------------------------------------------

    SynPose chassis;                   ///< vehicle's chassis pose
    std::vector<SynPose> track_shoes;  ///< vector of vehicle's track shoes
    std::vector<SynPose> sprockets;    ///< vector of vehicle's sprockets
    std::vector<SynPose> idlers;       ///< vector of vehicle's idlers
    std::vector<SynPose> road_wheels;  ///< vector of vehicle's road wheels
};

/// Description class that holds description information for a SynTrackedVehicle
class SynTrackedVehicleDescriptionMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    ///@param json the json specification file used to create an agent
    SynTrackedVehicleDescriptionMessage(AgentKey source_key = AgentKey(),
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

    ///@brief Set the visualization files
    ///
    ///@param chassis_visualization_file file name for chassis zombie visualization
    ///@param track_shoe_visualization_file file name for track shoe zombie visualization
    ///@param left_sprocket_visualization_file file name for the left sprocket zombie visualization
    ///@param right_sprocket_visualization_file file name for the right sprocket zombie visualization
    ///@param left_idler_visualization_file file name for the left idler zombie visualization
    ///@param right_idler_visualization_file file name for the right idler zombie visualization
    ///@param left_road_wheel_visualization_file file name for the left road wheel zombie visualization
    ///@param right_road_wheel_visualization_file file name for the right road wheel zombie visualization
    void SetVisualizationFiles(const std::string& chassis_visualization_file,
                               const std::string& track_shoe_visualization_file,
                               const std::string& left_sprocket_visualization_file,
                               const std::string& right_sprocket_visualization_file,
                               const std::string& left_idler_visualization_file,
                               const std::string& right_idler_visualization_file,
                               const std::string& left_road_wheel_visualization_file,
                               const std::string& right_road_wheel_visualization_file);

    ///@brief Set the number of each assembly component
    ///
    ///@param number_track_shoes number of track shoes the zombie vehicle has
    ///@param number_sprockets number of sprockets the zombie vehicle has
    ///@param number_idlers number of idlers the zombie vehicle has
    ///@param number_road_wheels number of road wheels the zombie vehicle has
    void SetNumAssemblyComponents(int number_track_shoes,   //
                                  int number_sprockets,     //
                                  int number_idlers,        //
                                  int number_road_wheels);  //

    // -------------------------------------------------------------------------------

    std::string json = "";  ///< the json specification file that is used to create an agent

    std::string chassis_vis_file = "";           ///< file name for chassis zombie visualization
    std::string track_shoe_vis_file = "";        ///< file name for track shoe zombie visualization
    std::string left_sprocket_vis_file = "";     ///< file name for the left sprocket zombie visualization
    std::string right_sprocket_vis_file = "";    ///< file name for the right sprocket zombie visualization
    std::string left_idler_vis_file = "";        ///< file name for the left idler zombie visualization
    std::string right_idler_vis_file = "";       ///< file name for the right idler zombie visualization
    std::string left_road_wheel_vis_file = "";   ///< file name for the left road wheel zombie visualization
    std::string right_road_wheel_vis_file = "";  ///< file name for the right road wheel zombie visualization

    int num_track_shoes = 0;  ///< number of track shoes the zombie vehicle has
    int num_sprockets = 0;    ///< number of sprockets the zombie vehicle has
    int num_idlers = 0;       ///< number of idlers the zombie vehicle has
    int num_road_wheels = 0;  ///< number of road wheels the zombie vehicle has
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
