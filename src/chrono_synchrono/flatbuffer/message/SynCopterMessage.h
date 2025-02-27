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
// Wraps data from a copter state message into a corresponding C++
// object.
// See also flatbuffer/fbs/Agent.fbs
//
// =============================================================================

#ifndef SYN_COPTER_MESSAGE_H
#define SYN_COPTER_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// State class that holds state information for a SynCopter
class SYN_API SynCopterStateMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    SynCopterStateMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

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
    ///@param chassis_pose copter's chassis pose
    ///@param prop_poses vector of the copter's propellers poses
    void SetState(double t, SynPose chassis_pose, std::vector<SynPose> prop_poses);

    // -------------------------------------------------------------------------------

    SynPose chassis;             ///< copter's chassis pose
    std::vector<SynPose> props;  ///< vector of copter's propellers
};

// ------------------------------------------------------------------------------------

/// Description class that holds description information for a SynCopter
class SYN_API SynCopterDescriptionMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    SynCopterDescriptionMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

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

    ///@brief Set the visualization files used for zombie visualization
    ///
    ///@param chassis_visualization_file filename for the chassis zombie visualization
    ///@param propeller_visualization_file filename for the propellers zombie visualization
    void SetVisualizationFiles(const std::string& chassis_visualization_file,
                               const std::string& propeller_visualization_file);

    ///@brief Set the total number of propellers for this copter
    ///
    ///@param number_props the total number of propellers on the copter
    void SetNumProps(int number_props);

    ///@brief Get the number of propellers for this copter
    int GetNumProps() const { return num_props; }

    // -------------------------------------------------------------------------------

    std::string chassis_vis_file = "";    ///< file name for chassis zombie visualization
    std::string propeller_vis_file = "";  ///< file name for propeller zombie visualization

    int num_props = 0;  ///< number of propellers the zombie copter has
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
