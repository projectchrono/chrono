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
// Authors: Yan Xiao, Jay Taves
// =============================================================================
//
// Class for Approach Messages, those contains data on approaches, which are a
// set of lanes going towards/away from an intersection in the same direction
//
// =============================================================================

#ifndef SYN_APPROACH_MESSAGE_H
#define SYN_APPROACH_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// @brief Lane that is grouped into an approach with other ApproachLanes
struct ApproachLane {
    double width;                           ///< define the width of the lane. Control points to the side is width/2.
    std::vector<ChVector<>> controlPoints;  ///< The points define the center of the lane.

    ApproachLane(double width, std::vector<ChVector<>> controlPoints) : width(width), controlPoints(controlPoints) {}
    ApproachLane(const SynFlatBuffers::Approach::Lane* lane);
};

///@brief Approach message
class SYN_API SynApproachMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    SynApproachMessage(unsigned int source_id, unsigned int destination_id);

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts a received spat flatbuffer message to a SynMessage
    ///
    ///@param message the spat flatbuffer message to convert to a SynMessage
    void ConvertSPATFromFlatBuffers(const SynFlatBuffers::Approach::State* message);

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<ApproachLane> lanes;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
