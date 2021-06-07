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
// Authors: Yan Xiao, Shuo He
// =============================================================================
//
// Wraps data received from a flatbuffer SPAT message into a corresponding C++
// class
// See also flatbuffer/fbs/SPAT.fbs
//
// =============================================================================

#ifndef SYN_SPAT_MESSAGE_H
#define SYN_SPAT_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

enum class LaneColor { GREEN, YELLOW, RED };

/// Lane for the purpose of SPAT messages (i.e. something that can have its light color change)
struct IntersectionLane {
    int intersection;
    int approach;
    int lane;
    LaneColor color;

    IntersectionLane(int intersection, int approach, int lane, LaneColor color)
        : intersection(intersection), approach(approach), lane(lane), color(color) {}
};

/// SPAT Message
class SYN_API SynSPATMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    SynSPATMessage(unsigned int source_id, unsigned int destination_id);

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts a received spat flatbuffer message to a SynMessage
    ///
    ///@param state the spat flatbuffer message to convert to a SynMessage
    void ConvertSPATFromFlatBuffers(const SynFlatBuffers::SPAT::State* state);

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    ///@brief Set the color
    ///
    ///@param intersection
    ///@param approach
    ///@param lane
    ///@param color
    virtual void SetColor(int intersection, int approach, int lane, LaneColor color);

    std::vector<IntersectionLane> lanes;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
