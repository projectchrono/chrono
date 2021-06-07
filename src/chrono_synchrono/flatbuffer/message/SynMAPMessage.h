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
// Store the Map information in the simulation. Currently only used for traffic
// light. A Map has Intersections which have approaches which have lanes
//
// =============================================================================

#ifndef SYN_MAP_MESSAGE_H
#define SYN_MAP_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// Contains some number of approaches - see ApproachMessage
struct Intersection {
    std::vector<std::shared_ptr<SynApproachMessage>> approaches;
};

/// Store the Map information in the simulation. Currently only used for traffic light. A Map has Intersections which
/// have approaches which have lanes
class SYN_API SynMAPMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    SynMAPMessage(unsigned int source_id, unsigned int destination_id);

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    ///@brief Add the lane to environment.
    ///
    ///@param intersection
    ///@param approach
    ///@param lane
    ///@return int The lane's position in that approach
    virtual unsigned AddLane(int intersection, int approach, ApproachLane lane);

    std::vector<Intersection> intersections;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
