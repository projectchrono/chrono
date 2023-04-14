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
// Authors: Jay Taves
// =============================================================================
//
// Class that wraps data contained in a message about Soil Contact Model (SCM)
// Deformable terrain. See chrono_vehicle/terrain/SCMTerrain.* for
// more details.
//
// =============================================================================

#ifndef SYN_SCM_MESSAGE_H
#define SYN_SCM_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// SCM Message
class SYN_API SynSCMMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_key the id of the source to which the message is sent from
    ///@param destination_key the id of the destination to which the message is sent to
    SynSCMMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<vehicle::SCMTerrain::NodeLevel> modified_nodes;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
