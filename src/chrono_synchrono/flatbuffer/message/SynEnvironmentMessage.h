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
// Authors: Yan Xiao
// =============================================================================
//
// Message class for Environment Agents. This class is only used to send the
// initial zombie description for an Environment agent. Environment agents do
// not synchronize their state in any way so don't need messages for any other
// purpose.
//
// =============================================================================

#ifndef SYN_ENVIRONMENT_MESSAGE_H
#define SYN_ENVIRONMENT_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

///@brief The environment agent state class
/// This should be inherited and stored with additional state information relevant to the environment agent
/// Should hold frequently passed data, such as synchronization information
/// For infrequently passed data, please see SynAgentDescriptionMessage
///
class SYN_API SynEnvironmentMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    SynEnvironmentMessage(unsigned int source_id, unsigned int destination_id);

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::shared_ptr<SynMAPMessage> map_message;
    std::shared_ptr<SynSPATMessage> spat_message;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif