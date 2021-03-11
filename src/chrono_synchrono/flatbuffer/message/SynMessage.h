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
// Common code for all SynChrono messages classes that handle Flatbuffer data.
// Knows about a rank, a type and that each class should handle going to and
// from flatbuffer messages
//
// =============================================================================

#ifndef SYN_MESSAGE_H
#define SYN_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"
#include "chrono_synchrono/flatbuffer/message/SynFlatBuffers_generated.h"

#include <map>

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

typedef flatbuffers::Offset<SynFlatBuffers::Message> FlatBufferMessage;

///@brief SynMessage is the base class for all messages
/// Basically wraps the FlatBuffer methods to better handle the SynChrono message passing system
/// Will be inherited from to create new message types
class SYN_API SynMessage {
  public:
    ///@brief Destroy the SynMessage object
    virtual ~SynMessage() {}

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) = 0;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const = 0;

    ///@brief Get the key of the source of this message
    ///
    ///@return AgentKey the source key
    AgentKey GetSourceKey() { return m_source_key; }

    ///@brief Set the key of the source of this message
    ///
    ///@param source_key AgentKey the source key
    void SetSourceKey(AgentKey source_key) { m_source_key = source_key; }

    ///@brief Get the key of the destination for this message
    ///
    ///@return AgentKey the destination key
    AgentKey GetDestinationKey() { return m_destination_key; }

    SynFlatBuffers::Type GetMessageType() { return m_msg_type; }
    void SetMessageType(SynFlatBuffers::Type msg_type) { m_msg_type = msg_type; }

    double time;  ///< simulation time

  protected:
    ///@brief Constructor
    ///
    ///@param source_key the key of the source to which the message is sent from
    ///@param destination_key the key of the destination to which the message is sent to
    SynMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey())
        : time(0.0),
          m_source_key(source_key),
          m_destination_key(destination_key),
          m_msg_type(SynFlatBuffers::Type::Type_NONE) {}

    AgentKey m_source_key;       ///< key for the source which sent this message
    AgentKey m_destination_key;  ///< key for the destination of this message

    SynFlatBuffers::Type m_msg_type;  ///< Type of message that we contain
};

typedef std::vector<std::shared_ptr<SynMessage>> SynMessageList;

typedef std::map<int, std::map<int, std::shared_ptr<SynMessage>>> SynMessageMap;

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
