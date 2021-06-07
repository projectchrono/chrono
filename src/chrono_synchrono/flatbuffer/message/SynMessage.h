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

    ///@brief Get the id of the source of this message
    ///
    ///@return unsigned int the source id
    unsigned int GetSourceID() const { return m_source_id; }

    ///@brief Set the id of the source of this message
    ///
    ///@param source_id unsigned int the source id
    void SetSourceID(unsigned int source_id) { m_source_id = source_id; }

    ///@brief Get the id of the destination for this message
    ///
    ///@return unsigned int the destination id
    unsigned int GetDestinationID() const { return m_destination_id; }

    SynFlatBuffers::Type GetMessageType() { return m_msg_type; }
    void SetMessageType(SynFlatBuffers::Type msg_type) { m_msg_type = msg_type; }

    double time;  ///< simulation time

  protected:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    SynMessage(unsigned int source_id, unsigned int destination_id)
        : time(0.0),
          m_source_id(source_id),
          m_destination_id(destination_id),
          m_msg_type(SynFlatBuffers::Type::Type_NONE) {}

    unsigned int m_source_id;         ///< id for the source which sent this message
    unsigned int m_destination_id;    ///< id for the destination of this message
    SynFlatBuffers::Type m_msg_type;  ///< Type of message that we contain
};

typedef std::vector<std::shared_ptr<SynMessage>> SynMessageList;

typedef std::map<int, std::map<int, std::shared_ptr<SynMessage>>> SynMessageMap;

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
