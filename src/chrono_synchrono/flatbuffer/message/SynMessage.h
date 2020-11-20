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

namespace chrono {
namespace synchrono {

typedef flatbuffers::Offset<SynFlatBuffers::Message> FlatBufferMessage;

///@brief Type of each message
enum class SynMessageType {
    NONE,
    WHEELED_VEHICLE,
    TRACKED_VEHICLE,
    TRAFFIC_LIGHT,
    SCM_TERRAIN,
    ENVIRONMENT,
    MAP,
    SPAT,
    APPROACH,
    SENSOR,
    CONTROL
};

///@brief The message state struct
/// Holds information realted to the messages data
/// Should be inherited and stored with additional information relavent to new message types
/// Should hold data necessary to be passed often between ranks
struct SynMessageState {
    double time;

    SynMessageState(double time) : time(time) {}
    SynMessageState() : time(0.0) {}
};

///@brief SynMessage is the base class for all messages
/// Basically wraps the FlatBuffer methods to better handle the SynChrono message passing system
/// Uses a state struct and info struct to store data associated with the message passing
/// Will be inherited from to create new message types
class SynMessage {
  public:
    ///@brief Construct a new SynMessage object
    ///
    ///@param type the kind of message this object represents
    ///@param rank the rank of which the message was sent from
    SynMessage(int rank, SynMessageType type) : m_rank(rank), m_type(type) {}

    ///@brief Destroy the SynMessage object
    virtual ~SynMessage() {}

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) = 0;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) = 0;

    ///@brief Get the SynMessageType object
    ///
    ///@return Type the type of this message
    SynMessageType GetType() { return m_type; }

    ///@brief Get the rank from which this message originates
    ///
    ///@return unsigned int the rank from which this message originates
    unsigned int GetRank() { return m_rank; }

    ///@brief Get the SynMessageState object
    ///
    ///@return std::shared_ptr<SynMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() = 0;

  protected:
    int m_rank;             ///< rank of which sent or maintains this message
    SynMessageType m_type;  ///< type of which this message is
};

}  // namespace synchrono
}  // namespace chrono

#endif
