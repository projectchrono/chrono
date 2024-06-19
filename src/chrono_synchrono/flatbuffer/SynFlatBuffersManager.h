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
// Helper class that wraps the flatbuffers::FlatBufferBuilder
// Maintains a vector of flatbuffer offsets for outgoing messages and a vector
// of bytes for the incoming message buffer
//
// =============================================================================

#ifndef SYN_FLATBUFFERS_MANAGER_H
#define SYN_FLATBUFFERS_MANAGER_H

#include <vector>

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynFlatBuffers_generated.h"

namespace chrono {
namespace synchrono {

typedef std::vector<flatbuffers::Offset<SynFlatBuffers::Message>> SynFlatBufferMessageList;
class SynAgentMessage;

/// @addtogroup synchrono_flatbuffer
/// @{

/// Helper class that wraps the flatbuffers::FlatBufferBuilder
class SYN_API SynFlatBuffersManager {
  public:
    /// @brief Construct a flatbuffers manager with a builder starting length
    /// @param msg_length Initialize size hint for the underlying buffer
    SynFlatBuffersManager(int msg_length = 1024);

    ///@brief Destructor
    ///
    ~SynFlatBuffersManager() {}

    ///@brief Process a data buffer with the assumption it is a SynFlatBuffers::Buffer message
    ///
    ///@param data the data to process
    ///@param messages reference to message list to store the parsed messages
    void ProcessBuffer(std::vector<uint8_t>& data, SynMessageList& messages);

    ///@brief Adds a SynMessage to the flatbuffer message buffer. Will call MessageFromState automatically
    ///
    ///@param message the SynMessage to add
    void AddMessage(std::shared_ptr<SynMessage> message);

    ///@brief Completes the flatbuffer message.
    /// Creates a buffer message, of which stores every message
    /// If size_prefixed, Completes the flatbuffer message with a 4 bytes in the front of the buffer, which has the size
    /// of the byte array
    ///
    ///@param size_prefixed whether the buffer should be finished size prefixed
    void Finish(bool size_prefixed = true);

    ///@brief Reset the flatbuffer. Must be called, otherwise messages will just continue to be added to the vector
    ///
    void Reset();

    ///@brief Creates a vector from the flatbuffer data
    ///
    ///@return std::vector<uint8_t> the message buffer
    std::vector<uint8_t> ToMessageBuffer();

    // -------------------------------------------------------------------------------------------

    ///@brief Gets the FlatBufferBuilder associated with this manager
    ///
    ///@return flatbuffers::FlatBufferBuilder& the underlying builder
    flatbuffers::FlatBufferBuilder& GetBuilder() { return m_builder; }

    ///@brief Get the SynMessageList that is used to store received messages
    ///
    ///@return SynMessageList the message list
    SynMessageList& GetMessages() { return m_messages; }

    ///@brief Get the FlatBufferMessageList that is used to store outgoing messages
    ///
    ///@return SynFlatBufferMessageList& the flatbuffer message list
    SynFlatBufferMessageList& GetFlatBufferMessageList() { return m_flatbuffer_messages; }

    /// ---------------------
    /// Convenience Functions
    /// ---------------------

    ///@brief Get the size of the underlying finished buffer
    ///
    ///@return int32_t the size of the buffer
    int32_t GetSize() { return m_builder.GetSize(); }

    ///@brief Get the buffer pointer from the builder
    ///
    ///@return uint8_t* the buffer pointer
    uint8_t* GetBufferPointer() { return m_builder.GetBufferPointer(); }

  private:
    flatbuffers::FlatBufferBuilder m_builder;  ///< FlatBufferBuilder associated with this manager

    SynMessageList m_messages;                       ///< vector of SynMessages
    SynFlatBufferMessageList m_flatbuffer_messages;  ///< vector of SynFlatBuffers messages
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
