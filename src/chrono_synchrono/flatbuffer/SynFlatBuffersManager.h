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

/// Manager that handles all flatbuffer specifics
namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// Helper class that wraps the flatbuffers::FlatBufferBuilder
class SYN_API SynFlatBuffersManager {
  public:
    /// @brief Construct a flatbuffers manager with a builder starting length
    /// @param msg_length Initialize size hint for the underlying buffer
    SynFlatBuffersManager(int msg_length = 1024);

    /// Destructor
    ~SynFlatBuffersManager() {}

    /// Adds a SynMessage to the flatbuffer message buffer
    void AddMessage(SynMessage* message);

    /// Adds a SynFlatBuffers Message to the flatbuffer message buffer
    void AddMessage(flatbuffers::Offset<SynFlatBuffers::Message> message);

    /// Completes the flatbuffer message. Creates a buffer message, which stores every message
    void Finish();

    /// Completes the flatbuffer message with 4 bytes in the front of the buffer, which has the size of the byte array
    /// Creates a buffer message, which stores every message
    void FinishSizePrefixed();

    /// Reset the flatbuffer. Must be called, otherwise messages will just continue to be added to the vector (memory
    /// leak).
    void Reset();

    // Verify the buffer is not malformed.
    // Used for two reasons:
    // 1. the data could have been messed up on transport
    // 2. the data should not represent a buffer message
    bool VerifyBuffer(const uint8_t* data);

    /// When a message is received through MPI, the byte array (buffer) is stored in the class variable m_buffer. This
    /// buffer is then converted to a flatbuffer message. The ith message is then retrieved from this buffer message.
    /// @param i which message to retrieve (zero indexed)
    const SynFlatBuffers::Message* Get(int i);

    /// When a message is received through TCP (interface), the byte array (buffer) is stored in the class variable
    /// m_buffer. This buffer is then converted to a flatbuffer message. The message with the same id is then retrieved
    /// from this buffer message.
    const SynFlatBuffers::Message* GetFromRank(uint32_t rank);

    std::vector<uint8_t>& ToMessageBuffer();

    /// Gets the size of the buffer message
    int32_t GetSize() { return m_builder.GetSize(); }

    /// Gets the buffer pointer
    uint8_t* GetBufferPointer() { return m_builder.GetBufferPointer(); }

    /// Gets the underlying buffer
    std::vector<uint8_t>& GetBuffer() { return m_buffer; }

    /// Gets the FlatBufferBuilder associated with this manager
    flatbuffers::FlatBufferBuilder& GetBuilder() { return m_builder; }

    /// Gets the message vector
    std::vector<flatbuffers::Offset<SynFlatBuffers::Message>>& GetMessageVector() { return m_message_vector; }

  private:
    flatbuffers::FlatBufferBuilder m_builder;  ///< FlatBufferBuilder associated with this manager

    std::vector<flatbuffers::Offset<SynFlatBuffers::Message>> m_message_vector;  ///< vector of SynFlatBuffers messages

    std::vector<uint8_t> m_buffer;  ///< When a message is received, the byte array is stored in this buffer
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
