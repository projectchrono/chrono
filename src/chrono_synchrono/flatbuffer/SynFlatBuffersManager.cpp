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

#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"

namespace chrono {
namespace synchrono {

/// Construct a flatbuffers manager with a builder starting length
SynFlatBuffersManager::SynFlatBuffersManager(int msg_length) : m_builder(msg_length) {
    // Start with a finished buffer so that GetBufferPointer doesn't return null
    FinishSizePrefixed();
}

// Adds a SynMessage to the flatbuffer message buffer.
void SynFlatBuffersManager::AddMessage(SynMessage* message) {
    m_message_vector.push_back(message->MessageFromState(m_builder));
}

// Adds a SynFlatBuffers Message to the flatbuffer message buffer.
void SynFlatBuffersManager::AddMessage(flatbuffers::Offset<SynFlatBuffers::Message> message) {
    m_message_vector.push_back(message);
}

// Completes the flatbuffer message. Creates a buffer message, of which stores every message in an vector.
void SynFlatBuffersManager::Finish() {
    auto buffer = SynFlatBuffers::CreateBufferDirect(m_builder, &m_message_vector);

    m_builder.Finish(buffer);
}

// Completes the flatbuffer message with a 4 bytes in the front of the buffer which has the size of the byte array.
// Creates a buffer message, of which stores every message in an vector.
void SynFlatBuffersManager::FinishSizePrefixed() {
    auto buffer = SynFlatBuffers::CreateBufferDirect(m_builder, &m_message_vector);

    m_builder.FinishSizePrefixed(buffer);
}

// Reset the flatbuffer. Must be called, otherwise messages will just continue to be added to the vector (memory leak).
void SynFlatBuffersManager::Reset() {
    m_builder.Clear();
    m_message_vector.clear();
}

bool SynFlatBuffersManager::VerifyBuffer(const uint8_t* data) {
    if (!data)
        return false;

    auto size = flatbuffers::GetPrefixedSize(data);

    auto verifier = flatbuffers::Verifier(data + sizeof(flatbuffers::uoffset_t), size);
    return SynFlatBuffers::VerifyBufferBuffer(verifier);
}

// When a message is received through MPI, the byte array (buffer) is stored in the class variable m_buffer. This buffer
// is then converted to a flatbuffer message. The ith message is then retrieved from this buffer message.
const SynFlatBuffers::Message* SynFlatBuffersManager::Get(int i) {
    const SynFlatBuffers::Buffer* msgs = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(m_buffer.data());
    const SynFlatBuffers::Message* message = msgs->buffer()->Get(i - 1);
    return message;
}

// When a message is received through TCP (interface), the byte array (buffer) is stored in the class variable m_buffer.
// This buffer is then converted to a flatbuffer message. The message with the same id is then retrieved from this
// buffer message.
const SynFlatBuffers::Message* SynFlatBuffersManager::GetFromRank(uint32_t rank) {
    const SynFlatBuffers::Buffer* msgs = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(m_buffer.data());
    for (auto message : (*msgs->buffer())) {
        if (message->rank() == rank)
            return message;
    }
    return nullptr;
}

std::vector<uint8_t>& SynFlatBuffersManager::ToMessageBuffer() {
    uint8_t* ptr = GetBufferPointer();
    int32_t size = GetSize();
    m_buffer = std::vector<uint8_t>(ptr, ptr + size);
    return m_buffer;
}

}  // namespace synchrono
}  // namespace chrono
