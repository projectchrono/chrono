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

#include "chrono_synchrono/flatbuffer/message/SynMessageFactory.h"

namespace chrono {
namespace synchrono {

/// Construct a flatbuffers manager with a builder starting length
SynFlatBuffersManager::SynFlatBuffersManager(int msg_length) : m_builder(msg_length) {
    // Start with a finished buffer so that GetBufferPointer doesn't return null
    Finish();
}

void SynFlatBuffersManager::ProcessBuffer(std::vector<uint8_t>& data, SynMessageList& messages) {
    auto buffer = flatbuffers::GetSizePrefixedRoot<SynFlatBuffers::Buffer>(data.data());
    for (auto message : (*buffer->buffer())) {
        auto msg = SynMessageFactory::GenerateMessage(message);
        messages.push_back(msg);
    }
}

// Adds a SynMessage to the flatbuffer message buffer.
void SynFlatBuffersManager::AddMessage(std::shared_ptr<SynMessage> message) {
    m_flatbuffer_messages.push_back(message->ConvertToFlatBuffers(m_builder));
}

// Completes the flatbuffer message. Creates a buffer message, of which stores every message in an vector.
// If size prefixed, completes the flatbuffer message with a 4 bytes in the front of the buffer which has the size of
// the byte array. Creates a buffer message, of which stores every message in an vector.
void SynFlatBuffersManager::Finish(bool size_prefixed) {
    auto buffer = SynFlatBuffers::CreateBufferDirect(m_builder, &m_flatbuffer_messages);

    if (size_prefixed)
        m_builder.FinishSizePrefixed(buffer);
    else
        m_builder.Finish(buffer);
}

// Reset the flatbuffer. Must be called, otherwise messages will just continue to be added to the vector (memory leak).
void SynFlatBuffersManager::Reset() {
    m_builder.Clear();
    m_flatbuffer_messages.clear();
}

std::vector<uint8_t> SynFlatBuffersManager::ToMessageBuffer() {
    uint8_t* ptr = GetBufferPointer();
    int32_t size = GetSize();
    return std::vector<uint8_t>(ptr, ptr + size);
}

}  // namespace synchrono
}  // namespace chrono
