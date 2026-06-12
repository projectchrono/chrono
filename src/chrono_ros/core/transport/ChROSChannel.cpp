// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================

#include "chrono_ros/core/transport/ChROSChannel.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <new>

namespace chrono {
namespace ros {
namespace core {

namespace {

/// Shared-memory control block at offset 0. The creator placement-news it;
/// the opener accesses it through a cast, which is the established pattern
/// for lock-free (and therefore address-free, [atomics.lockfree]) atomics in
/// shared memory.
struct ChannelControlBlock {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint64_t sim_to_node_capacity;
    uint64_t node_to_sim_capacity;
    std::atomic<uint32_t> node_attached;

    // Each cursor on its own cache line to avoid false sharing between the
    // producer and consumer of a ring.
    alignas(64) std::atomic<uint64_t> s2n_head;
    alignas(64) std::atomic<uint64_t> s2n_tail;
    alignas(64) std::atomic<uint64_t> n2s_head;
    alignas(64) std::atomic<uint64_t> n2s_tail;
};

constexpr uint32_t kChannelMagic = 0xC4050C4Au;
constexpr size_t kDataOffset = 4096;  // page-aligned start of the ring data regions

static_assert(sizeof(ChannelControlBlock) <= kDataOffset, "control block must fit before the data regions");

size_t RoundUpPowerOf2(size_t v) {
    if (RingBuffer::IsPowerOf2(v)) {
        return v;
    }
    size_t result = 1;
    while (result < v) {
        result <<= 1;
    }
    return result;
}

constexpr size_t kMinCapacity = 64 * 1024;

}  // namespace

std::unique_ptr<Channel> Channel::Create(const std::string& name, const Config& config) {
    const size_t s2n = RoundUpPowerOf2(std::max(config.sim_to_node_capacity, kMinCapacity));
    const size_t n2s = RoundUpPowerOf2(std::max(config.node_to_sim_capacity, kMinCapacity));

    auto shm = std::make_unique<SharedMemory>(name, kDataOffset + s2n + n2s, /*create_new=*/true);

    auto* block = new (shm->Get()) ChannelControlBlock{};
    block->version = kProtocolVersion;
    block->sim_to_node_capacity = s2n;
    block->node_to_sim_capacity = n2s;
    // Publish the magic last: an Open racing with Create sees either an
    // incomplete block (magic == 0, rejected) or a fully initialized one.
    std::atomic_thread_fence(std::memory_order_release);
    block->magic = kChannelMagic;

    return std::unique_ptr<Channel>(new Channel(std::move(shm), /*is_sim_side=*/true, /*created=*/true));
}

std::unique_ptr<Channel> Channel::Open(const std::string& name) {
    // Size is taken from the existing segment.
    auto shm = std::make_unique<SharedMemory>(name, 0, /*create_new=*/false);

    if (shm->GetSize() < kDataOffset) {
        throw ChannelError("segment '" + name + "' is too small to be a chrono_ros channel");
    }
    auto* block = static_cast<ChannelControlBlock*>(shm->Get());
    if (block->magic != kChannelMagic) {
        throw ChannelError("segment '" + name + "' is not an initialized chrono_ros channel");
    }
    if (block->version != kProtocolVersion) {
        throw ChannelError("protocol version mismatch on '" + name + "': segment is v" +
                           std::to_string(block->version) + ", this binary speaks v" +
                           std::to_string(kProtocolVersion) +
                           " (simulation and chrono_ros_node must come from the same build)");
    }
    const size_t expected = kDataOffset + block->sim_to_node_capacity + block->node_to_sim_capacity;
    if (shm->GetSize() < expected) {
        throw ChannelError("segment '" + name + "' truncated: " + std::to_string(shm->GetSize()) + " bytes, expected " +
                           std::to_string(expected));
    }

    auto channel = std::unique_ptr<Channel>(new Channel(std::move(shm), /*is_sim_side=*/false, /*created=*/false));
    block->node_attached.store(1, std::memory_order_release);
    return channel;
}

Channel::Channel(std::unique_ptr<SharedMemory> shm, bool is_sim_side, bool /*created*/) : m_shm(std::move(shm)) {
    auto* block = static_cast<ChannelControlBlock*>(m_shm->Get());
    auto* base = static_cast<uint8_t*>(m_shm->Get());

    uint8_t* s2n_data = base + kDataOffset;
    uint8_t* n2s_data = s2n_data + block->sim_to_node_capacity;

    auto s2n = std::make_unique<RingBuffer>(&block->s2n_head, &block->s2n_tail, s2n_data,
                                            static_cast<size_t>(block->sim_to_node_capacity));
    auto n2s = std::make_unique<RingBuffer>(&block->n2s_head, &block->n2s_tail, n2s_data,
                                            static_cast<size_t>(block->node_to_sim_capacity));

    if (is_sim_side) {
        m_send = std::move(s2n);
        m_recv = std::move(n2s);
    } else {
        m_send = std::move(n2s);
        m_recv = std::move(s2n);
    }
}

size_t Channel::MaxSendPayload() const {
    return m_send->Capacity() - sizeof(FrameHeader);
}

size_t Channel::SendSpace() const {
    return m_send->Space();
}

bool Channel::IsPeerAttached() const {
    const auto* block = static_cast<const ChannelControlBlock*>(m_shm->Get());
    return block->node_attached.load(std::memory_order_acquire) != 0;
}

bool Channel::SendFrame(FrameKind kind,
                        uint32_t channel_id,
                        uint64_t sim_time_ns,
                        const void* payload,
                        size_t payload_size) {
    if (payload_size > MaxSendPayload()) {
        throw ChannelError("frame of " + std::to_string(payload_size) + " payload bytes (" + FrameKindName(kind) +
                           ", channel " + std::to_string(channel_id) + ") exceeds the ring capacity of " +
                           std::to_string(m_send->Capacity()) +
                           " bytes; increase the channel capacity configuration for this direction");
    }
    if (payload_size > UINT32_MAX) {
        throw ChannelError("frame payload of " + std::to_string(payload_size) + " bytes exceeds the protocol limit");
    }

    FrameHeader header;
    header.kind = static_cast<uint16_t>(kind);
    header.channel_id = channel_id;
    header.payload_size = static_cast<uint32_t>(payload_size);
    header.sim_time_ns = sim_time_ns;

    return m_send->Write(&header, sizeof(header), payload, payload_size);
}

bool Channel::ReceiveFrame(FrameHeader& header, std::vector<uint8_t>& payload) {
    if (!m_recv->Peek(&header, sizeof(header))) {
        return false;
    }
    if (header.magic != kFrameMagic || header.version != kProtocolVersion || !IsValidFrameKind(header.kind)) {
        throw ChannelError("corrupted frame header on '" + GetName() +
                           "' (magic/version/kind mismatch); the channel state is unrecoverable");
    }
    if (header.payload_size > m_recv->Capacity()) {
        throw ChannelError("corrupted frame header on '" + GetName() + "': payload size " +
                           std::to_string(header.payload_size) + " exceeds ring capacity");
    }
    // The producer publishes header+payload with a single atomic head update,
    // so once the header is visible the payload is too.
    if (m_recv->Available() < sizeof(header) + header.payload_size) {
        throw ChannelError("torn frame on '" + GetName() + "': header visible without its payload");
    }
    m_recv->Consume(sizeof(header));
    payload.resize(header.payload_size);
    if (header.payload_size > 0 && !m_recv->Read(payload.data(), header.payload_size)) {
        throw ChannelError("failed to read frame payload on '" + GetName() + "'");
    }
    return true;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
