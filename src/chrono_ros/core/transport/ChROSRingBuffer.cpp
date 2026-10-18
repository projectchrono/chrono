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

#include "chrono_ros/core/transport/ChROSRingBuffer.h"

#include <cstring>
#include <stdexcept>

namespace chrono {
namespace ros {
namespace core {

RingBuffer::RingBuffer(std::atomic<uint64_t>* head, std::atomic<uint64_t>* tail, uint8_t* data, size_t capacity)
    : m_head(head), m_tail(tail), m_data(data), m_capacity(capacity), m_mask(capacity - 1) {
    if (head == nullptr || tail == nullptr || data == nullptr) {
        throw std::invalid_argument("RingBuffer: null shared-memory pointer");
    }
    if (!IsPowerOf2(capacity)) {
        throw std::invalid_argument("RingBuffer: capacity must be a power of 2");
    }
}

void RingBuffer::CopyIn(uint64_t at, const void* src, size_t size) {
    const size_t pos = static_cast<size_t>(at) & m_mask;
    const size_t first = std::min(size, m_capacity - pos);
    std::memcpy(m_data + pos, src, first);
    if (first < size) {  // wrap
        std::memcpy(m_data, static_cast<const uint8_t*>(src) + first, size - first);
    }
}

void RingBuffer::CopyOut(uint64_t from, void* dst, size_t size) const {
    const size_t pos = static_cast<size_t>(from) & m_mask;
    const size_t first = std::min(size, m_capacity - pos);
    std::memcpy(dst, m_data + pos, first);
    if (first < size) {  // wrap
        std::memcpy(static_cast<uint8_t*>(dst) + first, m_data, size - first);
    }
}

bool RingBuffer::Write(const void* data1, size_t size1, const void* data2, size_t size2) {
    const size_t total = size1 + size2;
    if (total == 0) {
        return true;
    }
    if ((size1 > 0 && data1 == nullptr) || (size2 > 0 && data2 == nullptr)) {
        throw std::invalid_argument("RingBuffer::Write: null chunk with non-zero size");
    }

    const uint64_t head = m_head->load(std::memory_order_relaxed);  // we are the only writer
    const uint64_t tail = m_tail->load(std::memory_order_acquire);
    if (total > m_capacity - static_cast<size_t>(head - tail)) {
        return false;
    }

    if (size1 > 0) {
        CopyIn(head, data1, size1);
    }
    if (size2 > 0) {
        CopyIn(head + size1, data2, size2);
    }

    // Publish both chunks at once; release pairs with the consumer's acquire.
    m_head->store(head + total, std::memory_order_release);
    return true;
}

bool RingBuffer::Peek(void* out, size_t size) const {
    if (size == 0) {
        return true;
    }
    const uint64_t tail = m_tail->load(std::memory_order_relaxed);  // we are the only reader
    const uint64_t head = m_head->load(std::memory_order_acquire);
    if (size > static_cast<size_t>(head - tail)) {
        return false;
    }
    CopyOut(tail, out, size);
    return true;
}

bool RingBuffer::Read(void* out, size_t size) {
    if (!Peek(out, size)) {
        return false;
    }
    Consume(size);
    return true;
}

void RingBuffer::Consume(size_t size) {
    if (size == 0) {
        return;
    }
    const uint64_t tail = m_tail->load(std::memory_order_relaxed);
    const uint64_t head = m_head->load(std::memory_order_acquire);
    if (size > static_cast<size_t>(head - tail)) {
        throw std::logic_error("RingBuffer::Consume: consuming more than is available");
    }
    // Release so the producer's acquire of tail observes our reads as done.
    m_tail->store(tail + size, std::memory_order_release);
}

size_t RingBuffer::Available() const {
    const uint64_t tail = m_tail->load(std::memory_order_relaxed);
    const uint64_t head = m_head->load(std::memory_order_acquire);
    return static_cast<size_t>(head - tail);
}

size_t RingBuffer::Space() const {
    const uint64_t head = m_head->load(std::memory_order_relaxed);
    const uint64_t tail = m_tail->load(std::memory_order_acquire);
    return m_capacity - static_cast<size_t>(head - tail);
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
