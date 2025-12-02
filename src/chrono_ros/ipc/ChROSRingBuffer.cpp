// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Lock-free single-producer single-consumer ring buffer for IPC
//
// =============================================================================

#include "chrono_ros/ipc/ChROSRingBuffer.h"
#include <stdexcept>
#include <algorithm>

namespace chrono {
namespace ros {
namespace ipc {

bool RingBuffer::IsPowerOf2(size_t value) {
    return value != 0 && (value & (value - 1)) == 0;
}

RingBuffer::RingBuffer(void* buffer, size_t size) {
    if (buffer == nullptr) {
        throw std::invalid_argument("Buffer pointer cannot be null");
    }
    
    // Reserve space for head/tail pointers at the beginning
    size_t metadata_size = 2 * sizeof(std::atomic<size_t>);
    if (size < metadata_size + 64) {  // Need at least some space for data
        throw std::invalid_argument("Buffer too small for ring buffer metadata");
    }
    
    m_data_size = size - metadata_size;
    if (!IsPowerOf2(m_data_size)) {
        // Find the largest power of 2 that fits
        m_data_size = 1;
        while (m_data_size * 2 <= (size - metadata_size)) {
            m_data_size *= 2;
        }
    }
    
    // Set up pointers to shared memory locations
    char* base_ptr = static_cast<char*>(buffer);
    m_head = reinterpret_cast<std::atomic<size_t>*>(base_ptr);
    m_tail = reinterpret_cast<std::atomic<size_t>*>(base_ptr + sizeof(std::atomic<size_t>));
    m_data_buffer = base_ptr + metadata_size;
    
    m_mask = m_data_size - 1;
}

bool RingBuffer::Write(const void* data, size_t size) {
    if (size == 0 || data == nullptr) {
        return true;  // Nothing to write
    }
    
    size_t head = m_head->load(std::memory_order_relaxed);
    size_t tail = m_tail->load(std::memory_order_acquire);
    
    // Check if we have enough space
    size_t available_space = m_data_size - (head - tail);
    if (size > available_space) {
        return false;  // Not enough space
    }
    
    // Calculate where to start writing
    size_t head_pos = head & m_mask;
    
    // Handle wrap-around case
    if (head_pos + size <= m_data_size) {
        // No wrap-around, simple copy
        std::memcpy(m_data_buffer + head_pos, data, size);
    } else {
        // Wrap-around, split the write
        size_t first_chunk = m_data_size - head_pos;
        size_t second_chunk = size - first_chunk;
        
        std::memcpy(m_data_buffer + head_pos, data, first_chunk);
        std::memcpy(m_data_buffer, 
                    static_cast<const char*>(data) + first_chunk, second_chunk);
    }
    
    // Update head position (release semantics ensures data is written before head update)
    m_head->store(head + size, std::memory_order_release);
    
    return true;
}

bool RingBuffer::Read(void* data, size_t size) {
    if (size == 0 || data == nullptr) {
        return true;  // Nothing to read
    }
    
    size_t head = m_head->load(std::memory_order_acquire);
    size_t tail = m_tail->load(std::memory_order_relaxed);
    
    // Check if we have enough data
    size_t available_data = head - tail;
    if (size > available_data) {
        return false;  // Not enough data
    }
    
    // Calculate where to start reading
    size_t tail_pos = tail & m_mask;
    
    // Handle wrap-around case
    if (tail_pos + size <= m_data_size) {
        // No wrap-around, simple copy
        std::memcpy(data, m_data_buffer + tail_pos, size);
    } else {
        // Wrap-around, split the read
        size_t first_chunk = m_data_size - tail_pos;
        size_t second_chunk = size - first_chunk;
        
        std::memcpy(data, m_data_buffer + tail_pos, first_chunk);
        std::memcpy(static_cast<char*>(data) + first_chunk,
                    m_data_buffer, second_chunk);
    }
    
    // Update tail position (release semantics for consistency)
    m_tail->store(tail + size, std::memory_order_release);
    
    return true;
}

size_t RingBuffer::Available() const {
    size_t head = m_head->load(std::memory_order_acquire);
    size_t tail = m_tail->load(std::memory_order_relaxed);
    return head - tail;
}

size_t RingBuffer::Space() const {
    size_t head = m_head->load(std::memory_order_relaxed);
    size_t tail = m_tail->load(std::memory_order_acquire);
    return m_data_size - (head - tail);
}

void RingBuffer::Clear() {
    m_head->store(0, std::memory_order_relaxed);
    m_tail->store(0, std::memory_order_relaxed);
}

}  // namespace ipc
}  // namespace ros
}  // namespace chrono