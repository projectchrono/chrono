// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Lock-free single-producer single-consumer ring buffer for IPC
//
// =============================================================================

#ifndef CH_ROS_RING_BUFFER_H
#define CH_ROS_RING_BUFFER_H

#include "chrono_ros/ChApiROS.h"
#include <atomic>
#include <cstddef>
#include <cstring>

namespace chrono {
namespace ros {
namespace ipc {

/// Lock-free ring buffer for single producer, single consumer
/// Thread-safe for one writer thread and one reader thread
class CH_ROS_API RingBuffer {
public:
    /// Constructor
    /// @param buffer Pointer to memory buffer (must be power-of-2 size)  
    /// @param size Size of buffer in bytes (must be power of 2)
    RingBuffer(void* buffer, size_t size);
    
    /// Write data to the ring buffer
    /// @param data Pointer to data to write
    /// @param size Number of bytes to write
    /// @return true if successful, false if not enough space
    bool Write(const void* data, size_t size);
    
    /// Read data from the ring buffer
    /// @param data Pointer to buffer to read into
    /// @param size Number of bytes to read
    /// @return true if successful, false if not enough data available
    bool Read(void* data, size_t size);
    
    /// Peek at available data without consuming it
    /// @return Number of bytes available to read
    size_t Available() const;
    
    /// Get free space available for writing
    /// @return Number of bytes available to write
    size_t Space() const;
    
    /// Check if buffer is empty
    bool IsEmpty() const { return Available() == 0; }
    
    /// Check if buffer is full  
    bool IsFull() const { return Space() == 0; }
    
    /// Clear the buffer (not thread-safe, use only when no concurrent access)
    void Clear();

    /// Write two data chunks to the ring buffer atomically
    /// This ensures that the reader sees either both chunks or neither, preventing race conditions
    /// @param data1 Pointer to first data chunk
    /// @param size1 Size of first data chunk
    /// @param data2 Pointer to second data chunk
    /// @param size2 Size of second data chunk
    /// @return true if successful, false if not enough space
    bool Write(const void* data1, size_t size1, const void* data2, size_t size2);

private:
    char* m_data_buffer;    // Points to actual data area
    size_t m_data_size;     // Size of data area
    size_t m_mask;          // Size - 1, for efficient modulo operation
    
    // Pointers to shared memory locations for head/tail
    // Use volatile for inter-process atomicity (std::atomic doesn't work across processes)
    alignas(64) volatile size_t* m_head;  // Write position (in shared memory)
    alignas(64) volatile size_t* m_tail;  // Read position (in shared memory)
    
    // Verify size is power of 2
    static bool IsPowerOf2(size_t value);
};

}  // namespace ipc
}  // namespace ros  
}  // namespace chrono

#endif