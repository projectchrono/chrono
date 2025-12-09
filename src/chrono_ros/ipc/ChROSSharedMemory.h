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
// Cross-platform shared memory implementation for IPC
//
// =============================================================================

#ifndef CH_ROS_SHARED_MEMORY_H
#define CH_ROS_SHARED_MEMORY_H

#include "chrono_ros/ChApiROS.h"
#include <string>
#include <memory>

namespace chrono {
namespace ros {
namespace ipc {

/// Cross-platform shared memory wrapper
class CH_ROS_API SharedMemory {
public:
    /// Create or open shared memory segment
    /// @param name Unique name for the shared memory segment
    /// @param size Size in bytes (only used when creating new segment)
    /// @param create_new If true, create new segment; if false, open existing
    SharedMemory(const std::string& name, size_t size, bool create_new = true);
    
    /// Destructor
    ~SharedMemory();
    
    /// Get pointer to mapped memory
    void* GetPtr() const { return m_ptr; }
    
    /// Get size of mapped memory
    size_t GetSize() const { return m_size; }
    
    /// Check if memory is successfully mapped
    bool IsValid() const { return m_ptr != nullptr; }
    
    /// Unmap and close shared memory
    void Close();

private:
    std::string m_name;
    size_t m_size;
    void* m_ptr;
    bool m_created;
    int m_signal_index;
    
#ifdef _WIN32
    void* m_handle;  // HANDLE
#else
    int m_fd;
#endif
    
    // Non-copyable
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif