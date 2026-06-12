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
//
// Cross-platform named shared-memory segment (POSIX shm / Windows file
// mapping). The creating side owns the name: it unlinks on destruction and -
// on POSIX - registers the segment with a SIGINT/SIGTERM handler so /dev/shm
// is not leaked when the simulation is interrupted.
//
// =============================================================================

#ifndef CH_ROS_CORE_SHARED_MEMORY_H
#define CH_ROS_CORE_SHARED_MEMORY_H

#include <cstddef>
#include <string>

namespace chrono {
namespace ros {
namespace core {

class SharedMemory {
  public:
    /// Create a new segment (fails if a segment with this name exists) or open
    /// an existing one (size is then taken from the segment).
    /// Throws std::runtime_error with the OS error message on failure.
    /// @param name segment name; portable charset [A-Za-z0-9_-], no slashes
    SharedMemory(const std::string& name, size_t size, bool create_new = true);
    ~SharedMemory();

    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    void* Get() const { return m_ptr; }
    size_t GetSize() const { return m_size; }
    const std::string& GetName() const { return m_name; }
    bool IsCreator() const { return m_created; }

  private:
    void Close();

    std::string m_name;
    size_t m_size;
    void* m_ptr;
    bool m_created;
    int m_signal_index;

#ifdef _WIN32
    void* m_handle;
#else
    int m_fd;
#endif
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
