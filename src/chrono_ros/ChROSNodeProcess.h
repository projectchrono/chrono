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
// Launches and supervises the chrono_ros_node bridge subprocess. Locates the
// executable relative to the Chrono_ros shared library (works in both build
// and install trees), with the CMake-configured paths as fallback.
//
// =============================================================================

#ifndef CH_ROS_NODE_PROCESS_H
#define CH_ROS_NODE_PROCESS_H

#include "chrono_ros/ChApiROS.h"

#include <string>

namespace chrono {
namespace ros {

/// @addtogroup ros_core
/// @{

class CH_ROS_API ChROSNodeProcess {
  public:
    ChROSNodeProcess() = default;
    ~ChROSNodeProcess();

    ChROSNodeProcess(const ChROSNodeProcess&) = delete;
    ChROSNodeProcess& operator=(const ChROSNodeProcess&) = delete;

    /// Spawn chrono_ros_node with the given node and IPC channel names.
    /// Throws std::runtime_error if the executable cannot be found or the
    /// process cannot be created.
    void Launch(const std::string& node_name, const std::string& channel_name);

    /// True while the subprocess is alive.
    bool IsRunning() const;

    /// Terminate: SIGTERM (or Windows termination) with a grace period, then
    /// force-kill. Safe to call repeatedly. (The bridge sends the protocol
    /// Shutdown frame before calling this.)
    void Terminate();

    /// Resolved path of the executable (after Launch).
    const std::string& GetExecutablePath() const { return m_executable; }

  private:
    std::string FindExecutable() const;

    std::string m_executable;
    bool m_running = false;
#ifdef _WIN32
    void* m_process_handle = nullptr;
    void* m_thread_handle = nullptr;
#else
    int m_child_pid = -1;
#endif
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
