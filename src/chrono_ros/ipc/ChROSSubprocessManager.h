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
// Subprocess manager for launching and controlling ROS node process
//
// =============================================================================

#ifndef CH_ROS_SUBPROCESS_MANAGER_H
#define CH_ROS_SUBPROCESS_MANAGER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include <string>
#include <memory>
#include <vector>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <sys/types.h>
#endif

namespace chrono {
namespace ros {
namespace ipc {

/// Manages the lifecycle of the ROS subprocess and IPC communication
class CH_ROS_API SubprocessManager {
public:
    /// Constructor
    /// @param node_name Name for the ROS node in the subprocess
    /// @param channel_name Unique name for the IPC channel
    SubprocessManager(const std::string& node_name, const std::string& channel_name);
    
    /// Destructor - automatically terminates subprocess
    ~SubprocessManager();
    
    /// Launch the ROS subprocess
    /// @return true if successful, false otherwise
    bool LaunchSubprocess();
    
    /// Terminate the subprocess gracefully
    void TerminateSubprocess();
    
    /// Check if subprocess is still running
    bool IsSubprocessRunning() const;
    
    /// Send a message to the subprocess
    /// @param message Message to send
    /// @return true if successful
    bool SendMessage(const Message& message);
    
    /// Receive a message from the subprocess (non-blocking)
    /// @param message Output message buffer
    /// @return true if message received
    bool ReceiveMessage(Message& message);
    
    /// Get the IPC channel
    IPCChannel* GetChannel() const { return m_channel.get(); }

private:
    std::string m_node_name;
    std::string m_channel_name;
    std::unique_ptr<IPCChannel> m_channel;
    
#ifdef _WIN32
    HANDLE m_process_handle;
    HANDLE m_thread_handle;
    DWORD m_process_id;
#else
    pid_t m_child_pid;
#endif
    
    bool m_subprocess_running;
    
    /// Get the path to the ROS node executable
    std::string GetExecutablePath() const;
    
    /// Build command line arguments for the subprocess
    std::vector<std::string> BuildArguments() const;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif