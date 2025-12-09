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

#include "chrono_ros/ipc/ChROSSubprocessManager.h"
#include "chrono_ros/ChConfigROS.h"

#include <stdexcept>
#include <iostream>
#include <vector>
#include <fstream>

#ifdef _WIN32
    #include <windows.h>
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#else
    #include <unistd.h>
    #include <sys/wait.h>
    #include <signal.h>
    #include <libgen.h>
    #include <climits>
    #include <dlfcn.h>
#endif

namespace chrono {
namespace ros {
namespace ipc {

SubprocessManager::SubprocessManager(const std::string& node_name, const std::string& channel_name)
    : m_node_name(node_name), m_channel_name(channel_name), m_subprocess_running(false) {
    
#ifdef _WIN32
    m_process_handle = INVALID_HANDLE_VALUE;
    m_thread_handle = INVALID_HANDLE_VALUE;
    m_process_id = 0;
#else
    m_child_pid = -1;
#endif
}

SubprocessManager::~SubprocessManager() {
    TerminateSubprocess();
}

bool SubprocessManager::LaunchSubprocess() {
    if (m_subprocess_running) {
        return true;  // Already running
    }
    
    // Create IPC channel first
    try {
        m_channel = IPCChannel::CreateMainChannel(m_channel_name);
    } catch (const std::exception& e) {
        std::cerr << "Failed to create IPC channel: " << e.what() << std::endl;
        return false;
    }
    
    std::string executable = GetExecutablePath();
    auto arguments = BuildArguments();
    
#ifdef _WIN32
    // Windows implementation
    std::string cmdline = executable;
    for (const auto& arg : arguments) {
        cmdline += " \"" + arg + "\"";
    }
    
    STARTUPINFOA si = {};
    PROCESS_INFORMATION pi = {};
    si.cb = sizeof(si);
    
    BOOL success = CreateProcessA(
        executable.c_str(),
        const_cast<char*>(cmdline.c_str()),
        nullptr,  // Process security attributes
        nullptr,  // Thread security attributes  
        FALSE,    // Inherit handles
        0,        // Creation flags
        nullptr,  // Environment
        nullptr,  // Current directory
        &si,      // Startup info
        &pi       // Process information
    );
    
    if (!success) {
        std::cerr << "Failed to launch subprocess: " << GetLastError() << std::endl;
        return false;
    }
    
    m_process_handle = pi.hProcess;
    m_thread_handle = pi.hThread;
    m_process_id = pi.dwProcessId;
    
#else
    // Unix/Linux implementation
    m_child_pid = fork();
    
    if (m_child_pid == -1) {
        std::cerr << "Failed to fork subprocess: " << strerror(errno) << std::endl;
        return false;
    }
    
    if (m_child_pid == 0) {
        // Child process - prepare arguments for execv
        std::vector<char*> argv_ptrs;
        argv_ptrs.push_back(const_cast<char*>(executable.c_str()));
        
        for (const auto& arg : arguments) {
            argv_ptrs.push_back(const_cast<char*>(arg.c_str()));
        }
        argv_ptrs.push_back(nullptr);
        
        // Execute the ROS node process
        execv(executable.c_str(), argv_ptrs.data());
        
        // If we reach here, execv failed
        std::cerr << "Failed to exec subprocess: " << strerror(errno) << std::endl;
        _exit(1);
    }
#endif
    
    m_subprocess_running = true;
    
    return true;
}

void SubprocessManager::TerminateSubprocess() {
    if (!m_subprocess_running) {
        return;
    }
    
    // Send shutdown message first
    if (m_channel) {
        Message shutdown_msg(MessageType::SHUTDOWN, 0, 0, nullptr, 0);
        SendMessage(shutdown_msg);
    }
    
#ifdef _WIN32
    if (m_process_handle != INVALID_HANDLE_VALUE) {
        // Wait for graceful shutdown, then force terminate
        DWORD wait_result = WaitForSingleObject(m_process_handle, 2000);  // 2 second timeout
        if (wait_result == WAIT_TIMEOUT) {
            TerminateProcess(m_process_handle, 1);
        }
        
        CloseHandle(m_process_handle);
        CloseHandle(m_thread_handle);
        m_process_handle = INVALID_HANDLE_VALUE;
        m_thread_handle = INVALID_HANDLE_VALUE;
    }
#else
    if (m_child_pid > 0) {
        
        // Send SIGTERM and wait
        kill(m_child_pid, SIGTERM);
        
        int status;
        // Wait up to 1 second for graceful shutdown
        for (int i = 0; i < 10; i++) {
            int wait_result = waitpid(m_child_pid, &status, WNOHANG);
            if (wait_result != 0) {
                break;
            }
            usleep(100000);  // 100ms
            
            // After 1 second, force kill
            if (i == 9) {
                kill(m_child_pid, SIGKILL);
                waitpid(m_child_pid, &status, 0);
            }
        }
        
        m_child_pid = -1;
    }
#endif
    
    m_subprocess_running = false;
    
    // Explicitly close IPC channel to ensure cleanup
    // This triggers SharedMemory destructor which calls shm_unlink
    m_channel.reset();
}

bool SubprocessManager::IsSubprocessRunning() const {
    if (!m_subprocess_running) {
        return false;
    }
    
#ifdef _WIN32
    if (m_process_handle == INVALID_HANDLE_VALUE) {
        return false;
    }
    
    DWORD exit_code;
    if (!GetExitCodeProcess(m_process_handle, &exit_code)) {
        return false;
    }
    
    return exit_code == STILL_ACTIVE;
#else
    if (m_child_pid <= 0) {
        return false;
    }
    
    int status;
    int result = waitpid(m_child_pid, &status, WNOHANG);
    return result == 0;  // 0 means still running
#endif
}
bool SubprocessManager::SendMessage(const Message& message) {
    return m_channel && m_channel->SendMessage(message);
}

bool SubprocessManager::ReceiveMessage(Message& message) {
    return m_channel && m_channel->ReceiveMessage(message);
}

std::string SubprocessManager::GetExecutablePath() const {
    // Strategy:
    // 1. Try to find the executable relative to the shared library (libChrono_ros).
    //    This is the most robust method for both build and install trees, as long as
    //    the relative structure (lib/ vs bin/) is preserved.
    // 2. Check the configured build directory path (from CMake).
    // 3. Check the configured install directory path (from CMake).
    // 4. Fallback to system PATH.

#ifdef _WIN32
    HMODULE hModule = NULL;
    // Get handle to the module containing this function (the DLL)
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | 
                          GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          (LPCSTR)&SubprocessManager::GetExecutablePath, 
                          &hModule)) {
        char path[MAX_PATH];
        if (GetModuleFileNameA(hModule, path, MAX_PATH)) {
            PathRemoveFileSpecA(path); // Get directory of DLL
            std::string lib_dir(path);
            
            // Check ../bin/chrono_ros_node.exe (Standard install structure)
            std::string bin_path = lib_dir + "\\..\\bin\\chrono_ros_node.exe";
            if (PathFileExistsA(bin_path.c_str())) return bin_path;
            
            // Check ./chrono_ros_node.exe (Flat build structure)
            bin_path = lib_dir + "\\chrono_ros_node.exe";
            if (PathFileExistsA(bin_path.c_str())) return bin_path;
        }
    }

    // Check configured paths
    std::string build_path = std::string(CH_ROS_NODE_BUILD_PATH) + ".exe";
    if (PathFileExistsA(build_path.c_str())) return build_path;

    std::string install_path = std::string(CH_ROS_NODE_INSTALL_PATH) + ".exe";
    if (PathFileExistsA(install_path.c_str())) return install_path;

    return "chrono_ros_node.exe";
#else
    Dl_info info;
    // Hack: Cast the member function pointer to void* via a union to avoid compiler warnings/errors
    // This allows us to find the location of the shared library containing this code.
    union {
        std::string (SubprocessManager::*pmf)() const;
        void* p;
    } u;
    u.pmf = &SubprocessManager::GetExecutablePath;
    
    if (dladdr(u.p, &info)) {
        char lib_path_buf[PATH_MAX];
        strncpy(lib_path_buf, info.dli_fname, PATH_MAX);
        lib_path_buf[PATH_MAX - 1] = '\0'; // Ensure null termination
        
        char* lib_dir_c = dirname(lib_path_buf);
        std::string lib_dir(lib_dir_c);
        
        // Check ../bin/chrono_ros_node (Standard install structure)
        std::string bin_path = lib_dir + "/../bin/chrono_ros_node";
        if (access(bin_path.c_str(), X_OK) == 0) return bin_path;
        
        // Check ./chrono_ros_node (Flat build structure)
        bin_path = lib_dir + "/chrono_ros_node";
        if (access(bin_path.c_str(), X_OK) == 0) return bin_path;
    }

    // Check configured paths
    std::string build_path = CH_ROS_NODE_BUILD_PATH;
    if (access(build_path.c_str(), X_OK) == 0) return build_path;

    std::string install_path = CH_ROS_NODE_INSTALL_PATH;
    if (access(install_path.c_str(), X_OK) == 0) return install_path;

    // Fallback to relative path (relies on PATH)
    return "chrono_ros_node";
#endif
}

std::vector<std::string> SubprocessManager::BuildArguments() const {
    return {
        "--node-name", m_node_name,
        "--channel-name", m_channel_name
    };
}

}  // namespace ipc
}  // namespace ros
}  // namespace chrono