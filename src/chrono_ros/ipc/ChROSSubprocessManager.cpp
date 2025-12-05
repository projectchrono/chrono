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
// Subprocess manager for launching and controlling ROS node process
//
// =============================================================================

#include "chrono_ros/ipc/ChROSSubprocessManager.h"
#include <stdexcept>
#include <iostream>
#include <vector>

#ifdef _WIN32
    #include <shlwapi.h>
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
    std::cout << "Launched ROS subprocess for node: " << m_node_name << std::endl;
    
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
        std::cout << "Terminating subprocess PID: " << m_child_pid << std::endl;
        
        // Send SIGTERM and wait
        kill(m_child_pid, SIGTERM);
        
        int status;
        // Wait up to 1 second for graceful shutdown
        for (int i = 0; i < 10; i++) {
            int wait_result = waitpid(m_child_pid, &status, WNOHANG);
            if (wait_result != 0) {
                std::cout << "Subprocess terminated gracefully" << std::endl;
                break;
            }
            usleep(100000);  // 100ms
            
            // After 1 second, force kill
            if (i == 9) {
                std::cout << "Forcing subprocess termination with SIGKILL" << std::endl;
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
    std::cout << "Cleaning up IPC channel..." << std::endl;
    m_channel.reset();
    
    std::cout << "Terminated ROS subprocess" << std::endl;
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
    // For now, assume the ROS node executable is in the same directory as the current executable
    // In a real implementation, this might be configured or found via CMake installation paths
    
#ifdef _WIN32
    char path[MAX_PATH];
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    PathRemoveFileSpecA(path);
    return std::string(path) + "\\chrono_ros_node.exe";
#else
    char path[PATH_MAX];
    // Try to get the path of the current executable (C++ app) or the python interpreter
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (len != -1) {
        path[len] = '\0';
        
        std::string full_path(path);
        std::string dir_path = full_path.substr(0, full_path.find_last_of("/"));
        std::string base_name = full_path.substr(full_path.find_last_of("/") + 1);
        
        // Check if we are running from python (e.g. "python", "python3", "python3.10")
        if (base_name.find("python") != std::string::npos) {
             // If running from python, we need to find where the chrono libraries are installed
             // We use dladdr to find the location of the current shared library (libChrono_ros.so)
             Dl_info info;
             
             // We need a symbol address that resides in this shared library.
             // Since we are inside a member function, we can't easily take its address as void*.
             // However, we can use the address of a static function or a global variable defined in this translation unit.
             // But we don't have one handy.
             // Let's use the address of the current function, but we need to be careful with syntax.
             // A safer and more portable way is to use a non-member function.
             // Let's assume the vtable or RTTI info is close enough.
             
             // Hack: Cast the member function pointer to void* via a union to avoid compiler warnings/errors
             union {
                 std::string (SubprocessManager::*pmf)() const;
                 void* p;
             } u;
             u.pmf = &SubprocessManager::GetExecutablePath;
             
             if (dladdr(u.p, &info)) {
                 std::string lib_path(info.dli_fname);
                 // lib_path is something like /path/to/lib/libChrono_ros.so
                 // We want to find /path/to/bin/chrono_ros_node
                 
                 char* lib_dir_c = dirname(const_cast<char*>(lib_path.c_str()));
                 std::string lib_dir(lib_dir_c);
                 
                 // Check if we are in a build tree (lib/ is sibling to bin/)
                 // or install tree (lib/ is sibling to bin/)
                 // In both cases, we go up one level and look into bin/
                 
                 // However, sometimes libs are in lib/ or lib64/ or just build/
                 // Let's try a few common locations relative to the library
                 
                 // 1. Sibling bin directory: ../bin/chrono_ros_node
                 std::string bin_path = lib_dir + "/../bin/chrono_ros_node";
                 if (access(bin_path.c_str(), X_OK) == 0) {
                     return bin_path;
                 }
                 
                 // 2. Same directory: ./chrono_ros_node (common in build trees on Windows or flat builds)
                 bin_path = lib_dir + "/chrono_ros_node";
                 if (access(bin_path.c_str(), X_OK) == 0) {
                     return bin_path;
                 }
             }
             
             // Fallback: Assume chrono_ros_node is in the PATH
             return "chrono_ros_node";
        }

        return dir_path + "/chrono_ros_node";
    }
    
    // Fallback to relative path
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