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

#include "chrono_ros/ChROSNodeProcess.h"
#include "chrono_ros/ChConfigROS.h"

#include <cstring>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
    #include <windows.h>
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#else
    #include <climits>
    #include <dlfcn.h>
    #include <libgen.h>
    #include <signal.h>
    #include <sys/wait.h>
    #include <unistd.h>
#endif

namespace chrono {
namespace ros {

ChROSNodeProcess::~ChROSNodeProcess() {
    Terminate();
}

std::string ChROSNodeProcess::FindExecutable() const {
    // 1. Relative to the Chrono_ros shared library (robust for build and
    //    install trees as long as the lib/bin structure is preserved).
    // 2. The CMake-configured build-tree path.
    // 3. The CMake-configured install-tree path.
    // 4. PATH lookup as a last resort.
#ifdef _WIN32
    HMODULE module = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCSTR>(&ChROSNodeProcess::IsRunning), &module)) {
        char path[MAX_PATH];
        if (GetModuleFileNameA(module, path, MAX_PATH)) {
            PathRemoveFileSpecA(path);
            const std::string lib_dir(path);
            for (const std::string& candidate :
                 {lib_dir + "\\chrono_ros_node.exe", lib_dir + "\\..\\bin\\chrono_ros_node.exe"}) {
                if (PathFileExistsA(candidate.c_str())) {
                    return candidate;
                }
            }
        }
    }
    for (const std::string& candidate :
         {std::string(CH_ROS_NODE_BUILD_PATH) + ".exe", std::string(CH_ROS_NODE_INSTALL_PATH) + ".exe"}) {
        if (PathFileExistsA(candidate.c_str())) {
            return candidate;
        }
    }
    return "chrono_ros_node.exe";
#else
    Dl_info info;
    // Find the shared library containing this code via a function address.
    union {
        bool (ChROSNodeProcess::*member)() const;
        void* address;
    } caster;
    caster.member = &ChROSNodeProcess::IsRunning;

    if (dladdr(caster.address, &info) && info.dli_fname != nullptr) {
        char lib_path[PATH_MAX];
        std::strncpy(lib_path, info.dli_fname, PATH_MAX - 1);
        lib_path[PATH_MAX - 1] = '\0';
        const std::string lib_dir(dirname(lib_path));
        for (const std::string& candidate : {lib_dir + "/chrono_ros_node", lib_dir + "/../bin/chrono_ros_node"}) {
            if (access(candidate.c_str(), X_OK) == 0) {
                return candidate;
            }
        }
    }
    for (const std::string& candidate : {std::string(CH_ROS_NODE_BUILD_PATH), std::string(CH_ROS_NODE_INSTALL_PATH)}) {
        if (!candidate.empty() && access(candidate.c_str(), X_OK) == 0) {
            return candidate;
        }
    }
    return "chrono_ros_node";
#endif
}

void ChROSNodeProcess::Launch(const std::string& node_name, const std::string& channel_name) {
    if (m_running) {
        throw std::runtime_error("chrono_ros_node is already running");
    }
    m_executable = FindExecutable();

    const std::vector<std::string> arguments = {"--node-name", node_name, "--channel-name", channel_name};

#ifdef _WIN32
    std::string cmdline = "\"" + m_executable + "\"";
    for (const auto& argument : arguments) {
        cmdline += " \"" + argument + "\"";
    }

    STARTUPINFOA startup = {};
    PROCESS_INFORMATION process = {};
    startup.cb = sizeof(startup);

    if (!CreateProcessA(m_executable.c_str(), const_cast<char*>(cmdline.c_str()), nullptr, nullptr, FALSE, 0, nullptr,
                        nullptr, &startup, &process)) {
        throw std::runtime_error("failed to launch '" + m_executable + "': error " + std::to_string(GetLastError()));
    }
    m_process_handle = process.hProcess;
    m_thread_handle = process.hThread;
#else
    const pid_t pid = fork();
    if (pid == -1) {
        throw std::runtime_error(std::string("failed to fork chrono_ros_node: ") + std::strerror(errno));
    }
    if (pid == 0) {
        // Child. The environment is inherited, which is what carries the
        // sourced ROS workspace(s) to the node.
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(m_executable.c_str()));
        for (const auto& argument : arguments) {
            argv.push_back(const_cast<char*>(argument.c_str()));
        }
        argv.push_back(nullptr);
        execv(m_executable.c_str(), argv.data());
        // Only reached if execv failed.
        _exit(127);
    }
    m_child_pid = pid;
#endif
    m_running = true;
}

bool ChROSNodeProcess::IsRunning() const {
    if (!m_running) {
        return false;
    }
#ifdef _WIN32
    DWORD exit_code = 0;
    if (m_process_handle == nullptr || !GetExitCodeProcess(m_process_handle, &exit_code)) {
        return false;
    }
    return exit_code == STILL_ACTIVE;
#else
    if (m_child_pid <= 0) {
        return false;
    }
    return waitpid(m_child_pid, nullptr, WNOHANG) == 0;
#endif
}

void ChROSNodeProcess::Terminate() {
    if (!m_running) {
        return;
    }
#ifdef _WIN32
    if (m_process_handle != nullptr) {
        if (WaitForSingleObject(m_process_handle, 2000) == WAIT_TIMEOUT) {
            TerminateProcess(m_process_handle, 1);
        }
        CloseHandle(static_cast<HANDLE>(m_process_handle));
        CloseHandle(static_cast<HANDLE>(m_thread_handle));
        m_process_handle = nullptr;
        m_thread_handle = nullptr;
    }
#else
    if (m_child_pid > 0) {
        kill(m_child_pid, SIGTERM);
        int status = 0;
        bool exited = false;
        for (int i = 0; i < 20; i++) {  // up to 2 s of grace
            if (waitpid(m_child_pid, &status, WNOHANG) != 0) {
                exited = true;
                break;
            }
            usleep(100000);
        }
        if (!exited) {
            kill(m_child_pid, SIGKILL);
            waitpid(m_child_pid, &status, 0);
        }
        m_child_pid = -1;
    }
#endif
    m_running = false;
}

}  // namespace ros
}  // namespace chrono
