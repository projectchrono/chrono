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

#include "chrono_ros/core/transport/ChROSSharedMemory.h"
#include <cstdint>

#include <atomic>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <signal.h>
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <unistd.h>
    #include <cerrno>
#endif

namespace chrono {
namespace ros {
namespace core {

namespace {

void ValidateName(const std::string& name) {
    if (name.empty() || name.size() > 200) {
        throw std::runtime_error("shared memory name must be 1-200 characters, got '" + name + "'");
    }
    for (char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
                        c == '-' || c == '.';
        if (!ok) {
            throw std::runtime_error("shared memory name '" + name +
                                     "' contains invalid character '" + std::string(1, c) +
                                     "' (allowed: alphanumerics, '_', '-', '.')");
        }
    }
}

}  // namespace

#ifndef _WIN32
// ----------------------------------------------------------------------------
// POSIX: track created segments so SIGINT/SIGTERM can unlink them. Only
// async-signal-safe calls (shm_unlink, _Exit) happen inside the handler.
// ----------------------------------------------------------------------------
namespace {

constexpr int kMaxTrackedSegments = 16;
constexpr size_t kTrackedNameLength = 256;

struct TrackedSegment {
    char path[kTrackedNameLength];
    std::atomic<bool> active{false};
};

TrackedSegment g_tracked[kMaxTrackedSegments];
std::atomic<int> g_tracked_total{0};
int g_next_tracker_index = 0;
std::mutex g_tracker_mutex;
std::once_flag g_handlers_installed;

void SegmentSignalHandler(int signal) {
    const int count = g_tracked_total.load(std::memory_order_acquire);
    for (int i = 0; i < count; ++i) {
        if (g_tracked[i].active.load(std::memory_order_acquire)) {
            shm_unlink(g_tracked[i].path);
        }
    }
    _Exit(128 + signal);
}

void InstallSegmentSignalHandlers() {
    struct sigaction action {};
    action.sa_handler = SegmentSignalHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    #ifdef SIGQUIT
    sigaction(SIGQUIT, &action, nullptr);
    #endif
}

int RegisterTrackedSegment(const std::string& name) {
    std::call_once(g_handlers_installed, InstallSegmentSignalHandlers);

    std::lock_guard<std::mutex> lock(g_tracker_mutex);
    if (g_next_tracker_index >= kMaxTrackedSegments) {
        return -1;  // not tracked; destructor cleanup still applies
    }
    const int index = g_next_tracker_index++;
    g_tracked[index].path[0] = '/';
    std::strncpy(g_tracked[index].path + 1, name.c_str(), kTrackedNameLength - 2);
    g_tracked[index].path[kTrackedNameLength - 1] = '\0';
    g_tracked[index].active.store(true, std::memory_order_release);
    g_tracked_total.store(g_next_tracker_index, std::memory_order_release);
    return index;
}

void UnregisterTrackedSegment(int index) {
    if (index < 0 || index >= kMaxTrackedSegments) {
        return;
    }
    g_tracked[index].active.store(false, std::memory_order_release);
}

}  // namespace
#endif  // !_WIN32

// ----------------------------------------------------------------------------

SharedMemory::SharedMemory(const std::string& name, size_t size, bool create_new)
    : m_name(name), m_size(size), m_ptr(nullptr), m_created(create_new), m_signal_index(-1) {
    ValidateName(name);

#ifdef _WIN32
    m_handle = nullptr;

    if (create_new) {
        m_handle = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                                      static_cast<DWORD>(static_cast<uint64_t>(size) >> 32),
                                      static_cast<DWORD>(size & 0xFFFFFFFFull), m_name.c_str());
        if (m_handle == nullptr) {
            throw std::runtime_error("failed to create shared memory '" + name +
                                     "': error " + std::to_string(GetLastError()));
        }
        if (GetLastError() == ERROR_ALREADY_EXISTS) {
            CloseHandle(m_handle);
            m_handle = nullptr;
            throw std::runtime_error("shared memory segment already exists: " + name);
        }
    } else {
        m_handle = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, m_name.c_str());
        if (m_handle == nullptr) {
            throw std::runtime_error("failed to open shared memory '" + name +
                                     "': error " + std::to_string(GetLastError()));
        }
    }

    m_ptr = MapViewOfFile(m_handle, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (m_ptr == nullptr) {
        CloseHandle(m_handle);
        m_handle = nullptr;
        throw std::runtime_error("failed to map shared memory '" + name +
                                 "': error " + std::to_string(GetLastError()));
    }

#else  // POSIX
    const std::string shm_name = "/" + name;

    if (create_new) {
        m_fd = shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (m_fd == -1) {
            throw std::runtime_error("failed to create shared memory '" + name + "': " + std::strerror(errno));
        }
        if (ftruncate(m_fd, static_cast<off_t>(size)) == -1) {
            const std::string detail = std::strerror(errno);
            close(m_fd);
            shm_unlink(shm_name.c_str());
            throw std::runtime_error("failed to size shared memory '" + name + "' to " + std::to_string(size) +
                                     " bytes: " + detail +
                                     " (in Docker, check that /dev/shm is large enough; see chrono_ros docs)");
        }
    } else {
        m_fd = shm_open(shm_name.c_str(), O_RDWR, 0600);
        if (m_fd == -1) {
            throw std::runtime_error("failed to open shared memory '" + name + "': " + std::strerror(errno));
        }
        struct stat sb {};
        if (fstat(m_fd, &sb) == -1) {
            const std::string detail = std::strerror(errno);
            close(m_fd);
            throw std::runtime_error("failed to stat shared memory '" + name + "': " + detail);
        }
        m_size = static_cast<size_t>(sb.st_size);
    }

    m_ptr = mmap(nullptr, m_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
    if (m_ptr == MAP_FAILED) {
        const std::string detail = std::strerror(errno);
        close(m_fd);
        if (create_new) {
            shm_unlink(shm_name.c_str());
        }
        m_ptr = nullptr;
        throw std::runtime_error("failed to map shared memory '" + name + "': " + detail);
    }

    if (create_new) {
        m_signal_index = RegisterTrackedSegment(name);
    }
#endif
}

SharedMemory::~SharedMemory() {
    Close();
}

void SharedMemory::Close() {
    if (m_ptr == nullptr) {
        return;
    }

#ifdef _WIN32
    UnmapViewOfFile(m_ptr);
    if (m_handle) {
        CloseHandle(m_handle);
        m_handle = nullptr;
    }
#else
    munmap(m_ptr, m_size);
    close(m_fd);
    if (m_created) {
        const std::string shm_name = "/" + m_name;
        if (shm_unlink(shm_name.c_str()) != 0 && errno != ENOENT) {
            // Destructor: report but never throw.
            std::cerr << "[chrono_ros] failed to unlink shared memory '" << m_name << "': " << std::strerror(errno)
                      << std::endl;
        }
    }
    if (m_signal_index >= 0) {
        UnregisterTrackedSegment(m_signal_index);
        m_signal_index = -1;
    }
#endif

    m_ptr = nullptr;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
