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
// Cross-platform shared memory implementation for IPC
//
// =============================================================================

#include "chrono_ros/ipc/ChROSSharedMemory.h"
#include <stdexcept>
#include <iostream>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <cerrno>
    #include <cstring>
#endif

namespace chrono {
namespace ros {
namespace ipc {

SharedMemory::SharedMemory(const std::string& name, size_t size, bool create_new)
    : m_name(name), m_size(size), m_ptr(nullptr), m_created(create_new) {
    
#ifdef _WIN32
    m_handle = nullptr;
    
    if (create_new) {
        m_handle = CreateFileMapping(
            INVALID_HANDLE_VALUE,
            nullptr,
            PAGE_READWRITE,
            0,
            static_cast<DWORD>(size),
            m_name.c_str()
        );
        
        if (m_handle == nullptr) {
            throw std::runtime_error("Failed to create shared memory: " + std::to_string(GetLastError()));
        }
        
        if (GetLastError() == ERROR_ALREADY_EXISTS) {
            throw std::runtime_error("Shared memory segment already exists: " + name);
        }
    } else {
        m_handle = OpenFileMapping(
            FILE_MAP_ALL_ACCESS,
            FALSE,
            m_name.c_str()
        );
        
        if (m_handle == nullptr) {
            throw std::runtime_error("Failed to open shared memory: " + std::to_string(GetLastError()));
        }
    }
    
    m_ptr = MapViewOfFile(
        m_handle,
        FILE_MAP_ALL_ACCESS,
        0,
        0,
        size
    );
    
    if (m_ptr == nullptr) {
        CloseHandle(m_handle);
        m_handle = nullptr;
        throw std::runtime_error("Failed to map shared memory: " + std::to_string(GetLastError()));
    }
    
#else  // Unix/Linux
    std::string shm_name = "/" + name;  // POSIX shared memory requires leading slash
    
    if (create_new) {
        m_fd = shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        if (m_fd == -1) {
            throw std::runtime_error("Failed to create shared memory '" + name + "': " + std::strerror(errno));
        }
        
        if (ftruncate(m_fd, size) == -1) {
            close(m_fd);
            shm_unlink(shm_name.c_str());
            throw std::runtime_error("Failed to set shared memory size: " + std::string(std::strerror(errno)));
        }
    } else {
        m_fd = shm_open(shm_name.c_str(), O_RDWR, 0666);
        if (m_fd == -1) {
            throw std::runtime_error("Failed to open shared memory '" + name + "': " + std::strerror(errno));
        }
        
        // Get actual size of existing shared memory
        struct stat sb;
        if (fstat(m_fd, &sb) == -1) {
            close(m_fd);
            throw std::runtime_error("Failed to get shared memory size: " + std::string(std::strerror(errno)));
        }
        m_size = sb.st_size;
    }
    
    m_ptr = mmap(nullptr, m_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
    if (m_ptr == MAP_FAILED) {
        close(m_fd);
        if (create_new) {
            shm_unlink(shm_name.c_str());
        }
        throw std::runtime_error("Failed to map shared memory: " + std::string(std::strerror(errno)));
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
    
    std::cout << "[SharedMemory] Closing shared memory: " << m_name << ", created=" << m_created << std::endl;
    
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
        std::string shm_name = "/" + m_name;
        std::cout << "[SharedMemory] Unlinking: " << shm_name << std::endl;
        if (shm_unlink(shm_name.c_str()) == 0) {
            std::cout << "[SharedMemory] Successfully unlinked" << std::endl;
        } else {
            std::cerr << "[SharedMemory] Failed to unlink: " << strerror(errno) << std::endl;
        }
    }
#endif
    
    m_ptr = nullptr;
}

}  // namespace ipc
}  // namespace ros
}  // namespace chrono