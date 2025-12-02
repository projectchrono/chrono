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
// IPC message definitions for chrono_ros subprocess communication
//
// =============================================================================

#ifndef CH_ROS_IPC_MESSAGE_H
#define CH_ROS_IPC_MESSAGE_H

#include <cstdint>
#include <cstring>

namespace chrono {
namespace ros {
namespace ipc {

/// Message types for IPC communication
enum class MessageType : uint32_t {
    INITIALIZE = 1,
    CLOCK_DATA = 2,
    BODY_DATA = 3,
    TF_DATA = 4,
    ROBOT_MODEL_DATA = 5,
    CUSTOM_DATA = 6,
    SHUTDOWN = 99
};

/// IPC message header for all communications
struct MessageHeader {
    static constexpr uint32_t MAGIC = 0xC4050515;
    
    uint32_t magic;           ///< Protocol validation
    MessageType type;         ///< Message type identifier
    uint64_t timestamp_ns;    ///< Chrono simulation time in nanoseconds
    uint32_t payload_size;    ///< Size of payload data in bytes
    uint32_t sequence;        ///< Message sequence number
    
    MessageHeader() = default;
    MessageHeader(MessageType msg_type, uint64_t time_ns, uint32_t size, uint32_t seq)
        : magic(MAGIC), type(msg_type), timestamp_ns(time_ns), payload_size(size), sequence(seq) {}
    
    /// Validate message header
    bool IsValid() const { return magic == MAGIC; }
};

/// Maximum payload size for a single message
static constexpr size_t MAX_PAYLOAD_SIZE = 1024 * 1024;  // 1MB

/// Complete IPC message with header and payload
struct Message {
    MessageHeader header;
    alignas(8) uint8_t payload[MAX_PAYLOAD_SIZE];
    
    Message() = default;
    Message(MessageType type, uint64_t time_ns, uint32_t seq, const void* data, size_t size)
        : header(type, time_ns, static_cast<uint32_t>(size), seq) {
        if (size > MAX_PAYLOAD_SIZE) {
            size = MAX_PAYLOAD_SIZE;
        }
        if (data && size > 0) {
            std::memcpy(payload, data, size);
        }
    }
    
    /// Get payload as typed pointer
    template<typename T>
    const T* GetPayload() const {
        return reinterpret_cast<const T*>(payload);
    }
    
    template<typename T>
    T* GetPayload() {
        return reinterpret_cast<T*>(payload);
    }
};

/// Clock handler data
struct ClockData {
    double time_seconds;
};

/// Body handler data
struct BodyData {
    char body_name[64];
    char topic_prefix[128];
    
    // Pose data
    double pos_x, pos_y, pos_z;
    double rot_w, rot_x, rot_y, rot_z;
    
    // Velocity data  
    double lin_vel_x, lin_vel_y, lin_vel_z;
    double ang_vel_x, ang_vel_y, ang_vel_z;
    
    // Acceleration data
    double lin_acc_x, lin_acc_y, lin_acc_z;
    double ang_acc_x, ang_acc_y, ang_acc_z;
};

/// TF transform data
struct TFTransform {
    char parent_frame[64];
    char child_frame[64];
    double pos_x, pos_y, pos_z;
    double rot_w, rot_x, rot_y, rot_z;
};

/// TF handler data (variable size)
struct TFData {
    uint32_t transform_count;
    // Followed by transform_count TFTransform structures
};

/// Robot model handler data
struct RobotModelData {
    char topic_name[128];
    uint32_t urdf_size;
    // Followed by URDF string data
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif