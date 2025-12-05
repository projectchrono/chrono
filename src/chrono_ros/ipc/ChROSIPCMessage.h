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
// IPC message definitions for chrono_ros subprocess communication
//
// =============================================================================

#ifndef CH_ROS_IPC_MESSAGE_H
#define CH_ROS_IPC_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <memory>

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
    DRIVER_INPUTS = 6,  ///< Bidirectional: subprocess subscriber -> main process
    VIPER_DC_MOTOR_CONTROL = 7,  ///< Bidirectional: subprocess subscriber -> main process
    CAMERA_DATA = 8,  ///< Publisher: main process -> subprocess ROS
    LIDAR_POINTCLOUD = 9,
    LIDAR_LASERSCAN = 10,
    ACCELEROMETER_DATA = 11,
    GYROSCOPE_DATA = 12,
    MAGNETOMETER_DATA = 13,
    IMU_DATA = 14,
    GPS_DATA = 15,
    CUSTOM_DATA = 16,
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
/// Increased to support large sensor data (4K images, dense point clouds)
static constexpr size_t MAX_PAYLOAD_SIZE = 64 * 1024 * 1024;  // 64MB

/// Complete IPC message with header and heap-allocated payload
/// Payload is dynamically allocated to avoid stack overflow with large buffers
struct Message {
    MessageHeader header;
    std::unique_ptr<uint8_t[]> payload;
    
    Message() : payload(std::make_unique<uint8_t[]>(MAX_PAYLOAD_SIZE)) {}
    
    Message(MessageType type, uint64_t time_ns, uint32_t seq, const void* data, size_t size)
        : header(type, time_ns, static_cast<uint32_t>(size), seq),
          payload(std::make_unique<uint8_t[]>(MAX_PAYLOAD_SIZE)) {
        if (size > MAX_PAYLOAD_SIZE) {
            size = MAX_PAYLOAD_SIZE;
        }
        if (data && size > 0) {
            std::memcpy(payload.get(), data, size);
        }
    }
    
    // Move constructor and assignment (unique_ptr is move-only)
    Message(Message&&) = default;
    Message& operator=(Message&&) = default;
    
    // Delete copy constructor and assignment (can't copy unique_ptr)
    Message(const Message&) = delete;
    Message& operator=(const Message&) = delete;
    
    /// Get payload as typed pointer
    template<typename T>
    const T* GetPayload() const {
        return reinterpret_cast<const T*>(payload.get());
    }
    
    template<typename T>
    T* GetPayload() {
        return reinterpret_cast<T*>(payload.get());
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

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif