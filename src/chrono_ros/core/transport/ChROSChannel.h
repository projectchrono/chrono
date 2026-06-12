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
// Bidirectional frame channel between the simulation process and
// chrono_ros_node, over one shared-memory segment holding two SPSC rings
// (sim->node and node->sim). Frames are published atomically: a reader sees a
// complete frame or nothing.
//
// Capacities are configurable and intentionally asymmetric: the sim->node
// direction carries bulk sensor data, the return path carries small commands.
// A frame larger than its ring's capacity is a configuration error and is
// reported with the knob to turn (never silently truncated).
//
// =============================================================================

#ifndef CH_ROS_CORE_CHANNEL_H
#define CH_ROS_CORE_CHANNEL_H

#include "chrono_ros/core/ChROSFrame.h"
#include "chrono_ros/core/transport/ChROSRingBuffer.h"
#include "chrono_ros/core/transport/ChROSSharedMemory.h"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace chrono {
namespace ros {
namespace core {

/// Error thrown on channel setup failure, protocol corruption, or an
/// oversized frame. Normal flow control (ring momentarily full / empty) is
/// reported through boolean returns, never exceptions.
class ChannelError : public std::runtime_error {
  public:
    explicit ChannelError(const std::string& what) : std::runtime_error("IPC channel error: " + what) {}
};

class Channel {
  public:
    struct Config {
        /// Bulk direction (sensor data). Default comfortably holds several
        /// 4K RGBA frames (~32 MiB each) in flight.
        size_t sim_to_node_capacity = 128ull << 20;
        /// Command/return direction.
        size_t node_to_sim_capacity = 32ull << 20;
    };

    /// Simulation side: create the shared-memory segment (owns its lifetime).
    /// Capacities are rounded up to powers of 2.
    static std::unique_ptr<Channel> Create(const std::string& name, const Config& config);
    static std::unique_ptr<Channel> Create(const std::string& name) { return Create(name, Config()); }

    /// Node side: attach to an existing segment and mark itself attached.
    static std::unique_ptr<Channel> Open(const std::string& name);

    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    /// Queue one frame. Returns false if the ring is currently full (caller
    /// decides whether to retry, drop, or warn). Throws ChannelError if the
    /// frame can never fit (payload exceeds ring capacity).
    bool SendFrame(FrameKind kind,
                   uint32_t channel_id,
                   uint64_t sim_time_ns,
                   const void* payload = nullptr,
                   size_t payload_size = 0);

    /// Dequeue one frame if available; 'payload' is resized to fit. Returns
    /// false when the ring is empty. Throws ChannelError on corruption.
    bool ReceiveFrame(FrameHeader& header, std::vector<uint8_t>& payload);

    /// Largest payload this side can ever send (send-ring capacity minus the
    /// frame header).
    size_t MaxSendPayload() const;

    /// Bytes currently free in the send ring.
    size_t SendSpace() const;

    /// True once the node side has attached (sim side uses this during startup).
    bool IsPeerAttached() const;

    const std::string& GetName() const { return m_shm->GetName(); }

  private:
    Channel(std::unique_ptr<SharedMemory> shm, bool is_sim_side, bool created);

    std::unique_ptr<SharedMemory> m_shm;
    std::unique_ptr<RingBuffer> m_send;
    std::unique_ptr<RingBuffer> m_recv;
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
