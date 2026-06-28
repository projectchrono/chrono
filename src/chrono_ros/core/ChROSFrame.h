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
// Wire protocol v1 between the simulation process and chrono_ros_node.
//
// Every frame on the IPC channel is a fixed 24-byte FrameHeader followed by
// payload_size bytes of payload. Control frames carry CDR-encoded payloads
// (see ChROSControl.h); PUBLISH/RECEIVED frames carry a ROS serialized
// message verbatim. The dispatcher switches on these kinds and nothing else -
// per-topic extensibility lives entirely in the data, never in this enum.
//
// =============================================================================

#ifndef CH_ROS_CORE_FRAME_H
#define CH_ROS_CORE_FRAME_H

#include <cstddef>
#include <cstdint>

namespace chrono {
namespace ros {
namespace core {

constexpr uint32_t kFrameMagic = 0xC4050516u;
constexpr uint16_t kProtocolVersion = 1;

enum class FrameKind : uint16_t {
    Hello = 1,        ///< node -> sim: node is up (payload: HelloPayload)
    DescribeType,     ///< sim -> node: request a type schema (DescribeTypePayload)
    TypeSchema,       ///< node -> sim: schema or error (TypeSchemaPayload)
    ValidateType,     ///< sim -> node: round-trip a CDR sample (ValidateTypePayload)
    ValidateResult,   ///< node -> sim: byte-compare verdict (ValidateResultPayload)
    Advertise,        ///< sim -> node: create publisher/subscription (AdvertisePayload)
    AdvertiseAck,     ///< node -> sim: result (AdvertiseAckPayload)
    Publish,          ///< sim -> node: serialized message for a publisher channel
    Received,         ///< node -> sim: serialized message from a subscription channel
    Unadvertise,      ///< sim -> node: tear down a channel (no payload)
    ChannelInfo,      ///< node -> sim: matched-endpoint count update (ChannelInfoPayload)
    Shutdown,         ///< sim -> node: exit cleanly (no payload)
};

/// Returns a human-readable name for diagnostics, or "unknown".
const char* FrameKindName(FrameKind kind);

/// True if the raw value is a defined FrameKind.
inline bool IsValidFrameKind(uint16_t raw) {
    return raw >= static_cast<uint16_t>(FrameKind::Hello) && raw <= static_cast<uint16_t>(FrameKind::Shutdown);
}

/// Fixed header preceding every frame. Plain bytes in shared memory; the
/// layout is identical on both ends because both are built from this header
/// (and little-endian per the protocol-wide requirement).
struct FrameHeader {
    uint32_t magic = kFrameMagic;
    uint16_t version = kProtocolVersion;
    uint16_t kind = 0;          ///< FrameKind
    uint32_t channel_id = 0;    ///< topic channel for Advertise/Publish/Received/...; 0 for global frames
    uint32_t payload_size = 0;  ///< bytes of payload following this header
    uint64_t sim_time_ns = 0;   ///< simulation time when the frame was produced (0 if not applicable)
};

static_assert(sizeof(FrameHeader) == 24, "FrameHeader must be exactly 24 bytes (no padding)");

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
