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
// Control-plane payload definitions for the wire protocol (ChROSFrame.h).
// Each payload is CDR-encoded (without encapsulation header) using the same
// codec the data plane is built on. Encode appends to a byte vector; Decode
// throws CdrError on malformed input.
//
// =============================================================================

#ifndef CH_ROS_CORE_CONTROL_H
#define CH_ROS_CORE_CONTROL_H

#include "chrono_ros/core/ChROSQoSSpec.h"

#include <cstdint>
#include <string>
#include <vector>

namespace chrono {
namespace ros {
namespace core {

/// node -> sim, once at startup.
struct HelloPayload {
    uint16_t protocol_version = 0;
    uint32_t pid = 0;
    std::string ros_distro;  ///< from $ROS_DISTRO
    std::string rmw_id;      ///< rmw implementation identifier
    std::string node_name;

    void Encode(std::vector<uint8_t>& out) const;
    static HelloPayload Decode(const uint8_t* data, size_t size);
};

/// sim -> node: request the schema of a message type.
struct DescribeTypePayload {
    std::string type_name;  ///< e.g. "sensor_msgs/msg/Image"

    void Encode(std::vector<uint8_t>& out) const;
    static DescribeTypePayload Decode(const uint8_t* data, size_t size);
};

/// node -> sim: the schema, or why it could not be produced.
struct TypeSchemaPayload {
    std::string type_name;
    bool ok = false;
    std::string error;                ///< human-readable, set when !ok
    std::vector<uint8_t> schema_blob; ///< Schema::EncodeBlob output, set when ok

    void Encode(std::vector<uint8_t>& out) const;
    static TypeSchemaPayload Decode(const uint8_t* data, size_t size);
};

/// sim -> node: a serialized sample to round-trip through the real rmw
/// typesupport for byte-exact validation.
struct ValidateTypePayload {
    std::string type_name;
    std::vector<uint8_t> sample_cdr;

    void Encode(std::vector<uint8_t>& out) const;
    static ValidateTypePayload Decode(const uint8_t* data, size_t size);
};

/// node -> sim: validation verdict. On mismatch, carries the first differing
/// offset and a window of both byte streams around it for diagnostics.
struct ValidateResultPayload {
    std::string type_name;
    bool ok = false;
    std::string error;  ///< set when the round trip itself failed
    uint64_t first_diff_offset = 0;
    std::vector<uint8_t> expected_window;  ///< node's re-serialization around the diff
    std::vector<uint8_t> actual_window;    ///< sim's bytes around the diff

    void Encode(std::vector<uint8_t>& out) const;
    static ValidateResultPayload Decode(const uint8_t* data, size_t size);
};

/// sim -> node: create a generic publisher or subscription.
struct AdvertisePayload {
    enum class Direction : uint8_t {
        Publish = 0,    ///< sim publishes; node creates a generic publisher
        Subscribe = 1,  ///< sim subscribes; node creates a generic subscription
    };

    Direction direction = Direction::Publish;
    std::string topic;
    std::string type_name;
    QoSSpec qos;

    void Encode(std::vector<uint8_t>& out) const;
    static AdvertisePayload Decode(const uint8_t* data, size_t size);
};

/// node -> sim: result of an Advertise.
struct AdvertiseAckPayload {
    bool ok = false;
    std::string error;

    void Encode(std::vector<uint8_t>& out) const;
    static AdvertiseAckPayload Decode(const uint8_t* data, size_t size);
};

/// node -> sim: number of matched remote endpoints on a channel (subscribers
/// of our publisher / publishers feeding our subscription). Sent on change.
struct ChannelInfoPayload {
    uint32_t matched_endpoints = 0;

    void Encode(std::vector<uint8_t>& out) const;
    static ChannelInfoPayload Decode(const uint8_t* data, size_t size);
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
