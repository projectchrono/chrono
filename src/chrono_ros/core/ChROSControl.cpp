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

#include "chrono_ros/core/ChROSControl.h"
#include "chrono_ros/core/ChROSCdr.h"
#include "chrono_ros/core/ChROSFrame.h"

namespace chrono {
namespace ros {
namespace core {

const char* FrameKindName(FrameKind kind) {
    switch (kind) {
        case FrameKind::Hello:
            return "Hello";
        case FrameKind::DescribeType:
            return "DescribeType";
        case FrameKind::TypeSchema:
            return "TypeSchema";
        case FrameKind::ValidateType:
            return "ValidateType";
        case FrameKind::ValidateResult:
            return "ValidateResult";
        case FrameKind::Advertise:
            return "Advertise";
        case FrameKind::AdvertiseAck:
            return "AdvertiseAck";
        case FrameKind::Publish:
            return "Publish";
        case FrameKind::Received:
            return "Received";
        case FrameKind::Unadvertise:
            return "Unadvertise";
        case FrameKind::ChannelInfo:
            return "ChannelInfo";
        case FrameKind::Shutdown:
            return "Shutdown";
    }
    return "unknown";
}

// ----------------------------------------------------------------------------

void HelloPayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteUInt16(protocol_version);
    w.WriteUInt32(pid);
    w.WriteString(ros_distro);
    w.WriteString(rmw_id);
    w.WriteString(node_name);
}

HelloPayload HelloPayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    HelloPayload p;
    p.protocol_version = r.ReadUInt16();
    p.pid = r.ReadUInt32();
    p.ros_distro = r.ReadString();
    p.rmw_id = r.ReadString();
    p.node_name = r.ReadString();
    return p;
}

// ----------------------------------------------------------------------------

void DescribeTypePayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteString(type_name);
}

DescribeTypePayload DescribeTypePayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    DescribeTypePayload p;
    p.type_name = r.ReadString();
    return p;
}

// ----------------------------------------------------------------------------

void TypeSchemaPayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteString(type_name);
    w.WriteBool(ok);
    w.WriteString(error);
    w.WriteBlob(schema_blob.data(), schema_blob.size());
}

TypeSchemaPayload TypeSchemaPayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    TypeSchemaPayload p;
    p.type_name = r.ReadString();
    p.ok = r.ReadBool();
    p.error = r.ReadString();
    p.schema_blob = r.ReadBlob();
    return p;
}

// ----------------------------------------------------------------------------

void ValidateTypePayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteString(type_name);
    w.WriteBlob(sample_cdr.data(), sample_cdr.size());
}

ValidateTypePayload ValidateTypePayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    ValidateTypePayload p;
    p.type_name = r.ReadString();
    p.sample_cdr = r.ReadBlob();
    return p;
}

// ----------------------------------------------------------------------------

void ValidateResultPayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteString(type_name);
    w.WriteBool(ok);
    w.WriteString(error);
    w.WriteUInt64(first_diff_offset);
    w.WriteBlob(expected_window.data(), expected_window.size());
    w.WriteBlob(actual_window.data(), actual_window.size());
}

ValidateResultPayload ValidateResultPayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    ValidateResultPayload p;
    p.type_name = r.ReadString();
    p.ok = r.ReadBool();
    p.error = r.ReadString();
    p.first_diff_offset = r.ReadUInt64();
    p.expected_window = r.ReadBlob();
    p.actual_window = r.ReadBlob();
    return p;
}

// ----------------------------------------------------------------------------

void AdvertisePayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteUInt8(static_cast<uint8_t>(direction));
    w.WriteString(topic);
    w.WriteString(type_name);
    w.WriteUInt8(static_cast<uint8_t>(qos.reliability));
    w.WriteUInt8(static_cast<uint8_t>(qos.durability));
    w.WriteUInt32(qos.depth);
}

AdvertisePayload AdvertisePayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    AdvertisePayload p;
    const uint8_t direction = r.ReadUInt8();
    if (direction > static_cast<uint8_t>(Direction::Subscribe)) {
        throw CdrError("invalid Advertise direction " + std::to_string(direction));
    }
    p.direction = static_cast<Direction>(direction);
    p.topic = r.ReadString();
    p.type_name = r.ReadString();
    const uint8_t reliability = r.ReadUInt8();
    const uint8_t durability = r.ReadUInt8();
    if (reliability > static_cast<uint8_t>(Reliability::BestEffort) ||
        durability > static_cast<uint8_t>(Durability::TransientLocal)) {
        throw CdrError("invalid QoS in Advertise payload");
    }
    p.qos.reliability = static_cast<Reliability>(reliability);
    p.qos.durability = static_cast<Durability>(durability);
    p.qos.depth = r.ReadUInt32();
    return p;
}

// ----------------------------------------------------------------------------

void AdvertiseAckPayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteBool(ok);
    w.WriteString(error);
}

AdvertiseAckPayload AdvertiseAckPayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    AdvertiseAckPayload p;
    p.ok = r.ReadBool();
    p.error = r.ReadString();
    return p;
}

// ----------------------------------------------------------------------------

void ChannelInfoPayload::Encode(std::vector<uint8_t>& out) const {
    CdrWriter w(out, false);
    w.WriteUInt32(matched_endpoints);
}

ChannelInfoPayload ChannelInfoPayload::Decode(const uint8_t* data, size_t size) {
    CdrReader r(data, size, false);
    ChannelInfoPayload p;
    p.matched_endpoints = r.ReadUInt32();
    return p;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
