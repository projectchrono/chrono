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
// Dev-time tests for the Schema model and its blob round trip.
//
// =============================================================================

#include "chrono_ros/core/ChROSCdr.h"
#include "chrono_ros/core/ChROSSchema.h"
#include "ChROSTestUtils.h"

using namespace chrono::ros::core;

namespace {

/// sensor_msgs/msg/Imu-shaped schema (header + nested quaternion/vector3 +
/// fixed covariance arrays) - exercises nesting and fixed arrays.
Schema BuildImuLikeSchema() {
    Schema schema;
    const int32_t time = schema.AddType("builtin_interfaces/msg/Time");
    schema.AddField(time, {"sec", FieldKind::Int32, ArrayKind::None, 0, -1});
    schema.AddField(time, {"nanosec", FieldKind::UInt32, ArrayKind::None, 0, -1});

    const int32_t header = schema.AddType("std_msgs/msg/Header");
    schema.AddField(header, {"stamp", FieldKind::Message, ArrayKind::None, 0, time});
    schema.AddField(header, {"frame_id", FieldKind::String, ArrayKind::None, 0, -1});

    const int32_t quat = schema.AddType("geometry_msgs/msg/Quaternion");
    for (const char* axis : {"x", "y", "z", "w"}) {
        schema.AddField(quat, {axis, FieldKind::Float64, ArrayKind::None, 0, -1});
    }

    const int32_t vec3 = schema.AddType("geometry_msgs/msg/Vector3");
    for (const char* axis : {"x", "y", "z"}) {
        schema.AddField(vec3, {axis, FieldKind::Float64, ArrayKind::None, 0, -1});
    }

    const int32_t imu = schema.AddType("sensor_msgs/msg/Imu");
    schema.AddField(imu, {"header", FieldKind::Message, ArrayKind::None, 0, header});
    schema.AddField(imu, {"orientation", FieldKind::Message, ArrayKind::None, 0, quat});
    schema.AddField(imu, {"orientation_covariance", FieldKind::Float64, ArrayKind::FixedArray, 9, -1});
    schema.AddField(imu, {"angular_velocity", FieldKind::Message, ArrayKind::None, 0, vec3});
    schema.AddField(imu, {"angular_velocity_covariance", FieldKind::Float64, ArrayKind::FixedArray, 9, -1});
    schema.AddField(imu, {"linear_acceleration", FieldKind::Message, ArrayKind::None, 0, vec3});
    schema.AddField(imu, {"linear_acceleration_covariance", FieldKind::Float64, ArrayKind::FixedArray, 9, -1});
    schema.Finalize(imu);
    return schema;
}

}  // namespace

TEST(blob_roundtrip_preserves_everything) {
    const Schema original = BuildImuLikeSchema();
    const std::vector<uint8_t> blob = original.EncodeBlob();
    const auto decoded = Schema::DecodeBlob(blob.data(), blob.size());

    CHECK_EQ(decoded->TypeCount(), original.TypeCount());
    CHECK_EQ(decoded->RootIndex(), original.RootIndex());
    CHECK_EQ(decoded->Root().name, std::string("sensor_msgs/msg/Imu"));
    for (size_t t = 0; t < original.TypeCount(); t++) {
        const TypeRecord& a = original.At(static_cast<int32_t>(t));
        const TypeRecord& b = decoded->At(static_cast<int32_t>(t));
        CHECK_EQ(a.name, b.name);
        CHECK_EQ(a.fields.size(), b.fields.size());
        for (size_t f = 0; f < a.fields.size(); f++) {
            CHECK_EQ(a.fields[f].name, b.fields[f].name);
            CHECK(a.fields[f].kind == b.fields[f].kind);
            CHECK(a.fields[f].array_kind == b.fields[f].array_kind);
            CHECK_EQ(a.fields[f].array_size, b.fields[f].array_size);
            CHECK_EQ(a.fields[f].nested_type, b.fields[f].nested_type);
        }
    }
    // The blob encoding must be deterministic.
    CHECK(decoded->EncodeBlob() == blob);
}

TEST(find_type) {
    const Schema schema = BuildImuLikeSchema();
    CHECK(schema.FindType("std_msgs/msg/Header") >= 0);
    CHECK_EQ(schema.FindType("not/a/Type"), -1);
}

TEST(to_string_is_msg_like) {
    const std::string text = BuildImuLikeSchema().ToString();
    CHECK(text.find("sensor_msgs/msg/Imu") != std::string::npos);
    CHECK(text.find("float64[9] orientation_covariance") != std::string::npos);
    CHECK(text.find("string frame_id") != std::string::npos);
}

TEST(rejects_duplicate_type) {
    Schema schema;
    schema.AddType("a/msg/A");
    CHECK_THROWS(schema.AddType("a/msg/A"), SchemaError, "duplicate");
}

TEST(rejects_bad_nested_reference) {
    Schema schema;
    const int32_t a = schema.AddType("a/msg/A");
    schema.AddField(a, {"child", FieldKind::Message, ArrayKind::None, 0, 7});  // no type 7
    CHECK_THROWS(schema.Finalize(a), SchemaError, "unknown nested type");
}

TEST(rejects_fixed_array_of_zero) {
    Schema schema;
    const int32_t a = schema.AddType("a/msg/A");
    schema.AddField(a, {"arr", FieldKind::Float64, ArrayKind::FixedArray, 0, -1});
    CHECK_THROWS(schema.Finalize(a), SchemaError, "non-zero size");
}

TEST(rejects_unfinalized_encode) {
    Schema schema;
    schema.AddType("a/msg/A");
    CHECK_THROWS(schema.EncodeBlob(), SchemaError, "finalized");
}

TEST(decode_rejects_version_mismatch) {
    const Schema schema = BuildImuLikeSchema();
    std::vector<uint8_t> blob = schema.EncodeBlob();
    blob[0] = 0xFF;  // clobber the version field
    CHECK_THROWS(Schema::DecodeBlob(blob.data(), blob.size()), SchemaError, "version mismatch");
}

TEST(decode_rejects_trailing_garbage) {
    const Schema schema = BuildImuLikeSchema();
    std::vector<uint8_t> blob = schema.EncodeBlob();
    blob.push_back(0);
    CHECK_THROWS(Schema::DecodeBlob(blob.data(), blob.size()), SchemaError, "trailing");
}

TEST(decode_rejects_truncation) {
    const Schema schema = BuildImuLikeSchema();
    std::vector<uint8_t> blob = schema.EncodeBlob();
    blob.resize(blob.size() / 2);
    CHECK_THROWS(Schema::DecodeBlob(blob.data(), blob.size()), CdrError, "truncated");
}

int main() {
    return chros_test::RunAll("schema");
}
