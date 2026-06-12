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
// Dev-time tests for MessageBuilder/MessageReader: golden wire bytes for
// hand-computed cases, full round trips for sensor-shaped messages (Image,
// PointCloud2, JointState lookalikes), defaults, and error reporting.
//
// =============================================================================

#include "chrono_ros/core/ChROSCdr.h"
#include "chrono_ros/core/ChROSMessageCodec.h"
#include "chrono_ros/core/ChROSSchema.h"
#include "ChROSTestUtils.h"

#include <cmath>
#include <cstring>

using namespace chrono::ros::core;

namespace {

std::shared_ptr<const Schema> Vector3Schema() {
    auto schema = std::make_shared<Schema>();
    const int32_t vec3 = schema->AddType("geometry_msgs/msg/Vector3");
    for (const char* axis : {"x", "y", "z"}) {
        schema->AddField(vec3, {axis, FieldKind::Float64, ArrayKind::None, 0, -1});
    }
    schema->Finalize(vec3);
    return schema;
}

std::shared_ptr<const Schema> HeaderSchema() {
    auto schema = std::make_shared<Schema>();
    const int32_t time = schema->AddType("builtin_interfaces/msg/Time");
    schema->AddField(time, {"sec", FieldKind::Int32, ArrayKind::None, 0, -1});
    schema->AddField(time, {"nanosec", FieldKind::UInt32, ArrayKind::None, 0, -1});
    const int32_t header = schema->AddType("std_msgs/msg/Header");
    schema->AddField(header, {"stamp", FieldKind::Message, ArrayKind::None, 0, time});
    schema->AddField(header, {"frame_id", FieldKind::String, ArrayKind::None, 0, -1});
    schema->Finalize(header);
    return schema;
}

/// sensor_msgs/msg/Image lookalike (real field list).
std::shared_ptr<const Schema> ImageSchema() {
    auto schema = std::make_shared<Schema>();
    const int32_t time = schema->AddType("builtin_interfaces/msg/Time");
    schema->AddField(time, {"sec", FieldKind::Int32, ArrayKind::None, 0, -1});
    schema->AddField(time, {"nanosec", FieldKind::UInt32, ArrayKind::None, 0, -1});
    const int32_t header = schema->AddType("std_msgs/msg/Header");
    schema->AddField(header, {"stamp", FieldKind::Message, ArrayKind::None, 0, time});
    schema->AddField(header, {"frame_id", FieldKind::String, ArrayKind::None, 0, -1});
    const int32_t image = schema->AddType("sensor_msgs/msg/Image");
    schema->AddField(image, {"header", FieldKind::Message, ArrayKind::None, 0, header});
    schema->AddField(image, {"height", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(image, {"width", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(image, {"encoding", FieldKind::String, ArrayKind::None, 0, -1});
    schema->AddField(image, {"is_bigendian", FieldKind::UInt8, ArrayKind::None, 0, -1});
    schema->AddField(image, {"step", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(image, {"data", FieldKind::UInt8, ArrayKind::UnboundedSequence, 0, -1});
    schema->Finalize(image);
    return schema;
}

/// sensor_msgs/msg/PointCloud2 lookalike: sequence of nested PointField.
std::shared_ptr<const Schema> PointCloud2Schema() {
    auto schema = std::make_shared<Schema>();
    const int32_t time = schema->AddType("builtin_interfaces/msg/Time");
    schema->AddField(time, {"sec", FieldKind::Int32, ArrayKind::None, 0, -1});
    schema->AddField(time, {"nanosec", FieldKind::UInt32, ArrayKind::None, 0, -1});
    const int32_t header = schema->AddType("std_msgs/msg/Header");
    schema->AddField(header, {"stamp", FieldKind::Message, ArrayKind::None, 0, time});
    schema->AddField(header, {"frame_id", FieldKind::String, ArrayKind::None, 0, -1});
    const int32_t point_field = schema->AddType("sensor_msgs/msg/PointField");
    schema->AddField(point_field, {"name", FieldKind::String, ArrayKind::None, 0, -1});
    schema->AddField(point_field, {"offset", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(point_field, {"datatype", FieldKind::UInt8, ArrayKind::None, 0, -1});
    schema->AddField(point_field, {"count", FieldKind::UInt32, ArrayKind::None, 0, -1});
    const int32_t cloud = schema->AddType("sensor_msgs/msg/PointCloud2");
    schema->AddField(cloud, {"header", FieldKind::Message, ArrayKind::None, 0, header});
    schema->AddField(cloud, {"height", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(cloud, {"width", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(cloud, {"fields", FieldKind::Message, ArrayKind::UnboundedSequence, 0, point_field});
    schema->AddField(cloud, {"is_bigendian", FieldKind::Bool, ArrayKind::None, 0, -1});
    schema->AddField(cloud, {"point_step", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(cloud, {"row_step", FieldKind::UInt32, ArrayKind::None, 0, -1});
    schema->AddField(cloud, {"data", FieldKind::UInt8, ArrayKind::UnboundedSequence, 0, -1});
    schema->AddField(cloud, {"is_dense", FieldKind::Bool, ArrayKind::None, 0, -1});
    schema->Finalize(cloud);
    return schema;
}

/// sensor_msgs/msg/JointState lookalike: string sequence + float64 sequences.
std::shared_ptr<const Schema> JointStateSchema() {
    auto schema = std::make_shared<Schema>();
    const int32_t js = schema->AddType("sensor_msgs/msg/JointState");
    schema->AddField(js, {"name", FieldKind::String, ArrayKind::UnboundedSequence, 0, -1});
    schema->AddField(js, {"position", FieldKind::Float64, ArrayKind::UnboundedSequence, 0, -1});
    schema->AddField(js, {"effort_limit", FieldKind::Float64, ArrayKind::FixedArray, 3, -1});
    schema->Finalize(js);
    return schema;
}

}  // namespace

// ----------------------------------------------------------------------------
// golden wire bytes
// ----------------------------------------------------------------------------

TEST(vector3_golden_with_default_z) {
    MessageBuilder msg(Vector3Schema());
    msg.SetDouble("x", 1.5);    // 0x3FF8000000000000
    msg.SetDouble("y", -2.0);   // 0xC000000000000000
    // z left unset -> serialized as 0.0

    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    chros_test::CheckBytes(out,
                           {0x00, 0x01, 0x00, 0x00,                                          // encapsulation
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F,                  // x = 1.5
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,                  // y = -2.0
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},                 // z = 0.0
                           "vector3");
}

TEST(header_golden) {
    MessageBuilder msg(HeaderSchema());
    msg.SetInt("stamp.sec", 7);
    msg.SetUInt("stamp.nanosec", 500000000);  // 0x1DCD6500
    msg.SetString("frame_id", "base");

    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    chros_test::CheckBytes(out,
                           {0x00, 0x01, 0x00, 0x00,        // encapsulation
                            0x07, 0x00, 0x00, 0x00,        // sec
                            0x00, 0x65, 0xCD, 0x1D,        // nanosec
                            0x05, 0x00, 0x00, 0x00,        // frame_id length (incl. NUL)
                            'b', 'a', 's', 'e', 0x00},
                           "header");
}

TEST(set_time_splits_and_carries) {
    MessageBuilder msg(HeaderSchema());
    msg.SetTime("stamp", 1.9999999999);  // rounds to 2.0 exactly: carry into sec
    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    MessageReader reader(HeaderSchema(), out.data(), out.size());
    // Schemas are value-compared via the same builder schema; re-read fields:
    CHECK_EQ(reader.GetInt("stamp.sec"), 2);
    CHECK_EQ(reader.GetUInt("stamp.nanosec"), 0u);

    MessageBuilder msg2(HeaderSchema());
    msg2.SetTime("stamp", 7.5);
    std::vector<uint8_t> out2;
    msg2.SerializeTo(out2);
    MessageReader reader2(HeaderSchema(), out2.data(), out2.size());
    CHECK_EQ(reader2.GetInt("stamp.sec"), 7);
    CHECK_EQ(reader2.GetUInt("stamp.nanosec"), 500000000u);
    CHECK_EQ(reader2.GetTimeSec("stamp"), 7.5);
}

// ----------------------------------------------------------------------------
// sensor-shaped round trips
// ----------------------------------------------------------------------------

TEST(image_blob_roundtrip) {
    auto schema = ImageSchema();
    MessageBuilder msg(schema);
    msg.SetTime("header.stamp", 12.25);
    msg.SetString("header.frame_id", "camera_link");
    msg.SetInt("height", 4);
    msg.SetInt("width", 6);
    msg.SetString("encoding", "rgba8");
    msg.SetInt("step", 24);

    std::vector<uint8_t> pixels(4 * 6 * 4);
    for (size_t i = 0; i < pixels.size(); i++) {
        pixels[i] = static_cast<uint8_t>(i * 7);
    }
    msg.SetBlob("data", pixels.data(), pixels.size());  // zero-copy

    std::vector<uint8_t> out;
    msg.SerializeTo(out);

    MessageReader reader(schema, out.data(), out.size());
    CHECK_EQ(reader.GetString("header.frame_id"), "camera_link");
    CHECK_EQ(reader.GetTimeSec("header.stamp"), 12.25);
    CHECK_EQ(reader.GetUInt("height"), 4u);
    CHECK_EQ(reader.GetUInt("width"), 6u);
    CHECK_EQ(reader.GetString("encoding"), "rgba8");
    CHECK_EQ(reader.GetUInt("is_bigendian"), 0u);  // default
    CHECK_EQ(reader.GetUInt("step"), 24u);

    const BlobView view = reader.GetBlob("data");
    CHECK_EQ(view.count, pixels.size());
    CHECK_EQ(view.element_size, size_t(1));
    CHECK(std::memcmp(view.data, pixels.data(), pixels.size()) == 0);
}

TEST(pointcloud2_nested_message_sequence) {
    auto schema = PointCloud2Schema();
    MessageBuilder msg(schema);
    msg.SetString("header.frame_id", "lidar");
    msg.SetInt("height", 1);
    msg.SetInt("width", 3);
    msg.SetBool("is_dense", true);
    msg.SetInt("point_step", 16);
    msg.SetInt("row_step", 48);

    const char* names[] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; i++) {
        MessageBuilder& field = msg.AppendMessage("fields");
        field.SetString("name", names[i]);
        field.SetInt("offset", i * 4);
        field.SetInt("datatype", 7);  // FLOAT32
        field.SetInt("count", 1);
    }

    std::vector<float> points = {1.0f, 2.0f, 3.0f, 0.5f,  4.0f, 5.0f, 6.0f, 0.6f,  7.0f, 8.0f, 9.0f, 0.7f};
    msg.SetBlob("data", points.data(), points.size() * sizeof(float));  // uint8[] field: count is bytes

    std::vector<uint8_t> out;
    msg.SerializeTo(out);

    MessageReader reader(schema, out.data(), out.size());
    CHECK_EQ(reader.GetCount("fields"), size_t(4));
    for (uint32_t i = 0; i < 4; i++) {
        const MessageReader field = reader.GetMessage("fields", i);
        CHECK_EQ(field.GetString("name"), names[i]);
        CHECK_EQ(field.GetUInt("offset"), i * 4);
        CHECK_EQ(field.GetUInt("datatype"), 7u);
    }
    CHECK_EQ(reader.GetBool("is_dense"), true);
    const BlobView data = reader.GetBlob("data");
    CHECK_EQ(data.count, points.size() * sizeof(float));
    CHECK(std::memcmp(data.data, points.data(), data.count) == 0);
}

TEST(jointstate_string_and_float_sequences) {
    auto schema = JointStateSchema();
    MessageBuilder msg(schema);
    msg.SetStringArray("name", {"shoulder", "elbow", "wrist"});
    const double positions[] = {0.1, -0.2, 0.3};
    msg.SetBlob("position", positions, 3);

    std::vector<uint8_t> out;
    msg.SerializeTo(out);

    MessageReader reader(schema, out.data(), out.size());
    const auto names = reader.GetStringArray("name");
    CHECK_EQ(names.size(), size_t(3));
    CHECK_EQ(names[1], "elbow");
    const BlobView pos = reader.GetBlob("position");
    CHECK_EQ(pos.count, size_t(3));
    CHECK_EQ(pos.element_size, size_t(8));
    double readback[3];
    std::memcpy(readback, pos.data, sizeof(readback));
    CHECK_EQ(readback[2], 0.3);
    // Unset fixed array serializes as zeros.
    const BlobView limits = reader.GetBlob("effort_limit");
    CHECK_EQ(limits.count, size_t(3));
    double zeros[3];
    std::memcpy(zeros, limits.data, sizeof(zeros));
    CHECK_EQ(zeros[0], 0.0);
    CHECK_EQ(zeros[2], 0.0);
}

TEST(fully_default_message_parses) {
    auto schema = PointCloud2Schema();
    MessageBuilder msg(schema);
    std::vector<uint8_t> out;
    msg.SerializeTo(out);

    MessageReader reader(schema, out.data(), out.size());
    CHECK_EQ(reader.GetString("header.frame_id"), "");
    CHECK_EQ(reader.GetCount("fields"), size_t(0));
    CHECK_EQ(reader.GetBool("is_dense"), false);
    CHECK_EQ(reader.GetBlob("data").count, size_t(0));
}

TEST(clear_resets_values) {
    MessageBuilder msg(Vector3Schema());
    msg.SetDouble("x", 5.0);
    msg.Clear();
    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    MessageReader reader(Vector3Schema(), out.data(), out.size());
    CHECK_EQ(reader.GetDouble("x"), 0.0);
}

// ----------------------------------------------------------------------------
// error reporting
// ----------------------------------------------------------------------------

TEST(unknown_field_lists_available) {
    MessageBuilder msg(HeaderSchema());
    CHECK_THROWS(msg.SetString("frame", "x"), FieldError, "available fields: stamp, frame_id");
}

TEST(kind_mismatch_is_descriptive) {
    MessageBuilder msg(HeaderSchema());
    CHECK_THROWS(msg.SetString("stamp.sec", "x"), FieldError, "int32");
    CHECK_THROWS(msg.SetDouble("frame_id", 1.0), FieldError, "string");
}

TEST(integer_range_is_checked) {
    auto schema = ImageSchema();
    MessageBuilder msg(schema);
    CHECK_THROWS(msg.SetInt("is_bigendian", 300), FieldError, "out of range");
    CHECK_THROWS(msg.SetInt("height", -1), FieldError, "out of range");
}

TEST(double_refused_on_integer_field) {
    MessageBuilder msg(HeaderSchema());
    CHECK_THROWS(msg.SetDouble("stamp.sec", 1.5), FieldError, "SetDouble applies to float scalars");
}

TEST(fixed_array_size_enforced) {
    auto schema = JointStateSchema();
    MessageBuilder msg(schema);
    const double two[] = {1.0, 2.0};
    CHECK_THROWS(msg.SetBlob("effort_limit", two, 2), FieldError, "fixed array of 3");
}

TEST(descend_through_scalar_fails) {
    MessageBuilder msg(HeaderSchema());
    CHECK_THROWS(msg.SetInt("frame_id.sec", 1), FieldError, "cannot descend");
}

TEST(append_on_singular_fails) {
    auto schema = PointCloud2Schema();
    MessageBuilder msg(schema);
    CHECK_THROWS(msg.AppendMessage("header"), FieldError, "not a message array");
}

TEST(reader_index_out_of_range) {
    auto schema = PointCloud2Schema();
    MessageBuilder msg(schema);
    msg.AppendMessage("fields").SetString("name", "x");
    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    MessageReader reader(schema, out.data(), out.size());
    CHECK_THROWS(reader.GetMessage("fields", 1), FieldError, "out of range");
}

TEST(reader_unknown_field_lists_available) {
    MessageBuilder msg(HeaderSchema());
    std::vector<uint8_t> out;
    msg.SerializeTo(out);
    MessageReader reader(HeaderSchema(), out.data(), out.size());
    CHECK_THROWS(reader.GetString("frame"), FieldError, "available fields");
}

int main() {
    return chros_test::RunAll("message");
}
