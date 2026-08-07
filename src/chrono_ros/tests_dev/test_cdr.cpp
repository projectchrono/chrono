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
// Dev-time tests for the CDR codec: golden-byte checks of the wire format
// (alignment, encapsulation, strings) and round trips of every scalar.
//
// =============================================================================

#include "chrono_ros/core/ChROSCdr.h"
#include "ChROSTestUtils.h"

#include <cmath>
#include <limits>

using namespace chrono::ros::core;

TEST(host_is_little_endian) {
    // Protocol-wide precondition; if this fails the platform is unsupported.
    CHECK(IsHostLittleEndian());
}

TEST(encapsulation_header_golden) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteUInt8(0xAB);
    chros_test::CheckBytes(out, {0x00, 0x01, 0x00, 0x00, 0xAB}, "encapsulation");
}

TEST(alignment_is_relative_to_body_not_buffer) {
    // Pre-existing bytes in the output buffer must not shift CDR alignment.
    std::vector<uint8_t> out = {0xEE, 0xEE, 0xEE};  // 3 bytes of unrelated prefix
    CdrWriter writer(out);
    writer.WriteUInt32(0x01020304);  // first body byte => no padding despite odd buffer offset
    chros_test::CheckBytes(out, {0xEE, 0xEE, 0xEE, 0x00, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01}, "prefix");
}

TEST(bool_then_double_padding_golden) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteBool(true);
    writer.WriteFloat64(1.5);  // 1.5 == 0x3FF8000000000000
    chros_test::CheckBytes(out,
                           {0x00, 0x01, 0x00, 0x00,                          // encapsulation
                            0x01,                                            // bool
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       // 7 bytes padding to align 8
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F},
                           "bool+double");
}

TEST(string_golden) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteString("base");
    chros_test::CheckBytes(out,
                           {0x00, 0x01, 0x00, 0x00,        // encapsulation
                            0x05, 0x00, 0x00, 0x00,        // length 5 (includes NUL)
                            'b', 'a', 's', 'e', 0x00},
                           "string");
}

TEST(string_alignment_after_string) {
    // A uint32 following a string must re-align (string ends unaligned).
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteString("ab");          // 4 (len) + 3 bytes => body offset 7
    writer.WriteUInt32(0x11223344);    // needs 1 byte padding to offset 8
    chros_test::CheckBytes(out,
                           {0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 'a', 'b', 0x00,
                            0x00,  // padding
                            0x44, 0x33, 0x22, 0x11},
                           "string+uint32");
}

TEST(all_scalars_roundtrip) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteBool(true);
    writer.WriteInt8(-12);
    writer.WriteUInt8(200);
    writer.WriteInt16(-30000);
    writer.WriteUInt16(60000);
    writer.WriteInt32(-2000000000);
    writer.WriteUInt32(4000000000u);
    writer.WriteInt64(std::numeric_limits<int64_t>::min());
    writer.WriteUInt64(std::numeric_limits<uint64_t>::max());
    writer.WriteFloat32(3.25f);
    writer.WriteFloat64(-2.625);
    writer.WriteString("hello world");
    writer.WriteString("");  // empty string is len=1 + NUL

    CdrReader reader(out.data(), out.size());
    CHECK_EQ(reader.ReadBool(), true);
    CHECK_EQ(reader.ReadInt8(), -12);
    CHECK_EQ(reader.ReadUInt8(), 200);
    CHECK_EQ(reader.ReadInt16(), -30000);
    CHECK_EQ(reader.ReadUInt16(), 60000);
    CHECK_EQ(reader.ReadInt32(), -2000000000);
    CHECK_EQ(reader.ReadUInt32(), 4000000000u);
    CHECK_EQ(reader.ReadInt64(), std::numeric_limits<int64_t>::min());
    CHECK_EQ(reader.ReadUInt64(), std::numeric_limits<uint64_t>::max());
    CHECK_EQ(reader.ReadFloat32(), 3.25f);
    CHECK_EQ(reader.ReadFloat64(), -2.625);
    CHECK_EQ(reader.ReadString(), "hello world");
    CHECK_EQ(reader.ReadString(), "");
    CHECK_EQ(reader.Remaining(), size_t(0));
}

TEST(blob_roundtrip) {
    std::vector<uint8_t> out;
    CdrWriter writer(out, false);
    const std::vector<uint8_t> blob = {1, 2, 3, 4, 5};
    writer.WriteBlob(blob.data(), blob.size());
    writer.WriteBlob(nullptr, 0);

    CdrReader reader(out.data(), out.size(), false);
    CHECK(reader.ReadBlob() == blob);
    CHECK(reader.ReadBlob().empty());
}

TEST(no_encapsulation_mode) {
    std::vector<uint8_t> out;
    CdrWriter writer(out, false);
    writer.WriteUInt16(0x0102);
    chros_test::CheckBytes(out, {0x02, 0x01}, "no-encapsulation");
}

TEST(reader_rejects_big_endian) {
    const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x01};  // CDR_BE identifier
    CHECK_THROWS(CdrReader(data, sizeof(data)), CdrError, "little-endian");
}

TEST(reader_rejects_truncation) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    writer.WriteUInt64(42);
    CdrReader reader(out.data(), out.size() - 1);
    CHECK_THROWS(reader.ReadUInt64(), CdrError, "truncated");
}

TEST(reader_rejects_string_without_nul) {
    std::vector<uint8_t> data = {0x02, 0x00, 0x00, 0x00, 'a', 'b'};  // len 2, no NUL
    CdrReader reader(data.data(), data.size(), false);
    CHECK_THROWS(reader.ReadString(), CdrError, "NUL");
}

TEST(reader_tolerates_zero_length_string) {
    std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00};
    CdrReader reader(data.data(), data.size(), false);
    CHECK_EQ(reader.ReadString(), "");
}

TEST(writer_rejects_embedded_nul) {
    std::vector<uint8_t> out;
    CdrWriter writer(out);
    CHECK_THROWS(writer.WriteString(std::string("a\0b", 3)), CdrError, "NUL");
}

TEST(skip_and_position) {
    std::vector<uint8_t> out;
    CdrWriter writer(out, false);
    writer.WriteUInt32(1);
    writer.WriteUInt32(2);
    CdrReader reader(out.data(), out.size(), false);
    reader.Skip(4);
    CHECK_EQ(reader.ReadUInt32(), 2u);
    CHECK_THROWS(reader.Skip(1), CdrError, "truncated");
}

int main() {
    return chros_test::RunAll("cdr");
}
