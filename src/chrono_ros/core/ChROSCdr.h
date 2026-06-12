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
// Minimal CDR (Common Data Representation) codec for ROS 2 serialized messages.
//
// ROS 2 messages travel on the wire as XCDR1 "plain CDR", little-endian, with a
// 4-byte encapsulation header. All ROS 2 message types are declared @final and
// are therefore serialized with these classic CDR rules:
//   - primitives are aligned to their own size (max 8), where alignment is
//     computed relative to the first byte AFTER the encapsulation header
//   - string: uint32 length (including NUL) + bytes + NUL
//   - unbounded/bounded sequence: uint32 element count + aligned elements
//   - fixed-size array: elements only (no count)
//   - nested message: its fields in order
//
// This codec has zero dependencies (no ROS, no Chrono). Correctness against the
// actual middleware is enforced at runtime by the VALIDATE_TYPE handshake (see
// CLAUDE.md section 6.2), which round-trips a sample through the real rmw
// serialization support and byte-compares.
//
// =============================================================================

#ifndef CH_ROS_CORE_CDR_H
#define CH_ROS_CORE_CDR_H

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace chrono {
namespace ros {
namespace core {

/// Error thrown on malformed CDR input or invalid codec usage.
class CdrError : public std::runtime_error {
  public:
    explicit CdrError(const std::string& what) : std::runtime_error("CDR error: " + what) {}
};

/// True if the host is little-endian. The codec (and the IPC protocol as a
/// whole) supports little-endian hosts only; this is checked once at bridge
/// initialization. All platforms Chrono targets (x86-64, AArch64) qualify.
bool IsHostLittleEndian();

/// CDR encapsulation header: representation identifier CDR_LE + zeroed options.
constexpr uint8_t kCdrEncapsulation[4] = {0x00, 0x01, 0x00, 0x00};
constexpr size_t kCdrEncapsulationSize = 4;

// -----------------------------------------------------------------------------

/// Appends CDR-encoded data to a caller-owned byte vector.
///
/// The writer appends starting at the vector's current size; existing contents
/// are preserved (this lets a frame header or other prefix precede the body).
class CdrWriter {
  public:
    /// @param out buffer to append to (not owned; must outlive the writer)
    /// @param with_encapsulation if true, the 4-byte CDR_LE encapsulation
    ///        header is written first and alignment is computed after it (the
    ///        ROS 2 wire format). If false, alignment starts at the writer's
    ///        first byte (used for the bridge's internal control payloads).
    explicit CdrWriter(std::vector<uint8_t>& out, bool with_encapsulation = true);

    void WriteBool(bool v);
    void WriteUInt8(uint8_t v);
    void WriteInt8(int8_t v);
    void WriteUInt16(uint16_t v);
    void WriteInt16(int16_t v);
    void WriteUInt32(uint32_t v);
    void WriteInt32(int32_t v);
    void WriteUInt64(uint64_t v);
    void WriteInt64(int64_t v);
    void WriteFloat32(float v);
    void WriteFloat64(double v);

    /// CDR string: aligned uint32 length (strlen + 1 for NUL) + bytes + NUL.
    /// Embedded NUL characters are rejected (ROS strings are NUL-terminated on
    /// the wire; embedded NULs would be silently truncated by consumers).
    void WriteString(const std::string& v);

    /// Raw bytes, no alignment, no length prefix. Used for pre-packed
    /// little-endian primitive arrays (bulk sensor payloads) after the caller
    /// has aligned for the element type, and for nested pre-encoded blobs.
    void WriteBytes(const void* data, size_t size);

    /// Length-prefixed byte blob (uint32 size + raw bytes). Convenience for
    /// control payloads carrying opaque data (schema blobs, CDR samples).
    void WriteBlob(const void* data, size_t size);

    /// Insert zero padding so the next write lands on the given alignment
    /// (1, 2, 4 or 8) relative to the encapsulation origin.
    void Align(size_t alignment);

    /// Bytes written by this writer so far (excluding pre-existing contents).
    size_t BytesWritten() const { return m_out.size() - m_start; }

  private:
    template <typename T>
    void WriteScalar(T v);

    std::vector<uint8_t>& m_out;
    size_t m_start;   ///< out.size() at construction
    size_t m_origin;  ///< alignment origin (start of CDR body)
};

// -----------------------------------------------------------------------------

/// Bounds-checked reader over a CDR-encoded byte range. Does not own the data.
class CdrReader {
  public:
    /// @param data start of the CDR data (encapsulation header first, if any)
    /// @param size total size in bytes
    /// @param with_encapsulation if true, a 4-byte encapsulation header is
    ///        validated (must be CDR little-endian) and alignment is computed
    ///        after it. Big-endian input is rejected with a clear error.
    CdrReader(const uint8_t* data, size_t size, bool with_encapsulation = true);

    bool ReadBool();
    uint8_t ReadUInt8();
    int8_t ReadInt8();
    uint16_t ReadUInt16();
    int16_t ReadInt16();
    uint32_t ReadUInt32();
    int32_t ReadInt32();
    uint64_t ReadUInt64();
    int64_t ReadInt64();
    float ReadFloat32();
    double ReadFloat64();
    std::string ReadString();

    /// Read raw bytes (no alignment, no length prefix).
    void ReadBytes(void* out, size_t size);

    /// Read a length-prefixed byte blob written by CdrWriter::WriteBlob.
    std::vector<uint8_t> ReadBlob();

    /// Advance to the given alignment relative to the encapsulation origin.
    void Align(size_t alignment);

    /// Skip forward without copying (bounds-checked).
    void Skip(size_t size);

    /// Current read position (absolute offset from the start of the data).
    size_t Position() const { return m_pos; }

    /// Bytes remaining.
    size_t Remaining() const { return m_size - m_pos; }

    /// Pointer to the current position (valid for Remaining() bytes). Used for
    /// zero-copy access to bulk primitive arrays.
    const uint8_t* Current() const { return m_data + m_pos; }

  private:
    template <typename T>
    T ReadScalar();

    void Require(size_t bytes) const;

    const uint8_t* m_data;
    size_t m_size;
    size_t m_pos;
    size_t m_origin;
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
