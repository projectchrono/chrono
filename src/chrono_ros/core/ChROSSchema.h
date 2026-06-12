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
// Runtime description of a ROS 2 message type ("schema").
//
// A Schema is produced in the chrono_ros_node subprocess by walking the
// rosidl introspection typesupport of a message type, then shipped to the
// simulation process as a compact binary blob (the TYPE_SCHEMA payload).
// The simulation process uses it to serialize/deserialize CDR without any
// ROS code or compiled message types (see CLAUDE.md section 6).
//
// The blob format is internal to chrono_ros: both endpoints are always built
// from this same header, so it carries a version field for sanity checking
// but makes no cross-version compatibility promises.
//
// =============================================================================

#ifndef CH_ROS_CORE_SCHEMA_H
#define CH_ROS_CORE_SCHEMA_H

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace chrono {
namespace ros {
namespace core {

/// Error thrown on invalid schema construction or a malformed schema blob.
class SchemaError : public std::runtime_error {
  public:
    explicit SchemaError(const std::string& what) : std::runtime_error("schema error: " + what) {}
};

/// Field value categories, mirroring the rosidl type system.
enum class FieldKind : uint8_t {
    Bool = 1,
    Octet,    ///< rosidl 'byte'/'octet'
    Char,     ///< rosidl 'char' (uint8 on the wire)
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Float32,
    Float64,
    String,
    WString,  ///< recognized but not serializable in v1 (no real-world messages use it)
    Message,  ///< nested message type; see FieldRecord::nested_type
};

/// Whether a field is scalar, a fixed array, or a sequence.
enum class ArrayKind : uint8_t {
    None = 0,           ///< single value
    FixedArray = 1,     ///< exactly array_size elements, no count on the wire
    BoundedSequence,    ///< uint32 count + elements; count <= array_size
    UnboundedSequence,  ///< uint32 count + elements
};

/// True for kinds with a fixed wire size (everything except String/WString/Message).
bool IsPrimitive(FieldKind kind);

/// Wire size and alignment of a primitive kind (1, 2, 4 or 8). Throws for
/// non-primitive kinds.
size_t PrimitiveSize(FieldKind kind);

/// Human-readable name of a kind ("float64", "string", ...).
const char* FieldKindName(FieldKind kind);

/// One field of a message type.
struct FieldRecord {
    std::string name;
    FieldKind kind = FieldKind::Bool;
    ArrayKind array_kind = ArrayKind::None;
    uint32_t array_size = 0;      ///< FixedArray: exact size; BoundedSequence: upper bound; else 0
    int32_t nested_type = -1;     ///< index into Schema's type table when kind == Message
};

/// One message type: its full ROS name and ordered fields.
struct TypeRecord {
    std::string name;  ///< e.g. "sensor_msgs/msg/Image"
    std::vector<FieldRecord> fields;
};

/// A complete, self-contained description of one root message type and every
/// type nested within it. Immutable once built (treat as such after Finalize).
class Schema {
  public:
    static constexpr uint16_t kBlobVersion = 1;

    Schema() = default;

    // --- construction (used by the subprocess introspection walker and tests)

    /// Add a type and return its index. Names must be unique.
    int32_t AddType(const std::string& type_name);

    /// Append a field to a previously added type.
    void AddField(int32_t type_index, FieldRecord field);

    /// Designate the root type and validate the whole schema: every nested
    /// reference must resolve, the root must be set, names must be non-empty.
    /// Throws SchemaError with a precise description on any violation.
    void Finalize(int32_t root_index);

    // --- access

    bool IsFinalized() const { return m_root >= 0; }
    int32_t RootIndex() const { return m_root; }
    const TypeRecord& Root() const { return At(m_root); }
    const TypeRecord& At(int32_t index) const;
    size_t TypeCount() const { return m_types.size(); }

    /// Index of a type by full name, or -1.
    int32_t FindType(const std::string& type_name) const;

    /// Pretty-print in a .msg-like layout (for diagnostics and DescribeType).
    std::string ToString() const;

    // --- blob round trip (TYPE_SCHEMA payload)

    std::vector<uint8_t> EncodeBlob() const;
    static std::shared_ptr<const Schema> DecodeBlob(const uint8_t* data, size_t size);

  private:
    void Validate() const;

    std::vector<TypeRecord> m_types;
    int32_t m_root = -1;
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
