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
// Schema-driven construction and inspection of ROS 2 messages, without ROS.
//
// MessageBuilder assembles a message as a sparse tree of named field values
// over a Schema and serializes it to wire-format CDR (unset fields serialize
// as zeros / empty). MessageReader is the inverse: field access by name over
// a received CDR buffer.
//
// Field paths are dot-separated ("header.frame_id", "linear.x"). These are
// internal engine classes; the user-facing API (ChROSMessage in the Chrono_ros
// library, and its Python projection) wraps them.
//
// =============================================================================

#ifndef CH_ROS_CORE_MESSAGE_CODEC_H
#define CH_ROS_CORE_MESSAGE_CODEC_H

#include "chrono_ros/core/ChROSSchema.h"

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace chrono {
namespace ros {
namespace core {

class CdrWriter;
class CdrReader;

/// Error thrown for an unknown field path, a kind mismatch, or an
/// out-of-range value. The message always names the offending type/field and
/// lists the valid fields, so the failure is actionable without ROS docs.
class FieldError : public std::runtime_error {
  public:
    explicit FieldError(const std::string& what) : std::runtime_error("field error: " + what) {}
};

/// Zero-copy view of a primitive array/sequence inside a CDR buffer.
/// Valid only as long as the buffer passed to MessageReader is alive.
struct BlobView {
    const uint8_t* data = nullptr;
    size_t count = 0;         ///< number of elements
    size_t element_size = 0;  ///< bytes per element
    size_t SizeBytes() const { return count * element_size; }
};

// -----------------------------------------------------------------------------

/// Mutable, schema-backed message under construction. Move-only.
class MessageBuilder {
  public:
    /// Create a builder for the schema's root type.
    explicit MessageBuilder(std::shared_ptr<const Schema> schema);

    MessageBuilder(MessageBuilder&&) = default;
    MessageBuilder& operator=(MessageBuilder&&) = default;
    MessageBuilder(const MessageBuilder&) = delete;
    MessageBuilder& operator=(const MessageBuilder&) = delete;

    // --- scalar setters (throw FieldError on unknown path / kind mismatch / range)

    void SetBool(const std::string& path, bool value);
    /// Accepted by all integer fields (with range check) and float fields.
    void SetInt(const std::string& path, int64_t value);
    void SetUInt(const std::string& path, uint64_t value);
    /// Accepted by float32/float64 fields only (integer fields refuse doubles
    /// to surface accidental lossy writes).
    void SetDouble(const std::string& path, double value);
    void SetString(const std::string& path, const std::string& value);

    // --- arrays and sequences

    /// Set a primitive array/sequence from raw little-endian element data.
    /// 'count' is the element count; the byte size is derived from the field's
    /// element type. The data is NOT copied: it must stay alive until
    /// serialization. For bool elements, bytes must be 0 or 1.
    void SetBlob(const std::string& path, const void* data, size_t count);

    /// As SetBlob, but copies the data into the builder (safe lifetime).
    void SetBlobCopy(const std::string& path, const void* data, size_t count);

    /// Set a string array/sequence.
    void SetStringArray(const std::string& path, const std::vector<std::string>& value);

    // --- nested messages

    /// Builder for a singular nested message field (created on first access).
    MessageBuilder& GetNested(const std::string& path);

    /// Append an element to a message array/sequence field and return its
    /// builder. Bounded sequences and fixed arrays enforce their capacity.
    MessageBuilder& AppendMessage(const std::string& path);

    // --- convenience

    /// Set a builtin_interfaces/Time-shaped nested message ("<path>.sec",
    /// "<path>.nanosec") from seconds. Negative times are rejected.
    void SetTime(const std::string& path, double time_seconds);

    /// Remove all values (schema is retained).
    void Clear();

    // --- output

    /// Append the wire-format serialization (encapsulation header + CDR body)
    /// of this message to 'out'. May be called repeatedly.
    void SerializeTo(std::vector<uint8_t>& out) const;

    const Schema& GetSchema() const { return *m_schema; }
    int32_t TypeIndex() const { return m_type; }

  private:
    MessageBuilder(std::shared_ptr<const Schema> schema, int32_t type_index);

    struct BlobValue {
        std::vector<uint8_t> owned;  ///< storage when copied
        const void* data = nullptr;
        size_t count = 0;
    };
    using MessageSeq = std::vector<std::unique_ptr<MessageBuilder>>;
    using Value = std::variant<bool,
                               int64_t,
                               uint64_t,
                               double,
                               std::string,
                               BlobValue,
                               std::vector<std::string>,
                               std::unique_ptr<MessageBuilder>,  // singular nested message
                               MessageSeq>;                      // message array/sequence

    /// Walk a dot path, creating intermediate nested builders as needed.
    /// Returns the builder owning the terminal segment and the field's index.
    std::pair<MessageBuilder*, size_t> Resolve(const std::string& path);

    const FieldRecord& Field(size_t index) const;
    std::string Where(const FieldRecord& field) const;  ///< "type.field" for error text

    void SetNumeric(const std::string& path, Value value, bool from_signed, bool from_unsigned, bool from_double);
    void SetBlobImpl(const std::string& path, const void* data, size_t count, bool copy);

    void SerializeBody(CdrWriter& writer) const;
    void SerializeField(CdrWriter& writer, const FieldRecord& field, const Value* value) const;
    static void SerializeDefaultBody(CdrWriter& writer, const Schema& schema, int32_t type_index);
    static void SerializeDefaultField(CdrWriter& writer, const Schema& schema, const FieldRecord& field);

    std::shared_ptr<const Schema> m_schema;
    int32_t m_type;
    std::map<size_t, Value> m_values;  ///< field index -> value
};

// -----------------------------------------------------------------------------

/// Read-only, schema-backed view over a serialized message. Does not own the
/// buffer; it must outlive the reader and any BlobView obtained from it.
///
/// Access cost: each getter walks the CDR buffer from the start (CDR is a
/// sequential format). This is the right trade for the bridge's traffic
/// pattern - small control messages read a handful of fields, and bulk
/// payloads are fetched with a single GetBlob.
class MessageReader {
  public:
    /// @param data wire-format message (encapsulation header + CDR body)
    MessageReader(std::shared_ptr<const Schema> schema, const uint8_t* data, size_t size);

    bool GetBool(const std::string& path) const;
    int64_t GetInt(const std::string& path) const;    ///< any integer field (range-checked narrowing)
    uint64_t GetUInt(const std::string& path) const;  ///< any non-negative integer field
    double GetDouble(const std::string& path) const;  ///< float fields, or lossless from integers
    std::string GetString(const std::string& path) const;
    std::vector<std::string> GetStringArray(const std::string& path) const;

    /// Zero-copy view of a primitive array/sequence field.
    BlobView GetBlob(const std::string& path) const;

    /// Element count of an array/sequence field (primitive, string or message).
    size_t GetCount(const std::string& path) const;

    /// Reader scoped to a nested message: a singular nested field, or element
    /// 'index' of a message array/sequence.
    MessageReader GetMessage(const std::string& path, size_t index = 0) const;

    /// Read a builtin_interfaces/Time-shaped nested message as seconds.
    double GetTimeSec(const std::string& path) const;

    const Schema& GetSchema() const { return *m_schema; }

  private:
    struct PathSegment {
        std::string name;
        size_t index = 0;      ///< element index when the segment is a message sequence
        bool has_index = false;
    };

    MessageReader(std::shared_ptr<const Schema> schema,
                  const uint8_t* data,
                  size_t size,
                  std::vector<PathSegment> prefix);

    /// Walk from the buffer start through m_prefix then 'path', leaving the
    /// reader positioned at the terminal field. Returns that field's record
    /// and the enclosing type index.
    const FieldRecord& Seek(CdrReader& reader, const std::string& path) const;

    static void SkipBody(CdrReader& reader, const Schema& schema, int32_t type_index);
    static void SkipField(CdrReader& reader, const Schema& schema, const FieldRecord& field);

    std::shared_ptr<const Schema> m_schema;
    const uint8_t* m_data;
    size_t m_size;
    std::vector<PathSegment> m_prefix;
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
