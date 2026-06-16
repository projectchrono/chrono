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

#include "chrono_ros/core/ChROSMessageCodec.h"
#include "chrono_ros/core/ChROSCdr.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

namespace chrono {
namespace ros {
namespace core {

namespace {

bool IsSignedInt(FieldKind kind) {
    switch (kind) {
        case FieldKind::Int8:
        case FieldKind::Int16:
        case FieldKind::Int32:
        case FieldKind::Int64:
            return true;
        default:
            return false;
    }
}

bool IsUnsignedInt(FieldKind kind) {
    switch (kind) {
        case FieldKind::UInt8:
        case FieldKind::UInt16:
        case FieldKind::UInt32:
        case FieldKind::UInt64:
        case FieldKind::Char:   // rosidl 'char' is uint8 on the wire
        case FieldKind::Octet:  // rosidl 'byte'/'octet'
            return true;
        default:
            return false;
    }
}

bool IsFloat(FieldKind kind) {
    return kind == FieldKind::Float32 || kind == FieldKind::Float64;
}

int64_t SignedMin(FieldKind kind) {
    switch (kind) {
        case FieldKind::Int8:
            return std::numeric_limits<int8_t>::min();
        case FieldKind::Int16:
            return std::numeric_limits<int16_t>::min();
        case FieldKind::Int32:
            return std::numeric_limits<int32_t>::min();
        default:
            return std::numeric_limits<int64_t>::min();
    }
}

int64_t SignedMax(FieldKind kind) {
    switch (kind) {
        case FieldKind::Int8:
            return std::numeric_limits<int8_t>::max();
        case FieldKind::Int16:
            return std::numeric_limits<int16_t>::max();
        case FieldKind::Int32:
            return std::numeric_limits<int32_t>::max();
        default:
            return std::numeric_limits<int64_t>::max();
    }
}

uint64_t UnsignedMax(FieldKind kind) {
    switch (kind) {
        case FieldKind::UInt8:
        case FieldKind::Char:
        case FieldKind::Octet:
            return std::numeric_limits<uint8_t>::max();
        case FieldKind::UInt16:
            return std::numeric_limits<uint16_t>::max();
        case FieldKind::UInt32:
            return std::numeric_limits<uint32_t>::max();
        default:
            return std::numeric_limits<uint64_t>::max();
    }
}

std::vector<std::string> SplitPath(const std::string& path) {
    std::vector<std::string> segments;
    size_t start = 0;
    while (true) {
        const size_t dot = path.find('.', start);
        const std::string segment = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        if (segment.empty()) {
            throw FieldError("malformed field path '" + path + "' (empty segment)");
        }
        segments.push_back(segment);
        if (dot == std::string::npos)
            break;
        start = dot + 1;
    }
    return segments;
}

std::string AvailableFields(const TypeRecord& type) {
    std::ostringstream out;
    for (size_t i = 0; i < type.fields.size(); i++) {
        out << (i ? ", " : "") << type.fields[i].name;
    }
    return out.str();
}

size_t FindFieldIndex(const Schema& schema, int32_t type_index, const std::string& name) {
    const TypeRecord& type = schema.At(type_index);
    for (size_t i = 0; i < type.fields.size(); i++) {
        if (type.fields[i].name == name)
            return i;
    }
    throw FieldError("no field '" + name + "' in " + type.name + " (available fields: " + AvailableFields(type) +
                     ")");
}

std::string DescribeArray(const FieldRecord& field) {
    switch (field.array_kind) {
        case ArrayKind::None:
            return "a scalar";
        case ArrayKind::FixedArray:
            return "a fixed array of " + std::to_string(field.array_size);
        case ArrayKind::BoundedSequence:
            return "a sequence bounded by " + std::to_string(field.array_size);
        case ArrayKind::UnboundedSequence:
            return "a sequence";
    }
    return "?";
}

void RejectWString(const FieldRecord& field, const std::string& where) {
    if (field.kind == FieldKind::WString) {
        throw FieldError("field '" + where +
                         "' is a wstring, which chrono_ros does not support (no real-world message uses it; "
                         "choose a message type without wstring fields)");
    }
}

}  // namespace

// ============================================================================
// MessageBuilder
// ============================================================================

MessageBuilder::MessageBuilder(std::shared_ptr<const Schema> schema) : m_schema(std::move(schema)) {
    if (!m_schema || !m_schema->IsFinalized()) {
        throw SchemaError("MessageBuilder requires a finalized schema");
    }
    m_type = m_schema->RootIndex();
}

MessageBuilder::MessageBuilder(std::shared_ptr<const Schema> schema, int32_t type_index)
    : m_schema(std::move(schema)), m_type(type_index) {}

const FieldRecord& MessageBuilder::Field(size_t index) const {
    return m_schema->At(m_type).fields[index];
}

std::string MessageBuilder::Where(const FieldRecord& field) const {
    return m_schema->At(m_type).name + "." + field.name;
}

std::pair<MessageBuilder*, size_t> MessageBuilder::Resolve(const std::string& path) {
    const std::vector<std::string> segments = SplitPath(path);
    MessageBuilder* current = this;
    for (size_t s = 0; s + 1 < segments.size(); s++) {
        const size_t index = FindFieldIndex(*current->m_schema, current->m_type, segments[s]);
        const FieldRecord& field = current->Field(index);
        if (field.kind != FieldKind::Message || field.array_kind != ArrayKind::None) {
            throw FieldError("cannot descend through '" + current->Where(field) + "': it is " +
                             (field.kind == FieldKind::Message
                                  ? DescribeArray(field) + " of messages (use AppendMessage to add elements)"
                                  : DescribeArray(field) + " of " + FieldKindName(field.kind)));
        }
        auto it = current->m_values.find(index);
        if (it == current->m_values.end()) {
            std::unique_ptr<MessageBuilder> child(new MessageBuilder(current->m_schema, field.nested_type));
            it = current->m_values.emplace(index, Value(std::move(child))).first;
        }
        current = std::get<std::unique_ptr<MessageBuilder>>(it->second).get();
    }
    const size_t terminal = FindFieldIndex(*current->m_schema, current->m_type, segments.back());
    return {current, terminal};
}

void MessageBuilder::SetBool(const std::string& path, bool value) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    if (field.kind != FieldKind::Bool || field.array_kind != ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a bool scalar");
    }
    owner->m_values.insert_or_assign(index, Value(value));
}

void MessageBuilder::SetInt(const std::string& path, int64_t value) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    RejectWString(field, owner->Where(field));
    if (field.array_kind != ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) +
                         "; use SetBlob for primitive arrays");
    }
    if (IsSignedInt(field.kind)) {
        if (value < SignedMin(field.kind) || value > SignedMax(field.kind)) {
            throw FieldError("value " + std::to_string(value) + " out of range for '" + owner->Where(field) + "' (" +
                             FieldKindName(field.kind) + ")");
        }
        owner->m_values.insert_or_assign(index, Value(value));
    } else if (IsUnsignedInt(field.kind)) {
        if (value < 0 || static_cast<uint64_t>(value) > UnsignedMax(field.kind)) {
            throw FieldError("value " + std::to_string(value) + " out of range for '" + owner->Where(field) + "' (" +
                             FieldKindName(field.kind) + ")");
        }
        owner->m_values.insert_or_assign(index, Value(static_cast<uint64_t>(value)));
    } else if (IsFloat(field.kind)) {
        owner->m_values.insert_or_assign(index, Value(static_cast<double>(value)));
    } else {
        throw FieldError("'" + owner->Where(field) + "' is a " + FieldKindName(field.kind) +
                         ", not a numeric field");
    }
}

void MessageBuilder::SetUInt(const std::string& path, uint64_t value) {
    if (value <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
        SetInt(path, static_cast<int64_t>(value));
        return;
    }
    // Values above INT64_MAX only fit uint64 fields.
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    if (field.kind != FieldKind::UInt64 || field.array_kind != ArrayKind::None) {
        throw FieldError("value " + std::to_string(value) + " out of range for '" + owner->Where(field) + "' (" +
                         FieldKindName(field.kind) + ")");
    }
    owner->m_values.insert_or_assign(index, Value(value));
}

void MessageBuilder::SetDouble(const std::string& path, double value) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    if (!IsFloat(field.kind) || field.array_kind != ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) +
                         "; SetDouble applies to float scalars only (use SetInt for integer fields)");
    }
    owner->m_values.insert_or_assign(index, Value(value));
}

void MessageBuilder::SetString(const std::string& path, const std::string& value) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    RejectWString(field, owner->Where(field));
    if (field.kind != FieldKind::String || field.array_kind != ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a string scalar");
    }
    owner->m_values.insert_or_assign(index, Value(value));
}

void MessageBuilder::SetBlobImpl(const std::string& path, const void* data, size_t count, bool copy) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    RejectWString(field, owner->Where(field));
    if (!IsPrimitive(field.kind) || field.array_kind == ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + "; SetBlob applies to primitive arrays/sequences");
    }
    if (field.array_kind == ArrayKind::FixedArray && count != field.array_size) {
        throw FieldError("'" + owner->Where(field) + "' is a fixed array of " + std::to_string(field.array_size) +
                         " elements; got " + std::to_string(count));
    }
    if (field.array_kind == ArrayKind::BoundedSequence && count > field.array_size) {
        throw FieldError("'" + owner->Where(field) + "' is bounded by " + std::to_string(field.array_size) +
                         " elements; got " + std::to_string(count));
    }
    if (data == nullptr && count != 0) {
        throw FieldError("SetBlob for '" + owner->Where(field) + "': null data with non-zero count");
    }
    BlobValue blob;
    blob.count = count;
    if (copy && count > 0) {
        const size_t bytes = count * PrimitiveSize(field.kind);
        blob.owned.assign(static_cast<const uint8_t*>(data), static_cast<const uint8_t*>(data) + bytes);
        blob.data = blob.owned.data();
    } else {
        blob.data = data;
    }
    owner->m_values.insert_or_assign(index, Value(std::move(blob)));
}

void MessageBuilder::SetBlob(const std::string& path, const void* data, size_t count) {
    SetBlobImpl(path, data, count, /*copy=*/false);
}

void MessageBuilder::SetBlobCopy(const std::string& path, const void* data, size_t count) {
    SetBlobImpl(path, data, count, /*copy=*/true);
}

void MessageBuilder::SetBlobBytes(const std::string& path, const void* data, size_t nbytes) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    RejectWString(field, owner->Where(field));
    if (!IsPrimitive(field.kind) || field.array_kind == ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + "; SetBlob applies to primitive arrays/sequences");
    }
    const size_t element_size = PrimitiveSize(field.kind);
    if (element_size == 0) {
        throw FieldError("'" + owner->Where(field) + "': unknown element size for " + FieldKindName(field.kind));
    }
    if (nbytes % element_size != 0) {
        throw FieldError("'" + owner->Where(field) + "': a buffer of " + std::to_string(nbytes) +
                         " bytes is not a whole number of " + std::to_string(element_size) + "-byte " +
                         FieldKindName(field.kind) + " elements");
    }
    SetBlobImpl(path, data, nbytes / element_size, /*copy=*/true);
}

void MessageBuilder::SetStringArray(const std::string& path, const std::vector<std::string>& value) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    RejectWString(field, owner->Where(field));
    if (field.kind != FieldKind::String || field.array_kind == ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a string array/sequence");
    }
    if (field.array_kind == ArrayKind::FixedArray && value.size() != field.array_size) {
        throw FieldError("'" + owner->Where(field) + "' is a fixed array of " + std::to_string(field.array_size) +
                         " strings; got " + std::to_string(value.size()));
    }
    if (field.array_kind == ArrayKind::BoundedSequence && value.size() > field.array_size) {
        throw FieldError("'" + owner->Where(field) + "' is bounded by " + std::to_string(field.array_size) +
                         " strings; got " + std::to_string(value.size()));
    }
    owner->m_values.insert_or_assign(index, Value(value));
}

MessageBuilder& MessageBuilder::GetNested(const std::string& path) {
    // Resolve the full path as a descent; reuse Resolve by descending through
    // every segment, so the terminal must itself be a singular message.
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    if (field.kind != FieldKind::Message || field.array_kind != ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a singular nested message");
    }
    auto it = owner->m_values.find(index);
    if (it == owner->m_values.end()) {
        std::unique_ptr<MessageBuilder> child(new MessageBuilder(owner->m_schema, field.nested_type));
        it = owner->m_values.emplace(index, Value(std::move(child))).first;
    }
    return *std::get<std::unique_ptr<MessageBuilder>>(it->second);
}

MessageBuilder& MessageBuilder::AppendMessage(const std::string& path) {
    auto [owner, index] = Resolve(path);
    const FieldRecord& field = owner->Field(index);
    if (field.kind != FieldKind::Message || field.array_kind == ArrayKind::None) {
        throw FieldError("'" + owner->Where(field) + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a message array/sequence");
    }
    auto it = owner->m_values.find(index);
    if (it == owner->m_values.end()) {
        it = owner->m_values.emplace(index, Value(MessageSeq{})).first;
    }
    MessageSeq& seq = std::get<MessageSeq>(it->second);
    if ((field.array_kind == ArrayKind::FixedArray || field.array_kind == ArrayKind::BoundedSequence) &&
        seq.size() >= field.array_size) {
        throw FieldError("'" + owner->Where(field) + "' cannot hold more than " + std::to_string(field.array_size) +
                         " elements");
    }
    seq.emplace_back(new MessageBuilder(owner->m_schema, field.nested_type));
    return *seq.back();
}

void MessageBuilder::SetTime(const std::string& path, double time_seconds) {
    if (!std::isfinite(time_seconds) || time_seconds < 0) {
        throw FieldError("SetTime('" + path + "'): time must be finite and non-negative, got " +
                         std::to_string(time_seconds));
    }
    auto seconds = static_cast<int64_t>(time_seconds);
    auto nanoseconds = static_cast<int64_t>(std::llround((time_seconds - static_cast<double>(seconds)) * 1e9));
    if (nanoseconds >= 1000000000) {  // carry from rounding at the second boundary
        seconds += 1;
        nanoseconds -= 1000000000;
    }
    MessageBuilder& stamp = GetNested(path);
    stamp.SetInt("sec", seconds);
    stamp.SetInt("nanosec", nanoseconds);
}

void MessageBuilder::Clear() {
    m_values.clear();
}

// --- serialization ----------------------------------------------------------

void MessageBuilder::SerializeTo(std::vector<uint8_t>& out) const {
    CdrWriter writer(out, /*with_encapsulation=*/true);
    SerializeBody(writer);
}

void MessageBuilder::SerializeBody(CdrWriter& writer) const {
    const TypeRecord& type = m_schema->At(m_type);
    for (size_t i = 0; i < type.fields.size(); i++) {
        auto it = m_values.find(i);
        SerializeField(writer, type.fields[i], it == m_values.end() ? nullptr : &it->second);
    }
}

void MessageBuilder::SerializeField(CdrWriter& writer, const FieldRecord& field, const Value* value) const {
    if (value == nullptr) {
        SerializeDefaultField(writer, *m_schema, field);
        return;
    }
    RejectWString(field, Where(field));

    switch (field.array_kind) {
        case ArrayKind::None: {
            if (field.kind == FieldKind::Message) {
                std::get<std::unique_ptr<MessageBuilder>>(*value)->SerializeBody(writer);
            } else if (field.kind == FieldKind::String) {
                writer.WriteString(std::get<std::string>(*value));
            } else {
                // Scalar: stored canonically by the setters.
                switch (field.kind) {
                    case FieldKind::Bool:
                        writer.WriteBool(std::get<bool>(*value));
                        break;
                    case FieldKind::Int8:
                        writer.WriteInt8(static_cast<int8_t>(std::get<int64_t>(*value)));
                        break;
                    case FieldKind::Int16:
                        writer.WriteInt16(static_cast<int16_t>(std::get<int64_t>(*value)));
                        break;
                    case FieldKind::Int32:
                        writer.WriteInt32(static_cast<int32_t>(std::get<int64_t>(*value)));
                        break;
                    case FieldKind::Int64:
                        writer.WriteInt64(std::get<int64_t>(*value));
                        break;
                    case FieldKind::UInt8:
                    case FieldKind::Char:
                    case FieldKind::Octet:
                        writer.WriteUInt8(static_cast<uint8_t>(std::get<uint64_t>(*value)));
                        break;
                    case FieldKind::UInt16:
                        writer.WriteUInt16(static_cast<uint16_t>(std::get<uint64_t>(*value)));
                        break;
                    case FieldKind::UInt32:
                        writer.WriteUInt32(static_cast<uint32_t>(std::get<uint64_t>(*value)));
                        break;
                    case FieldKind::UInt64:
                        writer.WriteUInt64(std::get<uint64_t>(*value));
                        break;
                    case FieldKind::Float32:
                        writer.WriteFloat32(static_cast<float>(std::get<double>(*value)));
                        break;
                    case FieldKind::Float64:
                        writer.WriteFloat64(std::get<double>(*value));
                        break;
                    default:
                        throw std::logic_error("unhandled scalar kind in SerializeField");
                }
            }
            break;
        }
        case ArrayKind::FixedArray:
        case ArrayKind::BoundedSequence:
        case ArrayKind::UnboundedSequence: {
            const bool is_fixed = field.array_kind == ArrayKind::FixedArray;
            if (field.kind == FieldKind::Message) {
                const MessageSeq& seq = std::get<MessageSeq>(*value);
                if (!is_fixed) {
                    writer.WriteUInt32(static_cast<uint32_t>(seq.size()));
                }
                for (const auto& element : seq) {
                    element->SerializeBody(writer);
                }
                if (is_fixed) {  // pad a partially filled fixed array with default elements
                    for (size_t i = seq.size(); i < field.array_size; i++) {
                        SerializeDefaultBody(writer, *m_schema, field.nested_type);
                    }
                }
            } else if (field.kind == FieldKind::String) {
                const auto& strings = std::get<std::vector<std::string>>(*value);
                if (!is_fixed) {
                    writer.WriteUInt32(static_cast<uint32_t>(strings.size()));
                }
                for (const auto& s : strings) {
                    writer.WriteString(s);
                }
                // Fixed-size string arrays were length-checked at Set time.
            } else {
                const BlobValue& blob = std::get<BlobValue>(*value);
                const size_t element_size = PrimitiveSize(field.kind);
                if (!is_fixed) {
                    writer.WriteUInt32(static_cast<uint32_t>(blob.count));
                }
                if (blob.count > 0) {
                    writer.Align(element_size);
                    writer.WriteBytes(blob.data, blob.count * element_size);
                }
            }
            break;
        }
    }
}

void MessageBuilder::SerializeDefaultBody(CdrWriter& writer, const Schema& schema, int32_t type_index) {
    for (const FieldRecord& field : schema.At(type_index).fields) {
        SerializeDefaultField(writer, schema, field);
    }
}

void MessageBuilder::SerializeDefaultField(CdrWriter& writer, const Schema& schema, const FieldRecord& field) {
    RejectWString(field, field.name);

    switch (field.array_kind) {
        case ArrayKind::None: {
            if (field.kind == FieldKind::Message) {
                SerializeDefaultBody(writer, schema, field.nested_type);
            } else if (field.kind == FieldKind::String) {
                writer.WriteString(std::string());
            } else {
                const size_t element_size = PrimitiveSize(field.kind);
                writer.Align(element_size);
                static const uint8_t zeros[8] = {0};
                writer.WriteBytes(zeros, element_size);
            }
            break;
        }
        case ArrayKind::FixedArray: {
            if (field.kind == FieldKind::Message) {
                for (uint32_t i = 0; i < field.array_size; i++) {
                    SerializeDefaultBody(writer, schema, field.nested_type);
                }
            } else if (field.kind == FieldKind::String) {
                for (uint32_t i = 0; i < field.array_size; i++) {
                    writer.WriteString(std::string());
                }
            } else {
                const size_t element_size = PrimitiveSize(field.kind);
                writer.Align(element_size);
                const std::vector<uint8_t> zeros(field.array_size * element_size, 0);
                writer.WriteBytes(zeros.data(), zeros.size());
            }
            break;
        }
        case ArrayKind::BoundedSequence:
        case ArrayKind::UnboundedSequence: {
            writer.WriteUInt32(0);  // empty sequence
            break;
        }
    }
}

// ============================================================================
// MessageReader
// ============================================================================

MessageReader::MessageReader(std::shared_ptr<const Schema> schema, const uint8_t* data, size_t size)
    : m_schema(std::move(schema)), m_data(data), m_size(size) {
    if (!m_schema || !m_schema->IsFinalized()) {
        throw SchemaError("MessageReader requires a finalized schema");
    }
}

MessageReader::MessageReader(std::shared_ptr<const Schema> schema,
                             const uint8_t* data,
                             size_t size,
                             std::vector<PathSegment> prefix)
    : m_schema(std::move(schema)), m_data(data), m_size(size), m_prefix(std::move(prefix)) {}

void MessageReader::SkipBody(CdrReader& reader, const Schema& schema, int32_t type_index) {
    for (const FieldRecord& field : schema.At(type_index).fields) {
        SkipField(reader, schema, field);
    }
}

void MessageReader::SkipField(CdrReader& reader, const Schema& schema, const FieldRecord& field) {
    if (field.kind == FieldKind::WString) {
        throw FieldError("field '" + field.name +
                         "' is a wstring, which chrono_ros does not support; cannot parse this message");
    }

    size_t count = 0;
    switch (field.array_kind) {
        case ArrayKind::None:
            count = 1;
            break;
        case ArrayKind::FixedArray:
            count = field.array_size;
            break;
        case ArrayKind::BoundedSequence:
        case ArrayKind::UnboundedSequence:
            count = reader.ReadUInt32();
            break;
    }
    if (count == 0)
        return;

    if (field.kind == FieldKind::Message) {
        for (size_t i = 0; i < count; i++) {
            SkipBody(reader, schema, field.nested_type);
        }
    } else if (field.kind == FieldKind::String) {
        for (size_t i = 0; i < count; i++) {
            const uint32_t length = reader.ReadUInt32();
            reader.Skip(length);
        }
    } else {
        const size_t element_size = PrimitiveSize(field.kind);
        reader.Align(element_size);
        reader.Skip(count * element_size);
    }
}

const FieldRecord& MessageReader::Seek(CdrReader& reader, const std::string& path) const {
    // Combine the reader's prefix (from GetMessage scoping) with the new path.
    std::vector<PathSegment> segments = m_prefix;
    for (const std::string& name : SplitPath(path)) {
        segments.push_back(PathSegment{name, 0, false});
    }

    int32_t type_index = m_schema->RootIndex();
    for (size_t s = 0;; s++) {
        const TypeRecord& type = m_schema->At(type_index);
        const size_t field_index = FindFieldIndex(*m_schema, type_index, segments[s].name);

        // Skip the fields that precede this one.
        for (size_t i = 0; i < field_index; i++) {
            SkipField(reader, *m_schema, type.fields[i]);
        }
        const FieldRecord& field = type.fields[field_index];

        if (s + 1 == segments.size()) {
            return field;  // reader is positioned at the start of this field
        }

        // Descend.
        if (field.kind != FieldKind::Message) {
            throw FieldError("cannot descend through '" + type.name + "." + field.name + "': it is " +
                             DescribeArray(field) + " of " + FieldKindName(field.kind));
        }
        if (field.array_kind == ArrayKind::None) {
            if (segments[s].has_index && segments[s].index != 0) {
                throw FieldError("'" + type.name + "." + field.name + "' is a singular message; index " +
                                 std::to_string(segments[s].index) + " is invalid");
            }
        } else {
            size_t count = field.array_size;
            if (field.array_kind != ArrayKind::FixedArray) {
                count = reader.ReadUInt32();
            }
            const size_t want = segments[s].has_index ? segments[s].index : 0;
            if (!segments[s].has_index) {
                throw FieldError("'" + type.name + "." + field.name +
                                 "' is a message array/sequence; access elements with GetMessage(path, index)");
            }
            if (want >= count) {
                throw FieldError("index " + std::to_string(want) + " out of range for '" + type.name + "." +
                                 field.name + "' (" + std::to_string(count) + " element(s))");
            }
            for (size_t i = 0; i < want; i++) {
                SkipBody(reader, *m_schema, field.nested_type);
            }
        }
        type_index = field.nested_type;
    }
}

namespace {

void RequireScalar(const FieldRecord& field, const char* expected) {
    if (field.array_kind != ArrayKind::None) {
        throw FieldError("field '" + field.name + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a " + expected + " scalar");
    }
}

}  // namespace

bool MessageReader::GetBool(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    RequireScalar(field,"bool");
    if (field.kind != FieldKind::Bool) {
        throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) + ", not a bool");
    }
    return reader.ReadBool();
}

int64_t MessageReader::GetInt(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    RequireScalar(field,"integer");
    switch (field.kind) {
        case FieldKind::Int8:
            return reader.ReadInt8();
        case FieldKind::Int16:
            return reader.ReadInt16();
        case FieldKind::Int32:
            return reader.ReadInt32();
        case FieldKind::Int64:
            return reader.ReadInt64();
        case FieldKind::UInt8:
        case FieldKind::Char:
        case FieldKind::Octet:
            return reader.ReadUInt8();
        case FieldKind::UInt16:
            return reader.ReadUInt16();
        case FieldKind::UInt32:
            return reader.ReadUInt32();
        case FieldKind::UInt64: {
            const uint64_t v = reader.ReadUInt64();
            if (v > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
                throw FieldError("field '" + field.name + "' value " + std::to_string(v) +
                                 " does not fit in int64; use GetUInt");
            }
            return static_cast<int64_t>(v);
        }
        default:
            throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) +
                             ", not an integer field");
    }
}

uint64_t MessageReader::GetUInt(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    RequireScalar(field,"integer");
    switch (field.kind) {
        case FieldKind::UInt8:
        case FieldKind::Char:
        case FieldKind::Octet:
            return reader.ReadUInt8();
        case FieldKind::UInt16:
            return reader.ReadUInt16();
        case FieldKind::UInt32:
            return reader.ReadUInt32();
        case FieldKind::UInt64:
            return reader.ReadUInt64();
        case FieldKind::Int8:
        case FieldKind::Int16:
        case FieldKind::Int32:
        case FieldKind::Int64: {
            int64_t v = 0;
            switch (field.kind) {
                case FieldKind::Int8:
                    v = reader.ReadInt8();
                    break;
                case FieldKind::Int16:
                    v = reader.ReadInt16();
                    break;
                case FieldKind::Int32:
                    v = reader.ReadInt32();
                    break;
                default:
                    v = reader.ReadInt64();
                    break;
            }
            if (v < 0) {
                throw FieldError("field '" + field.name + "' value " + std::to_string(v) +
                                 " is negative; use GetInt");
            }
            return static_cast<uint64_t>(v);
        }
        default:
            throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) +
                             ", not an integer field");
    }
}

double MessageReader::GetDouble(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    RequireScalar(field,"numeric");
    switch (field.kind) {
        case FieldKind::Float32:
            return reader.ReadFloat32();
        case FieldKind::Float64:
            return reader.ReadFloat64();
        case FieldKind::Int8:
            return reader.ReadInt8();
        case FieldKind::Int16:
            return reader.ReadInt16();
        case FieldKind::Int32:
            return reader.ReadInt32();
        case FieldKind::Int64:
            return static_cast<double>(reader.ReadInt64());
        case FieldKind::UInt8:
        case FieldKind::Char:
        case FieldKind::Octet:
            return reader.ReadUInt8();
        case FieldKind::UInt16:
            return reader.ReadUInt16();
        case FieldKind::UInt32:
            return reader.ReadUInt32();
        case FieldKind::UInt64:
            return static_cast<double>(reader.ReadUInt64());
        default:
            throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) +
                             ", not a numeric field");
    }
}

std::string MessageReader::GetString(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    RequireScalar(field,"string");
    if (field.kind != FieldKind::String) {
        throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) + ", not a string");
    }
    return reader.ReadString();
}

std::vector<std::string> MessageReader::GetStringArray(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    if (field.kind != FieldKind::String || field.array_kind == ArrayKind::None) {
        throw FieldError("field '" + field.name + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + ", not a string array/sequence");
    }
    size_t count = field.array_size;
    if (field.array_kind != ArrayKind::FixedArray) {
        count = reader.ReadUInt32();
    }
    std::vector<std::string> result;
    result.reserve(count);
    for (size_t i = 0; i < count; i++) {
        result.push_back(reader.ReadString());
    }
    return result;
}

BlobView MessageReader::GetBlob(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    if (!IsPrimitive(field.kind) || field.array_kind == ArrayKind::None) {
        throw FieldError("field '" + field.name + "' is " + DescribeArray(field) + " of " +
                         FieldKindName(field.kind) + "; GetBlob applies to primitive arrays/sequences");
    }
    size_t count = field.array_size;
    if (field.array_kind != ArrayKind::FixedArray) {
        count = reader.ReadUInt32();
    }
    BlobView view;
    view.count = count;
    view.element_size = PrimitiveSize(field.kind);
    if (count > 0) {
        reader.Align(view.element_size);
        view.data = reader.Current();
        reader.Skip(count * view.element_size);  // bounds check
    }
    return view;
}

size_t MessageReader::GetCount(const std::string& path) const {
    CdrReader reader(m_data, m_size, true);
    const FieldRecord& field = Seek(reader, path);
    switch (field.array_kind) {
        case ArrayKind::None:
            throw FieldError("field '" + field.name + "' is a scalar, not an array/sequence");
        case ArrayKind::FixedArray:
            return field.array_size;
        case ArrayKind::BoundedSequence:
        case ArrayKind::UnboundedSequence:
            return reader.ReadUInt32();
    }
    return 0;
}

MessageReader MessageReader::GetMessage(const std::string& path, size_t index) const {
    // Validate the path and the index eagerly so errors surface here, not at
    // the first getter call on the returned reader.
    {
        CdrReader reader(m_data, m_size, true);
        const FieldRecord& field = Seek(reader, path);
        if (field.kind != FieldKind::Message) {
            throw FieldError("field '" + field.name + "' is a " + FieldKindName(field.kind) +
                             ", not a nested message");
        }
        if (field.array_kind == ArrayKind::None) {
            if (index != 0) {
                throw FieldError("'" + field.name + "' is a singular message; index " + std::to_string(index) +
                                 " is invalid");
            }
        } else {
            size_t count = field.array_size;
            if (field.array_kind != ArrayKind::FixedArray) {
                count = reader.ReadUInt32();
            }
            if (index >= count) {
                throw FieldError("index " + std::to_string(index) + " out of range for '" + field.name + "' (" +
                                 std::to_string(count) + " element(s))");
            }
        }
    }

    std::vector<PathSegment> prefix = m_prefix;
    std::vector<std::string> names = SplitPath(path);
    for (size_t i = 0; i + 1 < names.size(); i++) {
        prefix.push_back(PathSegment{names[i], 0, false});
    }
    prefix.push_back(PathSegment{names.back(), index, true});
    return MessageReader(m_schema, m_data, m_size, std::move(prefix));
}

double MessageReader::GetTimeSec(const std::string& path) const {
    const MessageReader stamp = GetMessage(path);
    return static_cast<double>(stamp.GetInt("sec")) + static_cast<double>(stamp.GetUInt("nanosec")) * 1e-9;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
