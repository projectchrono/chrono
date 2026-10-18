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

#include "chrono_ros/core/ChROSSchema.h"
#include "chrono_ros/core/ChROSCdr.h"

#include <sstream>

namespace chrono {
namespace ros {
namespace core {

bool IsPrimitive(FieldKind kind) {
    switch (kind) {
        case FieldKind::String:
        case FieldKind::WString:
        case FieldKind::Message:
            return false;
        default:
            return true;
    }
}

size_t PrimitiveSize(FieldKind kind) {
    switch (kind) {
        case FieldKind::Bool:
        case FieldKind::Octet:
        case FieldKind::Char:
        case FieldKind::Int8:
        case FieldKind::UInt8:
            return 1;
        case FieldKind::Int16:
        case FieldKind::UInt16:
            return 2;
        case FieldKind::Int32:
        case FieldKind::UInt32:
        case FieldKind::Float32:
            return 4;
        case FieldKind::Int64:
        case FieldKind::UInt64:
        case FieldKind::Float64:
            return 8;
        default:
            throw SchemaError(std::string("kind '") + FieldKindName(kind) + "' has no fixed primitive size");
    }
}

const char* FieldKindName(FieldKind kind) {
    switch (kind) {
        case FieldKind::Bool:
            return "bool";
        case FieldKind::Octet:
            return "octet";
        case FieldKind::Char:
            return "char";
        case FieldKind::Int8:
            return "int8";
        case FieldKind::UInt8:
            return "uint8";
        case FieldKind::Int16:
            return "int16";
        case FieldKind::UInt16:
            return "uint16";
        case FieldKind::Int32:
            return "int32";
        case FieldKind::UInt32:
            return "uint32";
        case FieldKind::Int64:
            return "int64";
        case FieldKind::UInt64:
            return "uint64";
        case FieldKind::Float32:
            return "float32";
        case FieldKind::Float64:
            return "float64";
        case FieldKind::String:
            return "string";
        case FieldKind::WString:
            return "wstring";
        case FieldKind::Message:
            return "message";
    }
    return "unknown";
}

namespace {
bool IsValidKind(uint8_t raw) {
    return raw >= static_cast<uint8_t>(FieldKind::Bool) && raw <= static_cast<uint8_t>(FieldKind::Message);
}
bool IsValidArrayKind(uint8_t raw) {
    return raw <= static_cast<uint8_t>(ArrayKind::UnboundedSequence);
}
}  // namespace

// ----------------------------------------------------------------------------

int32_t Schema::AddType(const std::string& type_name) {
    if (type_name.empty()) {
        throw SchemaError("type name must not be empty");
    }
    if (FindType(type_name) >= 0) {
        throw SchemaError("duplicate type '" + type_name + "'");
    }
    m_types.push_back(TypeRecord{type_name, {}});
    return static_cast<int32_t>(m_types.size()) - 1;
}

void Schema::AddField(int32_t type_index, FieldRecord field) {
    if (type_index < 0 || type_index >= static_cast<int32_t>(m_types.size())) {
        throw SchemaError("AddField: type index " + std::to_string(type_index) + " out of range");
    }
    m_types[static_cast<size_t>(type_index)].fields.push_back(std::move(field));
}

const TypeRecord& Schema::At(int32_t index) const {
    if (index < 0 || index >= static_cast<int32_t>(m_types.size())) {
        throw SchemaError("type index " + std::to_string(index) + " out of range (have " +
                          std::to_string(m_types.size()) + " types)");
    }
    return m_types[static_cast<size_t>(index)];
}

int32_t Schema::FindType(const std::string& type_name) const {
    for (size_t i = 0; i < m_types.size(); i++) {
        if (m_types[i].name == type_name)
            return static_cast<int32_t>(i);
    }
    return -1;
}

void Schema::Finalize(int32_t root_index) {
    if (root_index < 0 || root_index >= static_cast<int32_t>(m_types.size())) {
        throw SchemaError("root index " + std::to_string(root_index) + " out of range");
    }
    m_root = root_index;
    Validate();
}

void Schema::Validate() const {
    if (m_root < 0) {
        throw SchemaError("schema has no root type");
    }
    for (const auto& type : m_types) {
        for (const auto& field : type.fields) {
            const std::string where = type.name + "." + field.name;
            if (field.name.empty()) {
                throw SchemaError("unnamed field in type '" + type.name + "'");
            }
            if (field.kind == FieldKind::Message) {
                if (field.nested_type < 0 || field.nested_type >= static_cast<int32_t>(m_types.size())) {
                    throw SchemaError("field '" + where + "' references unknown nested type index " +
                                      std::to_string(field.nested_type));
                }
            } else if (field.nested_type != -1) {
                throw SchemaError("non-message field '" + where + "' must not reference a nested type");
            }
            if (field.array_kind == ArrayKind::FixedArray && field.array_size == 0) {
                throw SchemaError("fixed-size array field '" + where + "' must have a non-zero size");
            }
            if ((field.array_kind == ArrayKind::None || field.array_kind == ArrayKind::UnboundedSequence) &&
                field.array_size != 0) {
                throw SchemaError("field '" + where + "' must not have an array size");
            }
        }
    }
}

std::string Schema::ToString() const {
    std::ostringstream out;
    for (size_t i = 0; i < m_types.size(); i++) {
        out << (static_cast<int32_t>(i) == m_root ? "" : "  # nested type\n");
        out << m_types[i].name << "\n";
        for (const auto& field : m_types[i].fields) {
            std::string type_name = field.kind == FieldKind::Message ? At(field.nested_type).name
                                                                     : std::string(FieldKindName(field.kind));
            switch (field.array_kind) {
                case ArrayKind::None:
                    break;
                case ArrayKind::FixedArray:
                    type_name += "[" + std::to_string(field.array_size) + "]";
                    break;
                case ArrayKind::BoundedSequence:
                    type_name += "[<=" + std::to_string(field.array_size) + "]";
                    break;
                case ArrayKind::UnboundedSequence:
                    type_name += "[]";
                    break;
            }
            out << "  " << type_name << " " << field.name << "\n";
        }
        if (i + 1 < m_types.size())
            out << "\n";
    }
    return out.str();
}

// ----------------------------------------------------------------------------
// Blob format (CDR-encoded, no encapsulation):
//   uint16 blob_version
//   int32  root_index
//   uint32 type_count
//   per type:  string name, uint32 field_count
//   per field: string name, uint8 kind, uint8 array_kind, uint32 array_size,
//              int32 nested_type
// ----------------------------------------------------------------------------

std::vector<uint8_t> Schema::EncodeBlob() const {
    if (!IsFinalized()) {
        throw SchemaError("cannot encode a schema that has not been finalized");
    }
    std::vector<uint8_t> blob;
    CdrWriter writer(blob, /*with_encapsulation=*/false);
    writer.WriteUInt16(kBlobVersion);
    writer.WriteInt32(m_root);
    writer.WriteUInt32(static_cast<uint32_t>(m_types.size()));
    for (const auto& type : m_types) {
        writer.WriteString(type.name);
        writer.WriteUInt32(static_cast<uint32_t>(type.fields.size()));
        for (const auto& field : type.fields) {
            writer.WriteString(field.name);
            writer.WriteUInt8(static_cast<uint8_t>(field.kind));
            writer.WriteUInt8(static_cast<uint8_t>(field.array_kind));
            writer.WriteUInt32(field.array_size);
            writer.WriteInt32(field.nested_type);
        }
    }
    return blob;
}

std::shared_ptr<const Schema> Schema::DecodeBlob(const uint8_t* data, size_t size) {
    CdrReader reader(data, size, /*with_encapsulation=*/false);

    const uint16_t version = reader.ReadUInt16();
    if (version != kBlobVersion) {
        throw SchemaError("schema blob version mismatch: got " + std::to_string(version) + ", expected " +
                          std::to_string(kBlobVersion) +
                          " (simulation library and chrono_ros_node are from different builds)");
    }

    auto schema = std::make_shared<Schema>();
    const int32_t root = reader.ReadInt32();
    const uint32_t type_count = reader.ReadUInt32();
    for (uint32_t t = 0; t < type_count; t++) {
        const int32_t type_index = schema->AddType(reader.ReadString());
        const uint32_t field_count = reader.ReadUInt32();
        for (uint32_t f = 0; f < field_count; f++) {
            FieldRecord field;
            field.name = reader.ReadString();
            const uint8_t raw_kind = reader.ReadUInt8();
            const uint8_t raw_array = reader.ReadUInt8();
            if (!IsValidKind(raw_kind)) {
                throw SchemaError("blob contains invalid field kind " + std::to_string(raw_kind));
            }
            if (!IsValidArrayKind(raw_array)) {
                throw SchemaError("blob contains invalid array kind " + std::to_string(raw_array));
            }
            field.kind = static_cast<FieldKind>(raw_kind);
            field.array_kind = static_cast<ArrayKind>(raw_array);
            field.array_size = reader.ReadUInt32();
            field.nested_type = reader.ReadInt32();
            schema->AddField(type_index, std::move(field));
        }
    }
    if (reader.Remaining() != 0) {
        throw SchemaError("schema blob has " + std::to_string(reader.Remaining()) + " trailing byte(s)");
    }
    schema->Finalize(root);  // also validates
    return schema;
}

}  // namespace core
}  // namespace ros
}  // namespace chrono
