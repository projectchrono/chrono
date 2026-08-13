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

#include "chrono_ros/node/ChROSNodeTypeSupport.h"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/version.h>

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace chrono {
namespace ros {
namespace node {

namespace intro = rosidl_typesupport_introspection_cpp;

namespace {

/// rclcpp 17 (Iron) renamed get_typesupport_handle to
/// get_message_typesupport_handle and deprecated the old spelling.
const rosidl_message_type_support_t* GetTypeSupportHandle(const std::string& type_name,
                                                          const std::string& identifier,
                                                          rcpputils::SharedLibrary& library) {
#if RCLCPP_VERSION_MAJOR >= 17
    return rclcpp::get_message_typesupport_handle(type_name, identifier, library);
#else
    return rclcpp::get_typesupport_handle(type_name, identifier, library);
#endif
}

/// "sensor_msgs::msg" + "Image" -> "sensor_msgs/msg/Image"
std::string FullTypeName(const intro::MessageMembers* members) {
    std::string ns = members->message_namespace_;
    size_t pos = 0;
    while ((pos = ns.find("::", pos)) != std::string::npos) {
        ns.replace(pos, 2, "/");
        pos += 1;
    }
    return ns + "/" + members->message_name_;
}

core::FieldKind MapFieldKind(uint8_t type_id, const std::string& type_name, const char* field_name) {
    switch (type_id) {
        case intro::ROS_TYPE_BOOLEAN:
            return core::FieldKind::Bool;
        case intro::ROS_TYPE_OCTET:
            return core::FieldKind::Octet;
        case intro::ROS_TYPE_CHAR:
            return core::FieldKind::Char;
        case intro::ROS_TYPE_INT8:
            return core::FieldKind::Int8;
        case intro::ROS_TYPE_UINT8:
            return core::FieldKind::UInt8;
        case intro::ROS_TYPE_INT16:
            return core::FieldKind::Int16;
        case intro::ROS_TYPE_UINT16:
            return core::FieldKind::UInt16;
        case intro::ROS_TYPE_INT32:
            return core::FieldKind::Int32;
        case intro::ROS_TYPE_UINT32:
            return core::FieldKind::UInt32;
        case intro::ROS_TYPE_INT64:
            return core::FieldKind::Int64;
        case intro::ROS_TYPE_UINT64:
            return core::FieldKind::UInt64;
        case intro::ROS_TYPE_FLOAT:
            return core::FieldKind::Float32;
        case intro::ROS_TYPE_DOUBLE:
            return core::FieldKind::Float64;
        case intro::ROS_TYPE_STRING:
            return core::FieldKind::String;
        case intro::ROS_TYPE_WSTRING:
        case intro::ROS_TYPE_WCHAR:
            return core::FieldKind::WString;  // recognized; rejected at use with a clear error
        case intro::ROS_TYPE_LONG_DOUBLE:
            throw std::runtime_error("type '" + type_name + "' field '" + field_name +
                                     "' is a long double, which chrono_ros does not support");
        case intro::ROS_TYPE_MESSAGE:
            return core::FieldKind::Message;
        default:
            throw std::runtime_error("type '" + type_name + "' field '" + field_name +
                                     "' has unknown introspection type id " + std::to_string(type_id));
    }
}

/// Recursively add 'members' (and everything nested in it) to the schema.
/// Returns the type's index. Deduplicates by full type name.
int32_t AddTypeRecursive(core::Schema& schema, const intro::MessageMembers* members) {
    const std::string type_name = FullTypeName(members);
    const int32_t existing = schema.FindType(type_name);
    if (existing >= 0) {
        return existing;
    }

    const int32_t type_index = schema.AddType(type_name);
    for (uint32_t i = 0; i < members->member_count_; i++) {
        const intro::MessageMember& member = members->members_[i];

        core::FieldRecord field;
        field.name = member.name_;
        field.kind = MapFieldKind(member.type_id_, type_name, member.name_);

        if (!member.is_array_) {
            field.array_kind = core::ArrayKind::None;
            field.array_size = 0;
        } else if (member.array_size_ > 0 && !member.is_upper_bound_) {
            field.array_kind = core::ArrayKind::FixedArray;
            field.array_size = static_cast<uint32_t>(member.array_size_);
        } else if (member.array_size_ > 0 && member.is_upper_bound_) {
            field.array_kind = core::ArrayKind::BoundedSequence;
            field.array_size = static_cast<uint32_t>(member.array_size_);
        } else {
            field.array_kind = core::ArrayKind::UnboundedSequence;
            field.array_size = 0;
        }

        if (field.kind == core::FieldKind::Message) {
            const auto* nested_ts = member.members_;
            if (nested_ts == nullptr || nested_ts->data == nullptr) {
                throw std::runtime_error("type '" + type_name + "' field '" + std::string(member.name_) +
                                         "' has no nested introspection data");
            }
            const auto* nested = static_cast<const intro::MessageMembers*>(nested_ts->data);
            field.nested_type = AddTypeRecursive(schema, nested);
        }

        schema.AddField(type_index, std::move(field));
    }
    return type_index;
}

std::string NotFoundHint(const std::string& type_name, const std::string& what) {
    return "could not load typesupport for '" + type_name + "': " + what +
           ". Is the message package installed and its workspace sourced in the environment that launched the "
           "simulation? (custom packages: build with colcon and 'source install/setup.bash' first)";
}

}  // namespace

core::Schema BuildSchema(const std::string& type_name) {
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_message_type_support_t* typesupport = nullptr;
    try {
        library = rclcpp::get_typesupport_library(type_name, intro::typesupport_identifier);
        typesupport = GetTypeSupportHandle(type_name, intro::typesupport_identifier, *library);
    } catch (const std::exception& e) {
        throw std::runtime_error(NotFoundHint(type_name, e.what()));
    }
    if (typesupport == nullptr || typesupport->data == nullptr) {
        throw std::runtime_error(NotFoundHint(type_name, "typesupport handle is null"));
    }

    const auto* members = static_cast<const intro::MessageMembers*>(typesupport->data);

    core::Schema schema;
    const int32_t root = AddTypeRecursive(schema, members);
    schema.Finalize(root);

    // Keep the library alive for the lifetime of the process: the schema only
    // holds copies of strings, but publishers created later will dlopen the
    // same library again via rclcpp anyway. Letting 'library' go out of scope
    // here is safe because all data was copied into the schema.
    return schema;
}

core::ValidateResultPayload ValidateSample(const std::string& type_name, const std::vector<uint8_t>& sample_cdr) {
    core::ValidateResultPayload result;
    result.type_name = type_name;

    try {
        // Introspection support: gives us size/init/fini for a generic instance.
        auto intro_library = rclcpp::get_typesupport_library(type_name, intro::typesupport_identifier);
        const auto* intro_ts = GetTypeSupportHandle(type_name, intro::typesupport_identifier, *intro_library);
        const auto* members = static_cast<const intro::MessageMembers*>(intro_ts->data);

        // Serialization support: the same path rmw uses on the wire.
        auto ts_library = rclcpp::get_typesupport_library(type_name, "rosidl_typesupport_cpp");
        const auto* ts_handle = GetTypeSupportHandle(type_name, "rosidl_typesupport_cpp", *ts_library);
        rclcpp::SerializationBase serialization(ts_handle);

        // Generic message instance.
        std::vector<uint8_t> storage(members->size_of_);
        members->init_function(storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

        try {
            rclcpp::SerializedMessage incoming(sample_cdr.size());
            auto& rcl_incoming = incoming.get_rcl_serialized_message();
            std::memcpy(rcl_incoming.buffer, sample_cdr.data(), sample_cdr.size());
            rcl_incoming.buffer_length = sample_cdr.size();

            serialization.deserialize_message(&incoming, storage.data());

            // Pre-size and zero the output buffer before re-serializing. CDR
            // alignment-padding gaps are advanced over without being written by
            // some rmw serializers (FastCDR leaves whatever was in the buffer),
            // so without zeroing, interior padding bytes are uninitialized
            // garbage and the byte comparison below gets false positives even
            // though the two encodings are semantically identical (readers skip
            // padding). Our codec zero-fills padding; zeroing here matches it.
            // Capacity comfortably exceeds the message so no realloc occurs.
            rclcpp::SerializedMessage outgoing(sample_cdr.size() + 64);
            auto& rcl_pre = outgoing.get_rcl_serialized_message();
            std::memset(rcl_pre.buffer, 0, rcl_pre.buffer_capacity);
            serialization.serialize_message(storage.data(), &outgoing);
            const auto& rcl_outgoing = outgoing.get_rcl_serialized_message();
            const uint8_t* out_data = rcl_outgoing.buffer;
            const size_t out_size = rcl_outgoing.buffer_length;

            // Byte-compare. Some serializers pad the tail to 4-byte alignment;
            // trailing zeros beyond the common prefix are not a mismatch
            // because no decoder reads past the encoded body.
            const size_t common = std::min(sample_cdr.size(), out_size);
            size_t diff = common;
            for (size_t i = 0; i < common; i++) {
                if (sample_cdr[i] != out_data[i]) {
                    diff = i;
                    break;
                }
            }
            bool tail_ok = true;
            if (diff == common) {
                for (size_t i = common; i < sample_cdr.size(); i++) {
                    tail_ok = tail_ok && sample_cdr[i] == 0;
                }
                for (size_t i = common; i < out_size; i++) {
                    tail_ok = tail_ok && out_data[i] == 0;
                }
            }

            if (diff == common && tail_ok) {
                result.ok = true;
            } else {
                result.ok = false;
                result.first_diff_offset = diff;
                const size_t window_start = diff >= 16 ? diff - 16 : 0;
                const size_t window_end = diff + 16;
                for (size_t i = window_start; i < std::min(window_end, out_size); i++) {
                    result.expected_window.push_back(out_data[i]);
                }
                for (size_t i = window_start; i < std::min(window_end, sample_cdr.size()); i++) {
                    result.actual_window.push_back(sample_cdr[i]);
                }
                result.error = "serialized bytes differ at offset " + std::to_string(diff) +
                               " (lengths: chrono " + std::to_string(sample_cdr.size()) + ", rmw " +
                               std::to_string(out_size) + ")";
            }
        } catch (...) {
            members->fini_function(storage.data());
            throw;
        }
        members->fini_function(storage.data());
    } catch (const std::exception& e) {
        result.ok = false;
        result.error = std::string("validation round-trip failed: ") + e.what();
    }

    return result;
}

}  // namespace node
}  // namespace ros
}  // namespace chrono
