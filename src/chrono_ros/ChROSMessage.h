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
// ChROSMessage: a ROS 2 message under construction, addressed by field name.
//
// Obtain one from ChROSPublisher::NewMessage(); set the fields you care about
// (unset fields publish as zeros/empty); pass it to Publish(). Field paths
// are dot-separated ("header.frame_id"). All setters throw
// chrono::ros::core::FieldError with the valid field list on a wrong name,
// kind mismatch, or out-of-range value.
//
// ChROSMessageView is the read-side equivalent, delivered to subscription
// callbacks.
//
// =============================================================================

#ifndef CH_ROS_MESSAGE_H
#define CH_ROS_MESSAGE_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/core/ChROSMessageCodec.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

/// @addtogroup ros_core
/// @{

/// A mutable, schema-backed ROS 2 message (see file header). Copyable like a
/// lightweight handle: copies share the same underlying message.
class CH_ROS_API ChROSMessage {
  public:
    // --- scalars
    /// Set a bool-typed field.
    void SetBool(const std::string& path, bool value);
    /// Set a signed-integer field (any int width; range-checked against the schema type).
    void SetInt(const std::string& path, int64_t value);
    /// Set an unsigned-integer field (any width; range-checked against the schema type).
    void SetUInt(const std::string& path, uint64_t value);
    /// Set a floating-point field (float32 or float64).
    void SetDouble(const std::string& path, double value);
    /// Set a string field.
    void SetString(const std::string& path, const std::string& value);

    // --- arrays / sequences
    /// Primitive array/sequence from raw little-endian elements ('count' is
    /// the element count). Zero-copy: the memory must stay valid until
    /// Publish() returns. Use SetBlobCopy when that is inconvenient.
    void SetBlob(const std::string& path, const void* data, size_t count);
    /// As SetBlob, but copies the elements immediately (no lifetime requirement on 'data').
    void SetBlobCopy(const std::string& path, const void* data, size_t count);
    /// As SetBlobCopy, but sized in bytes (element count = nbytes / element_size,
    /// which must divide evenly). For byte-oriented sources such as Python
    /// buffer-protocol objects, where only the byte length is known.
    void SetBlobBytes(const std::string& path, const void* data, size_t nbytes);
    /// Set a string array/sequence field.
    void SetStringArray(const std::string& path, const std::vector<std::string>& value);

    // --- nested messages
    /// Handle to a singular nested message field.
    ChROSMessage Nested(const std::string& path);
    /// Append an element to a message array/sequence and return its handle.
    ChROSMessage AppendMessage(const std::string& path);

    // --- convenience
    /// Fill a builtin_interfaces/Time-shaped field ("<path>.sec"/".nanosec")
    /// from seconds.
    void SetTime(const std::string& path, double time_seconds);

    /// Remove all values set so far.
    void Clear();

    /// Pretty-print the message type's full field layout.
    std::string DescribeType() const;

  private:
    friend class ChROSPublisher;
    friend class ChROSBridge;

    explicit ChROSMessage(std::shared_ptr<core::MessageBuilder> root);
    ChROSMessage(std::shared_ptr<core::MessageBuilder> root, core::MessageBuilder* node);

    core::MessageBuilder& Node() const;

    std::shared_ptr<core::MessageBuilder> m_root;  ///< keeps the message alive
    core::MessageBuilder* m_node;                  ///< this handle's scope within it
};

// -----------------------------------------------------------------------------

/// A received, immutable ROS 2 message, addressed by field name. Delivered to
/// ChROSSubscriptionCallback::OnMessage(); valid only during the callback
/// unless retained (handles share ownership of the payload buffer, so keeping
/// a copy of the view object is safe and cheap).
class CH_ROS_API ChROSMessageView {
  public:
    /// Read a bool-typed field.
    bool GetBool(const std::string& path) const;
    /// Read a signed-integer field.
    int64_t GetInt(const std::string& path) const;
    /// Read an unsigned-integer field.
    uint64_t GetUInt(const std::string& path) const;
    /// Read a floating-point field.
    double GetDouble(const std::string& path) const;
    /// Read a string field.
    std::string GetString(const std::string& path) const;
    /// Read a string array/sequence field.
    std::vector<std::string> GetStringArray(const std::string& path) const;

    /// Zero-copy view of a primitive array/sequence (camera pixels, lidar
    /// points, ...). The view is valid as long as this ChROSMessageView (or a
    /// copy of it) is alive.
    core::BlobView GetBlob(const std::string& path) const;

    /// Copying alternative (e.g. for retaining data past the callback).
    std::vector<uint8_t> CopyBlob(const std::string& path) const;

    /// Element count of an array/sequence field.
    size_t GetCount(const std::string& path) const;

    /// View scoped to a nested message (singular field, or element 'index'
    /// of a message array/sequence).
    ChROSMessageView GetMessage(const std::string& path, size_t index = 0) const;

    /// Read a builtin_interfaces/Time-shaped field as seconds.
    double GetTimeSec(const std::string& path) const;

    /// Pretty-print the message type's full field layout.
    std::string DescribeType() const;

  private:
    friend class ChROSBridge;

    ChROSMessageView(std::shared_ptr<const core::Schema> schema, std::shared_ptr<const std::vector<uint8_t>> data);
    ChROSMessageView(std::shared_ptr<const std::vector<uint8_t>> data, core::MessageReader reader);

    std::shared_ptr<const std::vector<uint8_t>> m_data;  ///< shared payload ownership
    core::MessageReader m_reader;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
