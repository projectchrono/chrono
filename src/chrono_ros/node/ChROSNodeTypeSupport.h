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
// chrono_ros_node side of the type-knowledge machinery (NEVER compiled into
// the simulation process):
//
//  - BuildSchema: dlopen the introspection typesupport of a message type by
//    name and walk it into a core::Schema (the TYPE_SCHEMA payload).
//  - ValidateSample: round-trip a CDR sample through the real rmw
//    serialization support and byte-compare, so a sim-side encoding bug or a
//    middleware format change aborts initialization instead of corrupting
//    topics (CLAUDE.md section 6.2).
//
// =============================================================================

#ifndef CH_ROS_NODE_TYPE_SUPPORT_H
#define CH_ROS_NODE_TYPE_SUPPORT_H

#include "chrono_ros/core/ChROSControl.h"
#include "chrono_ros/core/ChROSSchema.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {
namespace node {

/// Walk the rosidl introspection typesupport of 'type_name' (e.g.
/// "sensor_msgs/msg/Image") into a Schema. Throws std::runtime_error with an
/// actionable message if the type's package is not installed/sourced or the
/// type contains unsupported constructs (wstring, long double).
core::Schema BuildSchema(const std::string& type_name);

/// Deserialize 'sample_cdr' into a generically-constructed message instance
/// using the type's real serialization support, re-serialize it, and compare.
/// Fills a ValidateResultPayload (ok, or first-diff diagnostics).
core::ValidateResultPayload ValidateSample(const std::string& type_name, const std::vector<uint8_t>& sample_cdr);

}  // namespace node
}  // namespace ros
}  // namespace chrono

#endif
