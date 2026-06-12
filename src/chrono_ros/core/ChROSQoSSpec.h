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
// Quality-of-service specification shared by the simulation process and
// chrono_ros_node. Deliberately minimal (reliability, durability, depth):
// these three cover every built-in handler and the common user cases. The
// node maps this onto rclcpp::QoS. (See CLAUDE.md section 6.3.)
//
// =============================================================================

#ifndef CH_ROS_CORE_QOS_SPEC_H
#define CH_ROS_CORE_QOS_SPEC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace core {

enum class Reliability : uint8_t {
    Reliable = 0,
    BestEffort = 1,
};

enum class Durability : uint8_t {
    Volatile = 0,
    TransientLocal = 1,  ///< "latched": late joiners receive the last sample
};

struct QoSSpec {
    Reliability reliability = Reliability::Reliable;
    Durability durability = Durability::Volatile;
    uint32_t depth = 10;  ///< KEEP_LAST history depth

    /// Reliable, volatile, depth 10 - the rclcpp default profile.
    static QoSSpec Default() { return QoSSpec{}; }

    /// Best-effort, volatile, shallow queue - high-rate sensor streams
    /// (matches rclcpp::SensorDataQoS in the dimensions we expose).
    static QoSSpec SensorData() { return QoSSpec{Reliability::BestEffort, Durability::Volatile, 5}; }

    /// Reliable, transient-local, depth 1 - set-once topics that late
    /// subscribers must still receive (e.g. /robot_description).
    static QoSSpec Latched() { return QoSSpec{Reliability::Reliable, Durability::TransientLocal, 1}; }

    /// Best-effort, volatile, depth 1 - the /clock profile (rclcpp::ClockQoS).
    static QoSSpec Clock() { return QoSSpec{Reliability::BestEffort, Durability::Volatile, 1}; }
};

}  // namespace core
}  // namespace ros
}  // namespace chrono

#endif
