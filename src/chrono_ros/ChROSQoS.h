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
// Public quality-of-service type for Chrono::ROS publishers/subscriptions.
// The underlying spec lives in the dependency-free core so the bridge node
// shares the exact definition.
//
// =============================================================================

#ifndef CH_ROS_QOS_H
#define CH_ROS_QOS_H

#include "chrono_ros/core/ChROSQoSSpec.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_core
/// @{

/// Quality of service for a topic. Use the presets unless you know you need
/// otherwise: ChROSQoS::Default(), ChROSQoS::SensorData() (high-rate streams),
/// ChROSQoS::Latched() (set-once topics like /robot_description),
/// ChROSQoS::Clock() (/clock).
using ChROSQoS = core::QoSSpec;
using ChROSReliability = core::Reliability;
using ChROSDurability = core::Durability;

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
