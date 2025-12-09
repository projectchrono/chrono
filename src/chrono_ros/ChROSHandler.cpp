// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Base class for all ros handlers
//
// =============================================================================

#include "chrono_ros/ChROSHandler.h"

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

ChROSHandler::ChROSHandler(double update_rate) : m_update_rate(update_rate), m_time_elapsed_since_last_tick(0) {}

}  // namespace ros
}  // namespace chrono