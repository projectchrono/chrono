// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
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

void ChROSHandler::Update(double time, double step) {
    // NOTE: If update_rate == 0, tick is called each time
    double frame_time = m_update_rate == 0 ? 0 : 1 / m_update_rate;

    m_time_elapsed_since_last_tick += step;
    if (m_time_elapsed_since_last_tick < frame_time)
        return;

    m_time_elapsed_since_last_tick -= frame_time;

    Tick(time);
    m_tick_count++;
}

}  // namespace ros
}  // namespace chrono