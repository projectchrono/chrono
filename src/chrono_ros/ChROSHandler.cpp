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
// Authors: Aaron Young, Patrick Chen
// =============================================================================

#include "chrono_ros/ChROSHandler.h"

#include <stdexcept>

namespace chrono {
namespace ros {

ChROSHandler::ChROSHandler(double update_rate) : m_update_rate(update_rate) {
    if (update_rate < 0) {
        throw std::invalid_argument("ChROSHandler: update_rate must be >= 0 (0 ticks every simulation step)");
    }
}

void ChROSHandler::Advance(double time, double step) {
    if (m_update_rate <= 0) {
        Tick(time);
        m_tick_count++;
        return;
    }
    m_time_elapsed_since_last_tick += step;
    const double period = 1.0 / m_update_rate;
    if (m_time_elapsed_since_last_tick < period) {
        return;
    }
    // Keep the remainder so the average rate stays exact even when the step
    // size does not divide the period.
    m_time_elapsed_since_last_tick -= period;
    Tick(time);
    m_tick_count++;
}

}  // namespace ros
}  // namespace chrono
