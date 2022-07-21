// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for segmented track assemblies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

namespace chrono {
namespace vehicle {

ChTrackAssemblySegmented::ChTrackAssemblySegmented(const std::string& name, VehicleSide side)
    : ChTrackAssembly(name, side), m_torque_funct(nullptr), m_bushing_data(nullptr) {}

void ChTrackAssemblySegmented::EnableTrackBendingStiffness(bool val) {
    if (!m_torque_funct)
        return;

    for (size_t i = 0; i < GetNumTrackShoes(); i++) {
        auto shoe = std::static_pointer_cast<ChTrackShoeSegmented>(GetTrackShoe(i));
        shoe->EnableTrackBendingStiffness(val);
    }
}

double ChTrackAssemblySegmented::TrackBendingFunctor::evaluate(double time,
                                                               double angle,
                                                               double vel,
                                                               const ChLinkRSDA& link) {
    // Clamp angle in [-pi, +pi]
    if (angle < -CH_C_PI)
        angle = CH_C_2PI - angle;
    if (angle > CH_C_PI)
        angle = angle - CH_C_2PI;
    // Linear spring-damper (assume 0 rest angle)
    return m_t - m_k * angle - m_c * vel;
}

}  // end namespace vehicle
}  // end namespace chrono
