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
// Utility functions for creating paths as Bezier curves.
//
// =============================================================================

#ifndef CH_VEHICLE_PATH_H
#define CH_VEHICLE_PATH_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/core/ChBezierCurve.h"

namespace chrono {
namespace vehicle {

/// Straight line path between the two specified end points.
/// Intermediate points can be optionally included.
CH_VEHICLE_API std::shared_ptr<ChBezierCurve> StraightLinePath(
    const ChVector<>& start,           ///< start point
    const ChVector<> end,              ///< end point
    unsigned int num_intermediate = 0  ///< number of intermediate points
    );

/// Circular path with an initial straight-line run in the X direction.
/// The generated curve loops around the circle for a specified number of times.
/// 
/// <pre>
/// Example: left circle path
/// 
///                     __<__         Y
///                   /       \       ^
///                  /         \      |
///                 |     o     |     +---> X
///                  \         /
///  ________>________\ __>__ /
///         run
/// </pre>
CH_VEHICLE_API std::shared_ptr<ChBezierCurve> CirclePath(
    const ChVector<>& start,  ///< start point
    double radius,            ///< circle radius
    double run,               ///< length of initial straight line
    bool left_turn = true,    ///< left turn (true), right turn (false)
    int num_turns = 1         ///< number of turns around circle
    );

/// Double lane change path with initial and final runs in the X direction.
///
/// <pre>
/// Example: left double lane change path
///
///                                          Y
///                                          ^
///                   ______>______          |
///                  /             \         +---> X
///                /                 \       | 
/// _____>______ /                     \ ______>_______
///     run       ramp   length    ramp      run
/// </pre>
CH_VEHICLE_API std::shared_ptr<ChBezierCurve> DoubleLaneChangePath(
    const ChVector<>& start,  ///< start point
    double ramp,              ///< length of ramp sections
    double width,             ///< lane separation
    double length,            ///< distance in lane
    double run,               ///< length of initial and final straight lines
    bool left_turn = true     ///< left turn (true), right turn (false)
    );

}  // end namespace vehicle
}  // end namespace chrono

#endif
