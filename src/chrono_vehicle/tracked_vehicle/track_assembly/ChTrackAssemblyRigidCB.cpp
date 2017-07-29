// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a continuous band track assembly using rigid-link track shoes
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyRigidCB.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Assemble track shoes over wheels.
//
// Returns true if the track shoes were initialized in a counter clockwise
// direction and false otherwise.
//
// This procedure is performed in the chassis reference frame, taking into
// account the convention that the chassis reference frame has the x-axis
// pointing to the front of the vehicle and the z-axis pointing up.
// It is also assumed that the sprocket, idler, and road wheels lie in the
// same vertical plane (in the chassis reference frame). The assembly is done
// in the (x-z) plane.
//
// TODO: NEEDS fixes for clock-wise wrapping (idler in front of sprocket)
//
// -----------------------------------------------------------------------------
bool ChTrackAssemblyRigidCB::Assemble(std::shared_ptr<ChBodyAuxRef> chassis) {
    m_chassis = chassis;

    // Number of track shoes and road wheels.
    size_t num_shoes = m_shoes.size();
    size_t num_wheels = m_suspensions.size();
    size_t index = 0;

    // Positions of sprocket, idler, and (front and rear) wheels (in chassis reference frame).
    ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
    ChVector<> idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());
    ChVector<> front_wheel_pos_3d = chassis->TransformPointParentToLocal(m_suspensions[0]->GetWheelBody()->GetPos());
    ChVector<> rear_wheel_pos_3d = front_wheel_pos_3d;
    for (size_t i = 1; i < num_wheels; i++) {
        ChVector<> wheel_pos = chassis->TransformPointParentToLocal(m_suspensions[i]->GetWheelBody()->GetPos());
        if (wheel_pos.x() > front_wheel_pos_3d.x())
            front_wheel_pos_3d = wheel_pos;
        if (wheel_pos.x() < rear_wheel_pos_3d.x())
            rear_wheel_pos_3d = wheel_pos;
    }

    // Decide whether we wrap counter-clockwise (sprocket in front of idler) or
    // clockwise (sprocket behind idler).
    // Set the positions of the road wheel closest to the sprocket and of the one
    // closest to the idler.
    bool ccw = sprocket_pos_3d.x() > idler_pos_3d.x();
    double sign = ccw ? -1 : +1;
    const ChVector<>& wheel_sprocket_pos_3d = ccw ? front_wheel_pos_3d : rear_wheel_pos_3d;
    const ChVector<>& wheel_idler_pos_3d = ccw ? rear_wheel_pos_3d : front_wheel_pos_3d;

    // Restrict to (x-z) plane.
    ChVector2<> sprocket_pos(sprocket_pos_3d.x(), sprocket_pos_3d.z());
    ChVector2<> idler_pos(idler_pos_3d.x(), idler_pos_3d.z());
    ChVector2<> wheel_sprocket_pos(wheel_sprocket_pos_3d.x(), wheel_sprocket_pos_3d.z());
    ChVector2<> wheel_idler_pos(wheel_idler_pos_3d.x(), wheel_idler_pos_3d.z());

    //// TODO

    return ccw;
}

}  // end namespace vehicle
}  // end namespace chrono
