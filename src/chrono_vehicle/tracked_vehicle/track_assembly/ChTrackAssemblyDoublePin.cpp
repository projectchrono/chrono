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
// Base class for a track assembly using double-pin track shoes
// (template definitions).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"

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
bool ChTrackAssemblyDoublePin::Assemble(std::shared_ptr<ChBodyAuxRef> chassis) {
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

    // Subsystem parameters.
    // Note that the idler and wheel radii are inflated by a fraction of the shoe height.
    double shoe_length = m_shoes[0]->GetShoeLength();
    double connector_length = m_shoes[0]->GetConnectorLength();
    double shoe_pitch = m_shoes[0]->GetPitch();
    double shoe_height = m_shoes[0]->GetHeight();
    double sprocket_radius = m_sprocket->GetAssemblyRadius();
    double idler_radius = m_idler->GetWheelRadius() + 0.9 * shoe_height;
    double wheel_radius = m_suspensions[0]->GetWheelRadius() + 1.1 * shoe_height;

    m_chassis = chassis;
    m_sprocket_offset = sprocket_pos_3d.y();
    m_connector_offset = m_shoes[0]->GetShoeWidth() / 2;

    // Set target points around the track.
    ChVector2<> sprocket_bottom = sprocket_pos - ChVector2<>(0, sprocket_radius);
    ChVector2<> idler_top = idler_pos + ChVector2<>(0, idler_radius);
    ChVector2<> idler_bottom = idler_pos - ChVector2<>(0, idler_radius);
    ChVector2<> wheel_idler_bottom = wheel_idler_pos - ChVector2<>(0, wheel_radius);
    ChVector2<> wheel_sprocket_bottom = wheel_sprocket_pos - ChVector2<>(0, wheel_radius);

    // Keep track of the (x,z) locations of shoe body and connector bodies, as
    // well as of the angles (from the x axis) of the shoe and connector bodies.
    ChVector2<> ps;  // location of shoe center
    ChVector2<> pc;  // location of connector center
    ChVector2<> p1;  // location of rear pin (connection to previous)
    ChVector2<> p2;  // location of front pin (connection to next)
    double as;
    double ac;

    // 1. Create shoes around the sprocket, starting under the sprocket and
    //    moving away from the idler. Stop before creating a horizontal track
    //    shoe above the sprocket.

    // Create first track shoe, such that its connector body is horizontal and positioned
    // under the sprocket. Tilt the shoe body towards the roadwheel closest to the sprocket.
    pc = sprocket_bottom;
    p2 = pc + ChVector2<>(connector_length / 2, 0);
    ac = 0;
    ChVector2<> A = pc - ChVector2<>(connector_length / 2, 0);
    as = std::atan2(A.y() - wheel_sprocket_bottom.y(), A.x() - wheel_sprocket_bottom.x());
    ps = A - Vrot(ChVector2<>(shoe_length / 2, 0), as);
    p1 = ps - Vrot(ChVector2<>(shoe_length / 2, 0), as);
    CreateTrackShoe(index, ps, pc, as, ac);
    index++;

    // Cache location of rear pin (needed to close the track)
    ChVector2<> p0 = p1;

    // Wrap around sprocket.
    // ATTENTION:  USING SOME MAGIC NUMBERS HERE (angle adjustments)!!!!
    double delta = CH_C_2PI / m_sprocket->GetNumTeeth();
    for (int is = 1; is <= m_sprocket->GetNumTeeth() / 2; is++) {
        ChVector2<> A = sprocket_pos + sprocket_radius * ChVector2<>(std::sin(is*delta), -std::cos(is*delta));
        double angle = std::atan2(A.y() - p2.y(), A.x() - p2.x());
        as = angle - 0.07;
        ac = angle + 0.2;
        ps = p2 + Vrot(ChVector2<>(shoe_length / 2, 0), as);
        pc = ps + Vrot(ChVector2<>(shoe_length / 2, 0), as) + Vrot(ChVector2<>(connector_length / 2, 0), ac);
        CreateTrackShoe(index, ps, pc, as, ac);
        p2 = pc + Vrot(ChVector2<>(connector_length / 2, 0), ac);
        index++;
    }

    p1 = p2;

    // 2. Create shoes between sprocket and idler. These shoes are parallel to a
    //    line connecting the top points of the sprocket gear and idler wheel.
    //    We target a point that lies above the idler by slightly more than the
    //    track shoe's height and stop when we reach the idler location.

    // Calculate the constant pitch angle.
    double dy = (sprocket_pos.y() + sprocket_radius) - (idler_pos.y() + idler_radius);
    double dx = sprocket_pos.x() - idler_pos.x();
    double angle = ccw ? -CH_C_PI - std::atan2(dy, dx) : CH_C_PI + std::atan2(dy, -dx);

    // Create track shoes with constant orientation
    while (sign * (idler_pos.x() - p2.x() + shoe_pitch) > 0 && index < num_shoes) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
    }

    // 3. Create shoes around the idler wheel. Stop when we wrap under the idler.

    // Calculate the incremental pitch angle around the idler.
    double tmp = shoe_pitch / (2 * idler_radius);
    double delta_angle = sign * std::asin(tmp);

    while (std::abs(angle) < CH_C_2PI && index < num_shoes) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        angle += 2 * delta_angle;
        ++index;
    }

    // 4. Create shoes between idler and closest road wheel. The shoes are parallel
    //    to a line connecting bottom points on idler and wheel. Stop when passing
    //    the wheel position.

    dy = (idler_pos.y() - idler_radius) - (wheel_idler_pos.y() - wheel_radius);
    dx = idler_pos.x() - wheel_idler_pos.x();
    angle = ccw ? -CH_C_2PI + std::atan2(dy, -dx) : -CH_C_PI - std::atan2(dy, dx);

    // Create track shoes with constant orientation
    while (sign * (p2.x() - wheel_idler_pos.x()) > 0 && index < num_shoes) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
    }

    // 5. Create shoes below the road wheels. These shoes are horizontal. Stop when
    //    passing the position of the wheel closest to the sprocket.

    angle = ccw ? 0 : CH_C_2PI;

    while (sign * (p2.x() - wheel_sprocket_pos.x()) > 0 && index < num_shoes) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
    }

    // 6. If we have an odd number of track shoes left, create one more shoe, tilted towards
    //    the first track shoe.

    size_t num_left = num_shoes - index;

    if (num_left % 2 == 1) {
        angle = -std::atan2(p0.y() - p1.y(), p0.x() - p1.x());
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
        --num_left;
    }

    // 7. Check if the remaining shoes are enough to close the loop.

    double gap = (p0 - p1).Length();

    if (num_left * shoe_pitch < gap) {
        GetLog() << "\nInsufficient number of track shoes for this configuration.\n";
        GetLog() << "Missing distance: " << gap - num_left * shoe_pitch << "\n\n";
        for (size_t i = index; i < num_shoes; i++) {
            p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
            CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
            p1 = p2;
            ++index;
        }
        return ccw;
    }

    // 8. Complete the loop using the remaining shoes (always an even number)
    //    Form an isosceles triangle connecting the last initialized shoe with
    //    the very first one under the sprocket.

    double alpha = std::atan2(p0.y() - p2.y(), p0.x() - p2.x());
    double beta = std::acos(gap / (shoe_pitch * num_left));

    // Create half of the remaining shoes (with a pitch angle = alpha-beta).
    angle = sign * (alpha - beta);
    for (size_t i = 0; i < num_left / 2; i++) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
    }

    // Create the second half of the remaining shoes (pitch angle = alpha+beta).
    angle = sign * (alpha + beta);
    for (size_t i = 0; i < num_left / 2; i++) {
        p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
        CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
        p1 = p2;
        ++index;
    }

    ////GetLog() << "Track assembly done.  Number of track shoes: " << index << "\n";
    return ccw;
}

void ChTrackAssemblyDoublePin::CreateTrackShoe(size_t index, ChVector2<> ps, ChVector2<> pc, double as, double ac) {
    // Set index within the track assembly
    m_shoes[index]->SetIndex(index);

    // Body locations (relative to chassis frame)
    ChVector<> loc_shoe(ps.x(), m_sprocket_offset, ps.y());
    ChVector<> loc_connector_L(pc.x(), m_sprocket_offset + m_connector_offset, pc.y());
    ChVector<> loc_connector_R(pc.x(), m_sprocket_offset - m_connector_offset, pc.y());

    // Body orientation (relative to chassis frame)
    // Note that the angle sign must be flipped (x->y positive in 2D, but x->z negative in 3D)
    ChQuaternion<> rot_shoe = Q_from_AngY(-as);
    ChQuaternion<> rot_connector = Q_from_AngY(-ac);

    // Initialize the track shoe system
    m_shoes[index]->Initialize(m_chassis, loc_shoe, rot_shoe, loc_connector_L, loc_connector_R, rot_connector);
}

void ChTrackAssemblyDoublePin::CreateTrackShoe(size_t index, ChVector2<> p, double angle) {
    // Set index within the track assembly
    m_shoes[index]->SetIndex(index);

    // Location of track shoe center (relative to chassis frame)
    ChVector<> loc(p.x(), m_sprocket_offset, p.y());

    m_shoes[index]->Initialize(m_chassis, loc, Q_from_AngY(-angle));
}

void ChTrackAssemblyDoublePin::RemoveTrackShoes() {
    m_shoes.clear();
}

}  // end namespace vehicle
}  // end namespace chrono
