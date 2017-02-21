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
// Base class for a tire.
// A tire subsystem is a force element. It is passed position and velocity
// information of the wheel body and it produces ground reaction forces and
// moments to be applied to the wheel body.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

ChTire::ChTire(const std::string& name)
    : ChPart(name), m_slip_angle(0), m_longitudinal_slip(0), m_camber_angle(0) {}

// -----------------------------------------------------------------------------
// Base class implementation of the initialization function.
// -----------------------------------------------------------------------------
void ChTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    m_wheel = wheel;
    m_side = side;

    ////WheelState state;
    ////state.pos = wheel->GetPos();
    ////state.rot = wheel->GetRot();
    ////state.lin_vel = wheel->GetPos_dt();
    ////state.ang_vel = wheel->GetWvel_par();
    ////ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
    ////state.omega = ang_vel_loc.y();
}

// -----------------------------------------------------------------------------
// Calculate kinematics quantities (slip angle, longitudinal slip, camber angle,
// and toe-in angle using the current state of the associated wheel body.
// -----------------------------------------------------------------------------
void ChTire::CalculateKinematics(double time, const WheelState& state, const ChTerrain& terrain) {
    // Wheel normal (expressed in global frame)
    ChVector<> wheel_normal = state.rot.GetYaxis();

    // Terrain normal at wheel location (expressed in global frame)
    ChVector<> Z_dir = terrain.GetNormal(state.pos.x(), state.pos.y());

    // Longitudinal (heading) and lateral directions, in the terrain plane
    ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
    X_dir.Normalize();
    ChVector<> Y_dir = Vcross(Z_dir, X_dir);

    // Tire reference coordinate system
    ChMatrix33<> rot;
    rot.Set_A_axis(X_dir, Y_dir, Z_dir);
    ChCoordsys<> tire_csys(state.pos, rot.Get_A_quaternion());

    // Express wheel linear velocity in tire frame
    ChVector<> V = tire_csys.TransformDirectionParentToLocal(state.lin_vel);
    // Express wheel normal in tire frame
    ChVector<> n = tire_csys.TransformDirectionParentToLocal(wheel_normal);

    // Slip angle
    double abs_Vx = std::abs(V.x());
    double zero_Vx = 1e-4;
    m_slip_angle = (abs_Vx > zero_Vx) ? std::atan(V.y() / abs_Vx) : 0;

    // Longitudinal slip
    m_longitudinal_slip = (abs_Vx > zero_Vx) ? -(V.x() - state.omega * GetRadius()) / abs_Vx : 0;

    // Camber angle
    m_camber_angle = std::atan2(n.z(), n.y());
}

// -----------------------------------------------------------------------------
// Utility function for characterizing the geometric contact between a disc with
// specified center location, normal direction, and radius and the terrain,
// assumed to be specified as a height field (over the x-y domain).
// This function returns false if no contact occurs. Otherwise, it sets the
// contact points on the disc (ptD) and on the terrain (ptT), the normal contact
// direction, and the resulting penetration depth (a positive value).
// -----------------------------------------------------------------------------
bool ChTire::disc_terrain_contact(const ChTerrain& terrain,
                                  const ChVector<>& disc_center,
                                  const ChVector<>& disc_normal,
                                  double disc_radius,
                                  ChCoordsys<>& contact,
                                  double& depth) {
    // Find terrain height below disc center. There is no contact if the disc
    // center is below the terrain or farther away by more than its radius.
    double hc = terrain.GetHeight(disc_center.x(), disc_center.y());
    if (disc_center.z() <= hc || disc_center.z() >= hc + disc_radius)
        return false;

    // Find the lowest point on the disc. There is no contact if the disc is
    // (almost) horizontal.
    ChVector<> dir1 = Vcross(disc_normal, ChVector<>(0, 0, 1));
    double sinTilt2 = dir1.Length2();

    if (sinTilt2 < 1e-3)
        return false;

    // Contact point (lowest point on disc).
    ChVector<> ptD = disc_center + disc_radius * Vcross(disc_normal, dir1 / sqrt(sinTilt2));

    // Find terrain height at lowest point. No contact if lowest point is above
    // the terrain.
    double hp = terrain.GetHeight(ptD.x(), ptD.y());

    if (ptD.z() > hp)
        return false;

    // Approximate the terrain with a plane. Define the projection of the lowest
    // point onto this plane as the contact point on the terrain.
    ChVector<> normal = terrain.GetNormal(ptD.x(), ptD.y());
    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);
    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, normal);

    contact.pos = ptD;
    contact.rot = rot.Get_A_quaternion();

    depth = Vdot(ChVector<>(0, 0, hp - ptD.z()), normal);
    assert(depth > 0);

    return true;
}

}  // end namespace vehicle
}  // end namespace chrono
