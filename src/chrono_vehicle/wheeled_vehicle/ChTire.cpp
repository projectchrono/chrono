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

ChTire::ChTire(const std::string& name) : ChPart(name), m_slip_angle(0), m_longitudinal_slip(0), m_camber_angle(0) {}

// -----------------------------------------------------------------------------
// Base class implementation of the initialization function.
// -----------------------------------------------------------------------------
void ChTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    m_wheel = wheel;
    m_side = side;

    // Increment mass and inertia of the wheel body.
    wheel->SetMass(wheel->GetMass() + GetMass());
    wheel->SetInertiaXX(wheel->GetInertiaXX() + GetInertia());
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

// -----------------------------------------------------------------------------
// Estimate the tire moments of inertia, given the tire specification and mass.
// The tire is assumed to be composed of simple geometric shapes:
// - The sidewalls are treated as discs with an inner diameter equal to the
//   wheel diameter, and an outer diameter equal to the static diameter of the
//   tire.
// - The tread face is treated as a band with a diameter equal to the static
//   radius of the tire.
// A rubber material is assumed, using a density of 1050 kg/m^3.
// -----------------------------------------------------------------------------
double VolumeCyl(double r_outer, double r_inner, double cyl_height) {
    return CH_C_PI * cyl_height * (r_outer * r_outer - r_inner * r_inner);
}

double InertiaRotCyl(double mass, double r_outer, double r_inner) {
    return mass * (r_outer * r_outer + r_inner * r_inner) / 2.0;
}

double InertiaTipCyl(double mass, double r_outer, double r_inner, double cyl_height) {
    return mass * (3.0 * (r_outer * r_outer + r_inner * r_inner) + cyl_height * cyl_height) / 12.0;
}

ChVector<> ChTire::EstimateInertia(double tire_width,    // tire width [mm]
                                   double aspect_ratio,  // aspect ratio: height to width [percentage]
                                   double rim_diameter,  // rim diameter [in]
                                   double tire_mass,     // mass of the tire [kg]
                                   double t_factor       // tread to sidewall thickness factor
) {
    double rho = 1050.0;  // rubber density in kg/m^3

    double width = tire_width / 1000;             // tire width in meters
    double secth = (aspect_ratio / 100) * width;  // tire height in meters
    double r_rim = (rim_diameter / 2) * 0.0254;   // rim radius in meters
    double r_tire = r_rim + secth;                // tire radius in meters

    // Estimate sidewall thickness.
    double t = 0;
    double m_test = 0;
    while (tire_mass - m_test > 0.0) {
        t += 1e-6;
        m_test = rho * (VolumeCyl(r_tire, r_tire - t_factor * t, width - 2 * t) + 2 * VolumeCyl(r_tire - t, r_rim, t));
    }

    // Lateral sidewall offset.
    double r_steiner = (width - t) / 2.0;

    // Calculate mass and moments of inertia of tread section and sidewall, separately.
    double m_tread = rho * VolumeCyl(r_tire, r_tire - t_factor * t, width - 2 * t);
    double Irot_tread = InertiaRotCyl(m_tread, r_tire, r_tire - 2 * t);
    double Itip_tread = InertiaTipCyl(m_tread, r_tire, r_tire - t_factor * t, width - 2 * t);

    double m_sidewall = rho * VolumeCyl(r_tire - t, r_rim, t);
    double Irot_sidewall = InertiaRotCyl(m_sidewall, r_tire - t, r_rim);
    double Itip_sidewall = InertiaTipCyl(m_sidewall, r_tire - t, r_rim, t) + m_sidewall * pow(r_steiner, 2.0);

    // Return composite tire inertia.
    return ChVector<>(Itip_tread + 2 * Itip_sidewall, Irot_tread + 2 * Irot_sidewall, Itip_tread + 2 * Itip_sidewall);
}

}  // end namespace vehicle
}  // end namespace chrono
