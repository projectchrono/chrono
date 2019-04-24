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

ChTire::ChTire(const std::string& name)
    : ChPart(name), m_stepsize(1e-3), m_slip_angle(0), m_longitudinal_slip(0), m_camber_angle(0) {}

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
// Default implementation of ReportMass simply returns the value from GetMass.
// However, concrete tire models which need to return 0 from GetMass (so that
// the mass of the tire is not double counted) will override this function.
// -----------------------------------------------------------------------------
double ChTire::ReportMass() const {
    return GetMass();
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

    // Slip angle (positive sign = left turn, negative sign = right turn)
    double abs_Vx = std::abs(V.x());
    double zero_Vx = 1e-4;
    m_slip_angle = (abs_Vx > zero_Vx) ? std::atan(V.y() / abs_Vx) : 0;

    // Longitudinal slip (positive sign = driving, negative sign = breaking)
    m_longitudinal_slip = (abs_Vx > zero_Vx) ? -(V.x() - state.omega * GetRadius()) / abs_Vx : 0;

    // Camber angle (positive sign = upper side tipping to the left, negative sign = upper side tipping to the right)
    m_camber_angle = std::atan2(n.z(), n.y());
}

// -----------------------------------------------------------------------------
// Utility functions for characterizing the geometric contact between a disc with
// specified center location, normal direction, and radius and the terrain,
// assumed to be specified as a height field (over the x-y domain).
// These functions returns false if no contact occurs.
// Otherwise, they set the contact points on the disc (ptD) and on the terrain (ptT),
// the normal contact direction, and the resulting penetration depth (a positive value).
//
// The first version uses a single point on the terrain (below the wheel center).
// The second version uses the average of four terrain heights below the wheel center.
// The third version uses the collision algorithm of Sui and Hirshey.
//
// NOTE: uses terrain normal at disc center for approximative calculation.
// Hence only valid for terrains with constant slope. A completely accurate
// solution would require an iterative calculation of the contact point.
// -----------------------------------------------------------------------------
bool ChTire::DiscTerrainCollision(
    const ChTerrain& terrain,       // [in] reference to terrain system
    const ChVector<>& disc_center,  // [in] global location of the disc center
    const ChVector<>& disc_normal,  // [in] disc normal, expressed in the global frame
    double disc_radius,             // [in] disc radius
    ChCoordsys<>& contact,          // [out] contact coordinate system (relative to the global frame)
    double& depth)                  // [out] penetration depth (positive if contact occurred)
{
    // Find terrain height below disc center. There is no contact if the disc
    // center is below the terrain or farther away by more than its radius.
    double hc = terrain.GetHeight(disc_center.x(), disc_center.y());
    if (disc_center.z() <= hc || disc_center.z() >= hc + disc_radius)
        return false;

    // Find the lowest point on the disc. There is no contact if the disc is
    // (almost) horizontal.
    ChVector<> nhelp = terrain.GetNormal(disc_center.x(), disc_center.y());
    ChVector<> dir1 = Vcross(disc_normal, nhelp);
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

bool ChTire::DiscTerrainCollision4pt(
    const ChTerrain& terrain,       // [in] reference to terrain system
    const ChVector<>& disc_center,  // [in] global location of the disc center
    const ChVector<>& disc_normal,  // [in] disc normal, expressed in the global frame
    double disc_radius,             // [in] disc radius
    double width,                   // [in] tire width
    ChCoordsys<>& contact,          // [out] contact coordinate system (relative to the global frame)
    double& depth,                  // [out] penetration depth (positive if contact occurred),
    double& camber_angle)           // [out] camber angle
{
    double dx = 0.1 * disc_radius;
    double dy = 0.3 * width;

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

    // Approximate the terrain with a plane. Define the projection of the lowest
    // point onto this plane as the contact point on the terrain.
    ChVector<> normal = terrain.GetNormal(ptD.x(), ptD.y());
    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);

    // Calculate four contact points in the contact patch
    ChVector<> ptQ1 = ptD + dx * longitudinal;
    ptQ1.z() = terrain.GetHeight(ptQ1.x(), ptQ1.y());

    ChVector<> ptQ2 = ptD - dx * longitudinal;
    ptQ2.z() = terrain.GetHeight(ptQ2.x(), ptQ2.y());

    ChVector<> ptQ3 = ptD + dy * lateral;
    ptQ3.z() = terrain.GetHeight(ptQ3.x(), ptQ3.y());

    ChVector<> ptQ4 = ptD - dy * lateral;
    ptQ4.z() = terrain.GetHeight(ptQ4.x(), ptQ4.y());

    // Calculate a smoothed road surface normal
    ChVector<> rQ2Q1 = ptQ1 - ptQ2;
    ChVector<> rQ4Q3 = ptQ3 - ptQ4;

    ChVector<> terrain_normal = Vcross(rQ2Q1, rQ4Q3);
    terrain_normal.Normalize();

    // Find terrain height as average of four points. No contact if lowest point is above
    // the terrain.
    ptD = 0.25 * (ptQ1 + ptQ2 + ptQ3 + ptQ4);
    ChVector<> d = ptD - disc_center;
    double da = d.Length();

    if (da >= disc_radius)
        return false;

    // Calculate an improved value for the camber angle
    camber_angle = std::asin(Vdot(disc_normal, terrain_normal));

    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, terrain_normal);

    contact.pos = ptD;
    contact.rot = rot.Get_A_quaternion();

    depth = disc_radius - da;
    assert(depth > 0);

    return true;
}

void ChTire::ConstructAreaDepthTable(double disc_radius, ChFunction_Recorder& areaDep) {
    const size_t n_lookup = 30;
    double depMax = disc_radius;  // should be high enough to avoid extrapolation
    double depStep = depMax / double(n_lookup - 1);
    for (size_t i = 0; i < n_lookup; i++) {
        double dep = depStep * double(i);
        double alpha = 2.0 * acos(1.0 - dep / disc_radius);
        double area = 0.5 * disc_radius * disc_radius * (alpha - sin(alpha));
        areaDep.AddPoint(area, dep);
    }
}

bool ChTire::DiscTerrainCollisionEnvelope(
    const ChTerrain& terrain,            // [in] reference to terrain system
    const ChVector<>& disc_center,       // [in] global location of the disc center
    const ChVector<>& disc_normal,       // [in] disc normal, expressed in the global frame
    double disc_radius,                  // [in] disc radius
    const ChFunction_Recorder& areaDep,  // [in] lookup table to calculate depth from intersection area
    ChCoordsys<>& contact,               // [out] contact coordinate system (relative to the global frame)
    double& depth                        // [out] penetration depth (positive if contact occurred)
) {
    // Approximate the terrain with a plane. Define the projection of the lowest
    // point onto this plane as the contact point on the terrain. We don't know
    // where the equivalent contact point exactly is, so we employ the intersection
    // area to decide if there is contact or not.
    // First guess:
    ChVector<> normal = terrain.GetNormal(disc_center.x(), disc_center.y());
    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    const size_t n_con_pts = 31;
    double x_step = 2.0 * disc_radius / double(n_con_pts - 1);
    // ChVectorDynamic<> q(n_con_pts);  // road surface height values along 'longitudinal'
    // ChVectorDynamic<> x(n_con_pts);  // x values along disc_center + x*longitudinal
    double A = 0;   // overlapping area of tire disc and road surface contour
    double Xc = 0;  // relative x coordinate of area A centroid
    double Zc = 0;  // relative z (rsp. to road height) height of area A centroid, actually unused
    for (size_t i = 0; i < n_con_pts; i++) {
        double x = -disc_radius + x_step * double(i);
        ChVector<> pTest = disc_center + x * longitudinal;
        double q = terrain.GetHeight(pTest.x(), pTest.y());
        double a = pTest.z() - sqrt(disc_radius * disc_radius - x * x);
        double Q1 = q - a;
        double Q2 = q + a;
        if (Q1 < 0) {
            Q1 = 0;
        }
        if (i == 0 || i == (n_con_pts - 1)) {
            A += 0.5 * Q1;
            Xc += 0.5 * Q1 * x;
            // Zc += 0.5 * Q1 * Q2 / 2.0;
        } else {
            A += Q1;
            Xc += Q1 * x;
            // Zc += Q1 * Q2 / 2.0;
        }
    }
    A *= x_step;
    Xc *= x_step / A;
    // Zc *= x_step / A;
    if (A == 0) {
        return false;
    }

    // Xc = negative means area centroid is in front of the disc_center
    ChVector<> pXc = disc_center - Xc * longitudinal;

    // Zc relative to q(x)
    // Zc = terrain.GetHeight(disc_center.x(), disc_center.y()) + Zc;

    // GetLog() << "A = " << A << "  Xc = " << Xc << "  Yc = " << Yc << "\n";

    // calculate equivalent depth from A
    depth = areaDep.Get_y(A);
    // Find the lowest point on the disc. There is no contact if the disc is
    // (almost) horizontal.
    // ChVector<> nhelp = terrain.GetNormal(disc_center.x(), disc_center.y());
    ChVector<> nhelp = terrain.GetNormal(pXc.x(), pXc.y());
    ChVector<> dir1 = Vcross(disc_normal, nhelp);
    double sinTilt2 = dir1.Length2();

    if (sinTilt2 < 1e-3)
        return false;

    // Contact point (lowest point on disc).
    ChVector<> ptD = disc_center + (disc_radius - depth) * Vcross(disc_normal, dir1 / sqrt(sinTilt2));

    // Find terrain height at lowest point. No contact if lowest point is above
    // the terrain.

    normal = terrain.GetNormal(ptD.x(), ptD.y());
    longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);
    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, normal);

    contact.pos = ptD;
    contact.rot = rot.Get_A_quaternion();

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
