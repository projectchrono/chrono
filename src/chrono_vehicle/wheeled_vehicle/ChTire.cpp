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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

ChTire::ChTire(const std::string& name)
    : ChPart(name),
      m_collision_type(CollisionType::SINGLE_POINT),
      m_stepsize(1e-3),
      m_slip_angle(0),
      m_longitudinal_slip(0),
      m_camber_angle(0) {}

// -----------------------------------------------------------------------------
// Initialize this tire by associating it to the specified wheel.
// Increment the mass and inertia of the associated suspension spindle body to
// account for the tire mass and inertia.
// -----------------------------------------------------------------------------
void ChTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    m_wheel = wheel;

    //// RADU TODO 
    //// Properly account for offset in adjusting inertia.
    //// This requires changing the spindle to a ChBodyAuxRef.
    wheel->GetSpindle()->SetMass(wheel->GetSpindle()->GetMass() + GetAddedMass());
    wheel->GetSpindle()->SetInertiaXX(wheel->GetSpindle()->GetInertiaXX() + GetAddedInertia());
}

// -----------------------------------------------------------------------------
// Calculate kinematics quantities (slip angle, longitudinal slip, camber angle,
// and toe-in angle) using the given state of the associated wheel.
// -----------------------------------------------------------------------------
void ChTire::CalculateKinematics(const WheelState& wheel_state, 
                                 const ChCoordsys<>& tire_frame) {
    // Wheel normal (expressed in global frame)
    ChVector<> wheel_normal = wheel_state.rot.GetYaxis();

    // Express wheel linear velocity in tire frame
    ChVector<> V = tire_frame.TransformDirectionParentToLocal(wheel_state.lin_vel);

    // Express wheel normal in tire frame
    ChVector<> n = tire_frame.TransformDirectionParentToLocal(wheel_normal);

    // Slip angle (positive sign = left turn, negative sign = right turn)
    double abs_Vx = std::abs(V.x());
    double zero_Vx = 1e-4;
    m_slip_angle = (abs_Vx > zero_Vx) ? std::atan(V.y() / abs_Vx) : 0;

    // Longitudinal slip (positive sign = driving, negative sign = breaking)
    m_longitudinal_slip = (abs_Vx > zero_Vx) ? -(V.x() - wheel_state.omega * GetRadius()) / abs_Vx : 0;

    // Camber angle (positive sign = upper side tipping to the left, negative sign = upper side tipping to the right)
    m_camber_angle = std::atan2(n.z(), n.y());
}

// -----------------------------------------------------------------------------
// Utility functions for adding/removing a mesh visualization asset to this tire
// -----------------------------------------------------------------------------

// Add visualization mesh: use one of the two provided OBJ files, depending on the side on which the tire is mounted.
std::shared_ptr<ChTriangleMeshShape> ChTire::AddVisualizationMesh(const std::string& mesh_file_left,
                                                                  const std::string& mesh_file_right) {
    bool left = (m_wheel->GetSide() == VehicleSide::LEFT);
    ChQuaternion<> rot = left ? Q_from_AngZ(0) : Q_from_AngZ(CH_C_PI);
    m_vis_mesh_file = left ? mesh_file_left : mesh_file_right;

    auto trimesh =
        geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_vis_mesh_file), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
    trimesh_shape->SetMutable(false);
    m_wheel->GetSpindle()->AddVisualShape(trimesh_shape, ChFrame<>(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(rot)));

    return trimesh_shape;
}

// -----------------------------------------------------------------------------
// Utility functions for characterizing the geometric contact between a disc with
// specified center location, normal direction, and radius and the terrain,
// assumed to be specified as a height field (over the x-y domain).
// These functions return false if no contact occurs.
// Otherwise, they set the contact points on the disc (ptD) and on the terrain (ptT),
// the normal contact direction, and the resulting penetration depth (a positive value).
//
// The first version uses a single point on the terrain.
// The second version uses the average of four terrain heights.
// The third version uses the collision algorithm of Sui and Hirshey.
//
// NOTE: uses terrain normal at disc center for approximate calculation.
// Hence, only valid for terrains with constant slope. A completely accurate
// solution would require an iterative calculation of the contact point.
// -----------------------------------------------------------------------------
bool ChTire::DiscTerrainCollision(
    CollisionType method,                // [in] tire-terrain collision detection method
    const ChTerrain& terrain,            // [in] reference to terrain system
    const ChVector<>& disc_center,       // [in] global location of the disc center
    const ChVector<>& disc_normal,       // [in] disc normal, expressed in the global frame
    double disc_radius,                  // [in] disc radius
    double width,                        // [in] tire width
    const ChFunction_Recorder& areaDep,  // [in] lookup table to calculate depth from intersection area
    ChCoordsys<>& contact,               // [out] contact coordinate system (relative to the global frame)
    double& depth,                       // [out] penetration depth (positive if contact occurred)
    float& mu                            // [out] coefficient of friction at contact
) {
    switch (method) {
        default:
        case CollisionType::SINGLE_POINT:
            return DiscTerrainCollision1pt(terrain, disc_center, disc_normal, disc_radius, contact, depth, mu);
        case CollisionType::FOUR_POINTS:
            return DiscTerrainCollision4pt(terrain, disc_center, disc_normal, disc_radius, width, contact, depth, mu);
        case CollisionType::ENVELOPE:
            return DiscTerrainCollisionEnvelope(terrain, disc_center, disc_normal, disc_radius, width, areaDep, contact,
                                                depth, mu);
    }
}

bool ChTire::DiscTerrainCollision1pt(
    const ChTerrain& terrain,       // [in] reference to terrain system
    const ChVector<>& disc_center,  // [in] global location of the disc center
    const ChVector<>& disc_normal,  // [in] disc normal, expressed in the global frame
    double disc_radius,             // [in] disc radius
    ChCoordsys<>& contact,          // [out] contact coordinate system (relative to the global frame)
    double& depth,                  // [out] penetration depth (positive if contact occurred)
    float& mu                       // [out] coefficient of friction at contact
) {
    // Find the location of the lowest point on the wheel disc in the direction of the world vertical.
    ChVector<> wheel_forward = Vcross(disc_normal, ChWorldFrame::Vertical());
    wheel_forward.Normalize();
    ChVector<> wheel_bottom_location = disc_center + disc_radius * Vcross(disc_normal, wheel_forward);

    // Find terrain height, normal, and friction at this point on the wheel disc.
    double hc;
    ChVector<> normal;
    terrain.GetProperties(wheel_bottom_location, hc, normal, mu);

    // No contact if the disc center is below the terrain.
    double disc_height = ChWorldFrame::Height(disc_center);
    if (disc_height <= hc)
        return false;

    // Calculate the contact depth at this point.
    double hc_height = ChWorldFrame::Height(wheel_bottom_location);
    depth = (hc - hc_height) * Vdot(ChWorldFrame::Vertical(), normal);

    // Based on the sampled normal we now do a first order approximation of where the contact point
    // would be. We also will estimate the contact depth at that point.
    ChVector<> wheel_forward_normal = Vcross(disc_normal, normal);

    // There is no contact if the disc is (almost) horizontal, so bail out in that case.
    double sinTilt2 = wheel_forward_normal.Length2();
    if (sinTilt2 < 1e-3)
        return false;

    wheel_forward_normal.Normalize();
    // Now re-calculate the depth.
    depth = disc_radius - ((disc_radius - depth) * Vdot(wheel_forward, wheel_forward_normal));
    // At this point we should check if our wheel still touches the ground and bail out if it does not.
    if (depth <= 0.0)
        return false;

    // And we re-calculate the contact point.
    wheel_bottom_location = disc_center + (disc_radius - depth) * Vcross(disc_normal, wheel_forward_normal);

    // Approximate the terrain with a plane. Define the projection of the lowest
    // point onto this plane as the contact point on the terrain.
    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);
    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, normal);

    contact.pos = wheel_bottom_location;
    contact.rot = rot.Get_A_quaternion();

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
    float& mu                       // [out] coefficient of friction at contact
) {
    double dx = 0.1 * disc_radius;
    double dy = 0.3 * width;

    // Find the location of the lowest point on the wheel disc in the direction of the world vertical.
    ChVector<> wheel_forward = Vcross(disc_normal, ChWorldFrame::Vertical());
    wheel_forward.Normalize();
    ChVector<> wheel_bottom_location = disc_center + disc_radius * Vcross(disc_normal, wheel_forward);

    // Find terrain height, normal, and friction at the this point on the wheel disc.
    double hc;
    ChVector<> normal;
    terrain.GetProperties(wheel_bottom_location, hc, normal, mu);

    // No contact if the disc center is below the terrain.
    double disc_height = ChWorldFrame::Height(disc_center);
    if (disc_height <= hc)
        return false;

    // Based on the sampled normal we now do a first order approximation of where the contact point
    // would be. We also will estimate the contact depth at that point.
    ChVector<> wheel_forward_normal = Vcross(disc_normal, normal);

    // There is no contact if the disc is (almost) horizontal, so bail out in that case.
    double sinTilt2 = wheel_forward_normal.Length2();
    if (sinTilt2 < 1e-3)
        return false;

    wheel_forward_normal.Normalize();

    // And we re-calculate the contact point.
    wheel_bottom_location = disc_center + disc_radius * Vcross(disc_normal, wheel_forward_normal);

    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);

    // Calculate four contact points in the contact patch
    ChVector<> ptQ1 = wheel_bottom_location + dx * longitudinal;
    double hQ1 = terrain.GetHeight(ptQ1);
    double ptQ1_height = ChWorldFrame::Height(ptQ1);
    ptQ1 = ptQ1 - (ptQ1_height - hQ1) * ChWorldFrame::Vertical();

    ChVector<> ptQ2 = wheel_bottom_location - dx * longitudinal;
    double hQ2 = terrain.GetHeight(ptQ2);
    double ptQ2_height = ChWorldFrame::Height(ptQ2);
    ptQ2 = ptQ2 - (ptQ2_height - hQ2) * ChWorldFrame::Vertical();

    ChVector<> ptQ3 = wheel_bottom_location + dy * lateral;
    double hQ3 = terrain.GetHeight(ptQ3);
    double ptQ3_height = ChWorldFrame::Height(ptQ3);
    ptQ3 = ptQ3 - (ptQ3_height - hQ3) * ChWorldFrame::Vertical();

    ChVector<> ptQ4 = wheel_bottom_location - dy * lateral;
    double hQ4 = terrain.GetHeight(ptQ4);
    double ptQ4_height = ChWorldFrame::Height(ptQ4);
    ptQ4 = ptQ4 - (ptQ4_height - hQ4) * ChWorldFrame::Vertical();

    // Calculate a smoothed road surface normal
    ChVector<> rQ2Q1 = ptQ1 - ptQ2;
    ChVector<> rQ4Q3 = ptQ3 - ptQ4;

    ChVector<> terrain_normal = Vcross(rQ2Q1, rQ4Q3);
    terrain_normal.Normalize();

    // Find terrain height as average of four points. No contact if lowest point is above the terrain.
    wheel_bottom_location = 0.25 * (ptQ1 + ptQ2 + ptQ3 + ptQ4);
    ChVector<> d = wheel_bottom_location - disc_center;
    double da = d.Length();

    if (da >= disc_radius)
        return false;

    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, terrain_normal);

    contact.pos = wheel_bottom_location;
    contact.rot = rot.Get_A_quaternion();

    depth = disc_radius - da;
    assert(depth > 0);

    return true;
}

void ChTire::ConstructAreaDepthTable(double disc_radius, ChFunction_Recorder& areaDep) {
    const size_t n_lookup = 90;
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
    double width,                        // [in] tire width
    const ChFunction_Recorder& areaDep,  // [in] lookup table to calculate depth from intersection area
    ChCoordsys<>& contact,               // [out] contact coordinate system (relative to the global frame)
    double& depth,                       // [out] penetration depth (positive if contact occurred)
    float& mu                            // [out] coefficient of friction at contact
) {
    // Approximate the terrain with a plane. Define the projection of the lowest
    // point onto this plane as the contact point on the terrain. We don't know
    // where the equivalent contact point is exactly, so we use the intersection
    // area to decide if there is contact or not.

    ChVector<> normal = terrain.GetNormal(disc_center);
    ChVector<> longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();

    const size_t n_div = 180;
    double x_step = 2.0 * disc_radius / n_div;
    double A = 0;  // overlapping area of tire disc and road surface contour
    for (size_t i = 1; i < n_div; i++) {
        double x = -disc_radius + x_step * double(i);
        ChVector<> pTest = disc_center + x * longitudinal;
        double q = terrain.GetHeight(pTest);
        double a = ChWorldFrame::Height(pTest) - sqrt(disc_radius * disc_radius - x * x);
        if (q > a) {
            A += q - a;
        }
    }
    A *= x_step;

    if (A == 0) {
        return false;
    }

    // Calculate equivalent depth from A
    depth = areaDep.Get_y(A);

    // Find the lowest point on the disc. There is no contact if the disc is (almost) horizontal.
    ChVector<> dir1 = Vcross(disc_normal, normal);
    double sinTilt2 = dir1.Length2();

    if (sinTilt2 < 1e-3)
        return false;

    // Contact point (lowest point on disc).
    ChVector<> ptD = disc_center + (disc_radius - depth) * Vcross(disc_normal, dir1 / sqrt(sinTilt2));

    // Find terrain height at lowest point. No contact if lowest point is above the terrain.

    normal = terrain.GetNormal(ptD);
    longitudinal = Vcross(disc_normal, normal);
    longitudinal.Normalize();
    ChVector<> lateral = Vcross(normal, longitudinal);
    ChMatrix33<> rot;
    rot.Set_A_axis(longitudinal, lateral, normal);

    contact.pos = ptD;
    contact.rot = rot.Get_A_quaternion();

    mu = terrain.GetCoefficientFriction(ptD);

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
