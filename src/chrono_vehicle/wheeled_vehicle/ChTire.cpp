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

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

ChTire::ChTire(const std::string& name)
    : ChPart(name),
      m_collision_type(CollisionType::SINGLE_POINT),
      m_contact_surface_type(ContactSurfaceType::NODE_CLOUD),
      m_contact_surface_dim(0.01),
      m_collision_family(13),
      m_stepsize(1e-3),
      m_slip_angle(0),
      m_longitudinal_slip(0),
      m_camber_angle(0),
      m_pressure(0.0) {}

// -----------------------------------------------------------------------------

void ChTire::SetContactSurfaceType(ContactSurfaceType type, double dim, int collision_family) {
    m_contact_surface_type = type;
    m_contact_surface_dim = dim;
    m_collision_family = collision_family;
}

// -----------------------------------------------------------------------------
// Initialize this tire by associating it to the specified wheel.
// Increment the mass and inertia of the associated suspension spindle body to
// account for the tire mass and inertia.
// -----------------------------------------------------------------------------
void ChTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    m_wheel = wheel;
    m_parent = wheel;
    m_obj_tag = VehicleObjTag::Generate(GetVehicleTag(), VehiclePartTag::TIRE);

    //// RADU TODO
    //// Properly account for offset in adjusting inertia.
    //// This requires changing the spindle to a ChBodyAuxRef.
    wheel->GetSpindle()->SetMass(wheel->GetSpindle()->GetMass() + GetAddedMass());
    wheel->GetSpindle()->SetInertiaXX(wheel->GetSpindle()->GetInertiaXX() + GetAddedInertia());

    // Mark as initialized
    ChPart::Initialize();
}

// -----------------------------------------------------------------------------
// Calculate kinematics quantities (slip angle, longitudinal slip, camber angle,
// and toe-in angle) using the given state of the associated wheel.
// -----------------------------------------------------------------------------
void ChTire::CalculateKinematics(const WheelState& wheel_state, const ChCoordsys<>& tire_frame) {
    // Wheel normal (expressed in global frame)
    ChVector3d wheel_normal = wheel_state.rot.GetAxisY();

    // Express wheel linear velocity in tire frame
    ChVector3d V = tire_frame.TransformDirectionParentToLocal(wheel_state.lin_vel);

    // Express wheel normal in tire frame
    ChVector3d n = tire_frame.TransformDirectionParentToLocal(wheel_normal);

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
std::shared_ptr<ChVisualShapeTriangleMesh> ChTire::AddVisualizationMesh(const std::string& mesh_file_left,
                                                                        const std::string& mesh_file_right) {
    bool left = (m_wheel->GetSide() == VehicleSide::LEFT);
    ChQuaternion<> rot = left ? QuatFromAngleZ(0) : QuatFromAngleZ(CH_PI);
    m_vis_mesh_file = left ? mesh_file_left : mesh_file_right;

    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetVehicleDataFile(m_vis_mesh_file), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
    m_wheel->GetSpindle()->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, GetOffset(), 0), ChMatrix33<>(rot)));

    return trimesh_shape;
}

// -----------------------------------------------------------------------------
// Utility functions for characterizing the geometric contact between a disc with specified center location, normal
// direction, and radius and the terrain, assumed to be specified as a height field (over the x-y domain). These
// functions return false if no contact occurs. Otherwise, they set the contact points on the disc (ptD) and on the
// terrain (ptT), the normal contact direction, and the resulting penetration depth (a positive value).
//
// The first version uses a single point on the terrain.
// The second version uses the average of four terrain heights.
// The third version uses the collision algorithm of Sui and Hirshey.
//
// NOTE: uses terrain normal at disc center for approximate calculation. Hence, only valid for terrains with constant
// slope. A completely accurate solution would require an iterative calculation of the contact point.
// -----------------------------------------------------------------------------

bool ChTire::DiscTerrainCollision(CollisionType method,                // collision detection method
                                  const ChTerrain& terrain,            // reference to terrain system
                                  const ChVector3d& C,                 // disc center (in global frame)
                                  const ChVector3d& dn,                // disc normal (in global frame)
                                  double r,                            // disc radius
                                  double w,                            // tire width
                                  const ChFunctionInterp& area2depth,  // contact frame (in global frame)
                                  ChCoordsys<>& contact,               // contact frame (in global frame)
                                  double& depth,                       // penetration (positive if contact)
                                  float& mu                            // coefficient of friction
) {
    switch (method) {
        default:
        case CollisionType::SINGLE_POINT:
            return DiscTerrainCollision1pt(terrain, C, dn, r, contact, depth, mu);
        case CollisionType::FOUR_POINTS:
            return DiscTerrainCollision4pt(terrain, C, dn, r, w, contact, depth, mu);
        case CollisionType::ENVELOPE:
            return DiscTerrainCollisionEnvelope(terrain, C, dn, r, w, area2depth, contact, depth, mu);
    }
}

// Find intersection point B of line (A, v) with a circle (C, r) -> B
// Line and circle are assumed co-planar (no check)
bool LineCircleIntersection(const ChVector3d& C,  // circle center
                            double r,             // circle radius
                            const ChVector3d& A,  // point on line
                            const ChVector3d& v,  // line versor
                            int dir,              // find point in the direction sgn(dir) * v
                            ChVector3d& B         // intersection point (if any)
) {
    // Solve quadratic a*t^2 + 2*b*t + c = 0
    // with a = Vdot(v,v) = 1
    ChVector3d w = A - C;
    double b = Vdot(v, w);
    double c = Vdot(w, w) - r * r;

    double delta = b * b - c;
    if (delta < 0)
        return false;

    double t = -b + std::sqrt(delta);
    if (dir < 0)
        t *= -1;

    B = A + t * v;

    return true;
}

// Project point A on plane (P,n) -> B
void PointPlaneProjection(const ChVector3d& A,  // point to be projected
                          const ChVector3d& P,  // point in plane
                          const ChVector3d& n,  // plane normal
                          ChVector3d& B         // projected point
) {
    double dist = Vdot((A - P), n);
    B = A - dist * n;
}

bool ChTire::DiscTerrainCollision1pt(const ChTerrain& terrain,  // reference to terrain system
                                     const ChVector3d& C,       // disc center (in global frame)
                                     const ChVector3d& dn,      // disc normal (in global frame)
                                     double r,                  // disc radius
                                     ChCoordsys<>& contact,     // contact frame (in global frame)
                                     double& depth,             // penetration (positive if contact)
                                     float& mu                  // coefficient of friction
) {
    // Vertical offset for terrain queries (above terrain by disc diameter)
    auto voffset = 2 * r * ChWorldFrame::Vertical();

    // Calculate horizontal direction in disc plane
    ChVector3d df = Vcross(dn, ChWorldFrame::Vertical());
    df.Normalize();

    // Find lowest point on disc in the direction of the world vertical
    ChVector3d B = C + r * Vcross(dn, df);

    // Query terrain at bottom point
    double h;
    ChVector3d P;
    ChVector3d n;
    terrain.GetProperties(B + voffset, P, h, n, mu);

    // No contact if the disc center is below the terrain.
    double dh = ChWorldFrame::Height(C);
    if (dh <= h)
        return false;

    // Project terrain normal onto disc plane
    // n <- dn X ( n X dn ) = n - (n.dn)dn
    n -= Vdot(n, dn) * dn;

    // -------
    // Approximate terrain with a plane (P, n)
    // Use disc center as reference point A
    // -------

    ChVector3d A = C;

    // Intersect line (A, -n) with the circle (C, r)
    // (no need to use LineCircleIntersection here)
    // (B not really needed here, since AB=r)
    B = A - r * n;

    // Project reference point onto terrain plane
    ChVector3d Pp;
    n.Normalize();
    PointPlaneProjection(A, P, n, Pp);

    // Calculate depth (along n)
    double AB = (B - A).Length();
    double AP = (Pp - A).Length();
    depth = AB - AP;
    if (depth <= 0)
        return false;

    // Query terrain at projection point
    terrain.GetProperties(Pp + voffset, Pp, h, n, mu);

    // Construct contact patch frame from P and n
    ChVector3d fwd = Vcross(dn, n);
    // Note that both dn and n are normalized; therefore, fwd will be too
    ChVector3d lat = Vcross(n, fwd);
    ChMatrix33<> rot(fwd, lat, n);
    contact.pos = Pp;
    contact.rot = rot.GetQuaternion();

    return true;
}

bool ChTire::DiscTerrainCollision4pt(const ChTerrain& terrain,  // reference to terrain system
                                     const ChVector3d& C,       // disc center (in global frame)
                                     const ChVector3d& dn,      // disc normal (in global frame)
                                     double r,                  // disc radius
                                     double w,                  // tire width
                                     ChCoordsys<>& contact,     // contact frame (in global frame)
                                     double& depth,             // penetration (positive if contact)
                                     float& mu                  // coefficient of friction
) {
    // Vertical offset for terrain queries (above terrain by disc diameter)
    auto voffset = 2 * r * ChWorldFrame::Vertical();

    // Calculate horizontal direction in disc plane
    ChVector3d df = Vcross(dn, ChWorldFrame::Vertical());
    df.Normalize();

    // Find lowest point on disc in the direction of the world vertical
    ChVector3d B = C + r * Vcross(dn, df);

    // Query terrain at bottom point
    double h;
    ChVector3d P;
    ChVector3d n;
    terrain.GetProperties(B + voffset, P, h, n, mu);

    // No contact if the disc center is below the terrain.
    double dh = ChWorldFrame::Height(C);
    if (dh <= h)
        return false;

    // Project terrain normal onto disc plane
    // n <- dn X ( n X dn ) = n - (n.dn)dn
    n -= Vdot(n, dn) * dn;

    // -------
    // Approximate terrain with a plane (P, n)
    // Find 4 reference points around disc center
    // -------

    ChVector3d fwd = Vcross(dn, n);
    fwd.Normalize();
    ChVector3d lat = Vcross(n, fwd);

    int num_points = 4;
    double dx = 0.1 * r;
    double dy = 0.3 * w;
    std::array<ChVector3d, 4> Q = {C + dx * fwd, C - dx * fwd, C + dy * lat, C - dy * lat};

    // Process each reference points and accumulate
    depth = 0;
    P = VNULL;
    for (const ChVector3d& A : Q) {
        // Intersect line (A, -n) with the circle (C, r)
        bool check = LineCircleIntersection(C, r, A, n, -1, B);
        if (!check) {
            num_points--;
            continue;
        }

        // Project reference point onto terrain plane
        ChVector3d Pp;
        PointPlaneProjection(A, P, n, Pp);

        // Calculate depth (along n) and update
        double AB = (B - A).Length();
        double AP = (Pp - A).Length();
        depth += AB - AP;

        // Query terrain at projection point
        terrain.GetProperties(Pp + voffset, Pp, h, n, mu);

        // Accumulate Pp
        P += Pp;
    }

    depth /= num_points;
    if (depth <= 0)
        return false;

    // Query terrain at average projection
    P /= num_points;
    terrain.GetProperties(P + voffset, P, h, n, mu);

    // Construct contact patch frame from P and n
    fwd = Vcross(dn, n);
    fwd.Normalize();
    lat = Vcross(n, fwd);
    ChMatrix33<> rot(fwd, lat, n);
    contact.pos = P;
    contact.rot = rot.GetQuaternion();

    return true;
}

void ChTire::ConstructAreaDepthTable(double r, ChFunctionInterp& area2depth) {
    const size_t n = 90;
    double depth_max = r;  // should be high enough to avoid extrapolation
    double depth_step = depth_max / double(n - 1);
    for (size_t i = 0; i < n; i++) {
        double depth = i * depth_step;
        double alpha = 2 * std::acos(1 - depth / r);
        double area = 0.5 * r * r * (alpha - std::sin(alpha));
        area2depth.AddPoint(area, depth);
    }
}

bool ChTire::DiscTerrainCollisionEnvelope(const ChTerrain& terrain,            // reference to terrain system
                                          const ChVector3d& C,                 // disc center (in global frame)
                                          const ChVector3d& dn,                // disc normal (in global frame)
                                          double r,                            // disc radius
                                          double w,                            // tire width
                                          const ChFunctionInterp& area2depth,  // depth from area lookup table
                                          ChCoordsys<>& contact,               // contact frame (in global frame)
                                          double& depth,                       // penetration (positive if contact)
                                          float& mu                            // coefficient of friction
) {
    // Vertical offset for terrain queries (above terrain by a disc radius)
    auto voffset = r * ChWorldFrame::Vertical();

    // Calculate horizontal direction in disc plane
    ChVector3d df = Vcross(dn, ChWorldFrame::Vertical());
    df.Normalize();

    // Find lowest point on disc in the direction of the world vertical
    ChVector3d B = C + r * Vcross(dn, df);

    // Query terrain at bottom point
    double h;
    ChVector3d P;
    ChVector3d n;
    terrain.GetProperties(B + voffset, P, h, n, mu);

    // No contact if the disc center is below the terrain.
    double dh = ChWorldFrame::Height(C);
    if (dh <= h)
        return false;

    // Project terrain normal onto disc plane
    // n <- dn X ( n X dn ) = n - (n.dn)dn
    n -= Vdot(n, dn) * dn;

    // -------
    // Approximate terrain with a plane (P, n)
    // Work in the plane of the disc and query actual terrain height
    // -------

    ChVector3d A;

    // Calculate overlapping area of tire disc and terrain surface contour
    const size_t n_div = 100;
    double x_step = 2 * r / n_div;
    double area = 0;
    for (size_t i = 0; i <= n_div; i++) {
        double x = -r + i * x_step;
        A = C + x * df;                              // query point
        double ph = ChWorldFrame::Height(A);         // query point height
        double th = terrain.GetHeight(A + voffset);  // terrain height at query point
        dh = ph - std::sqrt(r * r - x * x);          // disc point height
        depth = th - dh;                             // depth at query point
        ////std::cout << x << " " << ph << " " << th << " " << dh << std::endl;
        if (depth > 0) {
            area += depth;
        }
    }

    if (area == 0)
        return false;

    area *= x_step;

    // Calculate equivalent depth from A
    depth = area2depth.GetVal(area);

    if (depth > r)
        return false;

    // Find candidate point on terrain
    P = C + (r - depth) * n;

    // Query terrain at this point
    terrain.GetProperties(P + voffset, P, h, n, mu);

    // Construct contact patch frame from P and n
    ChVector3d fwd = Vcross(dn, n);
    fwd.Normalize();
    ChVector3d lat = Vcross(n, fwd);
    ChMatrix33<> rot(fwd, lat, n);
    contact.pos = P;
    contact.rot = rot.GetQuaternion();

    return true;
}

// -----------------------------------------------------------------------------
// Estimate the tire moments of inertia, given the tire specification and mass.
// The tire is assumed to be composed of simple geometric shapes:
// - The sidewalls are treated as discs with an inner diameter equal to the wheel diameter,
//   and an outer diameter equal to the static diameter of the  tire.
// - The tread face is treated as a band with a diameter equal to the static  radius of the tire.
// A rubber material is assumed, using a density of 1050 kg/m^3.
// -----------------------------------------------------------------------------

double VolumeCyl(double r_outer, double r_inner, double cyl_height) {
    return CH_PI * cyl_height * (r_outer * r_outer - r_inner * r_inner);
}

double InertiaRotCyl(double mass, double r_outer, double r_inner) {
    return mass * (r_outer * r_outer + r_inner * r_inner) / 2.0;
}

double InertiaTipCyl(double mass, double r_outer, double r_inner, double cyl_height) {
    return mass * (3.0 * (r_outer * r_outer + r_inner * r_inner) + cyl_height * cyl_height) / 12.0;
}

ChVector3d ChTire::EstimateInertia(double tire_width,    // tire width [mm]
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
    double Itip_sidewall = InertiaTipCyl(m_sidewall, r_tire - t, r_rim, t) + m_sidewall * std::pow(r_steiner, 2.0);

    // Return composite tire inertia.
    return ChVector3d(Itip_tread + 2 * Itip_sidewall, Irot_tread + 2 * Irot_sidewall, Itip_tread + 2 * Itip_sidewall);
}

}  // end namespace vehicle
}  // end namespace chrono
