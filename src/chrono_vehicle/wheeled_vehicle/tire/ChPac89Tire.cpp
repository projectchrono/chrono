// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Template for a tire model based on the MSC ADAMS PAC89 Tire Model
//
// Ref: Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// This implementation does not include transient slip state modifications.
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT
//  - Still need to check F&M Outputs
//  - Need to check to see if there is a need to account for what side the tire
//    is mounted on
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPac89Tire::ChPac89Tire(const std::string& name)
    : ChTire(name), m_kappa(0), m_alpha(0), m_gamma(0), m_gamma_limit(3), m_mu(0), m_mu0(0.8) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac89Tire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetPac89Params();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = std::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = m_unloaded_radius;
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetVisualizationWidth() / 2, 0);
    m_wheel->AddAsset(m_cyl_shape);

    m_texture = std::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChPac89Tire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChPac89Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_cyl_shape);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_texture);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac89Tire::Synchronize(double time,
                              const WheelState& wheel_state,
                              const ChTerrain& terrain,
                              CollisionType collision_type) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain, collision_type);

    m_mu = terrain.GetCoefficientFriction(m_tireforce.point.x(), m_tireforce.point.y());

    ChCoordsys<> contact_frame;
    // Clear the force accumulators and set the application point to the wheel
    // center.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    m_tireforce.point = wheel_state.pos;

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    double dum_cam;

    // Assuming the tire is a disc, check contact with terrain
    switch (collision_type) {
        case ChTire::CollisionType::SINGLE_POINT:
            m_data.in_contact = DiscTerrainCollision(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                     m_data.frame, m_data.depth);
            break;
        case ChTire::CollisionType::FOUR_POINTS:
            m_data.in_contact = DiscTerrainCollision4pt(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                        m_width, m_data.frame, m_data.depth, dum_cam);
            break;
        case ChTire::CollisionType::ENVELOPE:
            m_data.in_contact = DiscTerrainCollisionEnvelope(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                             m_areaDep, m_data.frame, m_data.depth);
            break;
    }
    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector<> vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force (recall, all forces are reduced to the wheel
        // center). If the resulting force is negative, the disc is moving away from
        // the terrain so fast that no contact force is generated.
        // The sign of the velocity term in the damping function is negative since
        // a positive velocity means a decreasing depth, not an increasing depth
        double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
        }

        m_data.normal_force = Fn_mag;
        m_states.R_eff = m_unloaded_radius - m_data.depth;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();  // PAC89 is defined in a modified SAE coordinate system
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_unloaded_radius;
        m_states.cp_long_slip = 0;
        m_states.cp_side_slip = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac89Tire::Advance(double step) {
    // Return now if no contact.  Tire force and moment are already set to 0 in Synchronize().
    if (!m_data.in_contact)
        return;

    if (m_states.vx != 0) {
        m_states.cp_long_slip = -m_states.vsx / m_states.vx;
    } else {
        m_states.cp_long_slip = 0;
    }
    if (m_states.omega != 0) {
        m_states.cp_side_slip = std::atan(m_states.vsy / std::abs(m_states.omega * (m_unloaded_radius - m_data.depth)));
    } else {
        m_states.cp_side_slip = 0;
    }
    // Ensure that cp_lon_slip stays between -1 & 1
    ChClampValue(m_states.cp_long_slip, -1.0, 1.0);

    // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
    ChClampValue(m_states.cp_side_slip, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);

    double mu_scale = m_mu / m_mu0;

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // Express Fz in kN (note that all other forces and moments are in N and Nm).
    // See reference for details on the calculations.
    double Fx = 0;
    double Fy = 0;
    double Fz = m_data.normal_force / 1000;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    // Express alpha and gamma in degrees. Express kappa as percentage.
    // Flip sign of alpha to convert to PAC89 modified SAE coordinates.
    m_gamma = 90.0 - std::acos(m_states.disc_normal.z()) * CH_C_RAD_TO_DEG;
    m_alpha = -m_states.cp_side_slip * CH_C_RAD_TO_DEG;
    m_kappa = m_states.cp_long_slip * 100.0;

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation.
    double gamma = ChClamp(m_gamma, -m_gamma_limit, m_gamma_limit);

    // Longitudinal Force
    {
        double C = m_PacCoeff.B0;
        double D = (m_PacCoeff.B1 * std::pow(Fz, 2) + m_PacCoeff.B2 * Fz);
        double BCD = (m_PacCoeff.B3 * std::pow(Fz, 2) + m_PacCoeff.B4 * Fz) * std::exp(-m_PacCoeff.B5 * Fz);
        double B = BCD / (C * D);
        double Sh = m_PacCoeff.B9 * Fz + m_PacCoeff.B10;
        double Sv = 0.0;
        double X1 = (m_kappa + Sh);
        double E = (m_PacCoeff.B6 * std::pow(Fz, 2) + m_PacCoeff.B7 * Fz + m_PacCoeff.B8);

        Fx = mu_scale * (D * std::sin(C * std::atan(B * X1 - E * (B * X1 - std::atan(B * X1))))) + Sv;
    }

    // Lateral Force
    {
        double C = m_PacCoeff.A0;
        double D = (m_PacCoeff.A1 * std::pow(Fz, 2) + m_PacCoeff.A2 * Fz);
        double BCD =
            m_PacCoeff.A3 * std::sin(std::atan(Fz / m_PacCoeff.A4) * 2.0) * (1.0 - m_PacCoeff.A5 * std::abs(gamma));
        double B = BCD / (C * D);
        double Sh = m_PacCoeff.A9 * Fz + m_PacCoeff.A10 + m_PacCoeff.A8 * gamma;
        double Sv = m_PacCoeff.A11 * Fz * gamma + m_PacCoeff.A12 * Fz + m_PacCoeff.A13;
        double X1 = m_alpha + Sh;
        double E = m_PacCoeff.A6 * Fz + m_PacCoeff.A7;

        // Ensure that X1 stays within +/-90 deg minus a little bit
        ChClampValue(X1, -89.5, 89.5);

        Fy = mu_scale * (D * std::sin(C * std::atan(B * X1 - E * (B * X1 - std::atan(B * X1))))) + Sv;
    }

    // Self-Aligning Torque
    {
        double C = m_PacCoeff.C0;
        double D = (m_PacCoeff.C1 * std::pow(Fz, 2) + m_PacCoeff.C2 * Fz);
        double BCD = (m_PacCoeff.C3 * std::pow(Fz, 2) + m_PacCoeff.C4 * Fz) * (1 - m_PacCoeff.C6 * std::abs(gamma)) *
                     std::exp(-m_PacCoeff.C5 * Fz);
        double B = BCD / (C * D);
        double Sh = m_PacCoeff.C11 * gamma + m_PacCoeff.C12 * Fz + m_PacCoeff.C13;
        double Sv =
            (m_PacCoeff.C14 * std::pow(Fz, 2) + m_PacCoeff.C15 * Fz) * gamma + m_PacCoeff.C16 * Fz + m_PacCoeff.C17;
        double X1 = m_alpha + Sh;
        double E = (m_PacCoeff.C7 * std::pow(Fz, 2) + m_PacCoeff.C8 * Fz + m_PacCoeff.C9) *
                   (1.0 - m_PacCoeff.C10 * std::abs(gamma));

        // Ensure that X1 stays within +/-90 deg minus a little bit
        ChClampValue(X1, -89.5, 89.5);

        Mz = mu_scale * (D * std::sin(C * std::atan(B * X1 - E * (B * X1 - std::atan(B * X1))))) + Sv;
    }

    // Overturning Moment
    {
        double deflection = Fy / m_lateral_stiffness;

        Mx = -(Fz * 1000) * deflection;
        Mz = Mz + Fx * deflection;
    }

    // Rolling Resistance
    {
        double Lrad = (m_unloaded_radius - m_data.depth);
        // Smoothing interval for My
        const double vx_min = 0.125;
        const double vx_max = 0.5;
        // Smoothing factor dependend on m_state.abs_vx, allows soft switching of My
        double myStartUp = ChSineStep(std::abs(m_states.vx), vx_min, 0.0, vx_max, 1.0);
        My = myStartUp * m_rolling_resistance * m_data.normal_force * Lrad * ChSignum(m_states.omega);
    }

    // std::cout << "Fx:" << Fx
    //    << " Fy:" << Fy
    //    << " Fz:" << Fz
    //    << " Mx:" << Mx
    //    << " My:" << My
    //    << " Mz:" << Mz
    //    << std::endl
    //    << " G:" << gamma
    //    << " A:" << alpha
    //    << " K:" << kappa
    //    << " O:" << m_states.omega
    //    << std::endl;

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector<>(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, -My, -Mz);

    // Rotate into global coordinates
    m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
    m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

    // Move the tire forces from the contact patch to the wheel center
    m_tireforce.moment +=
        Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
}

}  // end namespace vehicle
}  // end namespace chrono
