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
// Template for a tire model based on the MSC ADAMS PAC2002 Tire Model
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

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPac02Tire::ChPac02Tire(const std::string& name)
    : ChTire(name),
      m_kappa(0),
      m_alpha(0),
      m_gamma(0),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_mu(0),
      m_mu0(0.8),
      m_PacScal({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}),
      m_PacCoeff({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetPac02Params();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = m_unloaded_radius;
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0);
    m_wheel->GetSpindle()->AddAsset(m_cyl_shape);

    m_texture = chrono_types::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->GetSpindle()->AddAsset(m_texture);
}

void ChPac02Tire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChPac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    {
        auto it = std::find(assets.begin(), assets.end(), m_cyl_shape);
        if (it != assets.end())
            assets.erase(it);
    }
    {
        auto it = std::find(assets.begin(), assets.end(), m_texture);
        if (it != assets.end())
            assets.erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();
    CalculateKinematics(time, wheel_state, terrain);

    // Get mu at wheel location
    m_mu = terrain.GetCoefficientFriction(wheel_state.pos.x(), wheel_state.pos.y());

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    double dum_cam;

    // Assuming the tire is a disc, check contact with terrain
    switch (m_collision_type) {
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
void ChPac02Tire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.point = m_wheel->GetPos();
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
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
    double Fz = m_data.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    // Express alpha and gamma in rad. Express kappa as ratio.
    m_gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
    m_alpha = m_states.cp_side_slip;
    m_kappa = m_states.cp_long_slip;

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is in
    // rad too.
    double gamma = ChClamp(m_gamma, -m_gamma_limit, m_gamma_limit);

    // Longitudinal Force
    double Fx0 = mu_scale * CalcFx1(m_kappa, Fz);

    // Lateral Force
    double Fy0 = mu_scale * CalcFy1(m_alpha, Fz);
    double as = sin(m_alpha_c);
    double beta = acos(std::abs(m_kappa_c) / sqrt(pow(m_kappa_c, 2.0) + pow(as, 2.0)));
    double den = sqrt(pow(1.0 / m_mu_x_act, 2.0) + pow(tan(beta) / m_mu_y_act, 2.0));
    double mux = 1.0 / den;
    double muy = tan(beta) / den;

    // Combined Forces (Friction Ellipsis Method)
    Fx = mux / m_mu_x_act * Fx0;
    Fy = muy / m_mu_y_act * Fy0;

    // Self-Aligning Torque
    Mz = 0.0;

    // Overturning Moment
    {
        double deflection = Fy / m_lateral_stiffness;

        Mx = -(Fz)*deflection;
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

    // GetLog() << "Fx:" << Fx
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
    //    << "\n";

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

double ChPac02Tire::CalcFx1(double kappa, double Fz) {
    // calculates the longitudinal force based on a limited parameter set.
    // gamma, Pi are not considered
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcx1 * m_PacScal.lcx1;
    double Mu = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * m_PacScal.lmux;
    double D = Mu * Fz * m_PacScal.xsi1;
    double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * dFz * dFz) * m_PacScal.lex;
    double BCD = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2) * m_PacScal.lkx;  // BCD = Kx
    double B = BCD / (C * D);
    double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
    double Sv = Fz * (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * m_PacScal.lvx * m_PacScal.lmux * m_PacScal.xsi1;
    m_kappa_c = kappa + Sh + Sv / BCD;
    double X1 = B * (kappa + Sh);
    double Fx0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    m_mu_x_act = (Fx0 - Sv) / Fz;
    m_mu_x_max = D / Fz;
    return Fx0;
}

double ChPac02Tire::CalcFy1(double alpha, double Fz) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
    double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * m_PacScal.lmuy;
    double D = Mu * Fz * m_PacScal.xsi2;
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) * m_PacScal.ley;
    double Ky0 = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * sin(2.0 * atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                 m_PacScal.lfz0 * m_PacScal.lky;
    double BCD = Ky0 * m_PacScal.xsi3;  // BCD = Ky
    double B = BCD / (C * D);
    double Sh = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy + m_PacScal.xsi4 - 1.0;
    double Sv = Fz * ((m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy) * m_PacScal.lmuy * m_PacScal.xsi2;
    m_alpha_c = alpha - Sh + Sv / BCD;
    double X1 = B * (alpha + Sh);
    X1 =
        ChClamp(X1, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double Fy0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    m_mu_y_act = (Fy0 - Sv) / Fz;
    m_mu_y_max = D / Fz;
    return Fy0;
}

}  // end namespace vehicle
}  // end namespace chrono
