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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Template for a tire model based on the Pacejka 2002 Tire Model
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT
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
    : ChForceElementTire(name),
      m_kappa(0),
      m_alpha(0),
      m_gamma(0),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_use_friction_ellipsis(true),
      m_mu(0),
      m_Shf(0),
      m_measured_side(LEFT),
      m_allow_mirroring(false),
      m_use_mode(1) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // standard settings for scaling factors
    m_PacScal.lfz0 = 1.0;
    m_PacScal.lcx = 1.0;
    m_PacScal.lex = 1.0;
    m_PacScal.lkx = 1.0;
    m_PacScal.lhx = 1.0;
    m_PacScal.lmux = 1.0;
    m_PacScal.lvx = 1.0;
    m_PacScal.lxal = 1.0;

    m_PacScal.lcy = 1.0;
    m_PacScal.ley = 1.0;
    m_PacScal.lhy = 1.0;
    m_PacScal.lky = 1.0;
    m_PacScal.lmuy = 1.0;
    m_PacScal.lvy = 1.0;
    m_PacScal.lyka = 1.0;
    m_PacScal.lvyka = 1.0;

    m_PacScal.ltr = 1.0;
    m_PacScal.lgax = 1.0;
    m_PacScal.lgay = 1.0;
    m_PacScal.lgaz = 1.0;
    m_PacScal.lres = 1.0;
    m_PacScal.ls = 1.0;
    m_PacScal.lsgkp = 1.0;
    m_PacScal.lsgal = 1.0;
    m_PacScal.lgyr = 1.0;

    m_PacCoeff.mu0 = 0.8;           // reference friction coefficient
    m_PacCoeff.R0 = 0.0;            // unloaded radius
    m_PacCoeff.width = 0.0;         // tire width = 0.0;
    m_PacCoeff.aspect_ratio = 0.8;  // actually unused
    m_PacCoeff.rim_radius = 0.0;    // actually unused
    m_PacCoeff.rim_width = 0.0;     // actually unused
    m_PacCoeff.FzNomin = 0.0;       // nominla wheel load
    m_PacCoeff.Cz = 0.0;            // vertical tire stiffness
    m_PacCoeff.Kz = 0.0;            // vertical tire damping

    // Longitudinal Coefficients
    m_PacCoeff.pcx1 = 0.0;
    m_PacCoeff.pdx1 = 0.0;
    m_PacCoeff.pdx2 = 0.0;
    m_PacCoeff.pdx3 = 0.0;
    m_PacCoeff.pex1 = 0.0;
    m_PacCoeff.pex2 = 0.0;
    m_PacCoeff.pex3 = 0.0;
    m_PacCoeff.pex4 = 0.0;
    m_PacCoeff.phx1 = 0.0;
    m_PacCoeff.phx2 = 0.0;
    m_PacCoeff.pkx1 = 0.0;
    m_PacCoeff.pkx2 = 0.0;
    m_PacCoeff.pkx3 = 0.0;
    m_PacCoeff.pvx1 = 0.0;
    m_PacCoeff.pvx2 = 0.0;
    m_PacCoeff.rcx1 = 0.0;
    m_PacCoeff.rbx1 = 0.0;
    m_PacCoeff.rbx2 = 0.0;
    m_PacCoeff.rbx3 = 0.0;
    m_PacCoeff.rex1 = 0.0;
    m_PacCoeff.rex2 = 0.0;
    m_PacCoeff.rhx1 = 0.0;
    m_PacCoeff.ptx1 = 0.0;
    m_PacCoeff.ptx2 = 0.0;
    m_PacCoeff.ptx3 = 0.0;
    m_PacCoeff.ptx4 = 0.0;

    // overturning coefficients
    m_PacCoeff.qsx1 = 0.0;
    m_PacCoeff.qsx2 = 0.0;
    m_PacCoeff.qsx3 = 0.0;
    m_PacCoeff.qsx4 = 0.0;
    m_PacCoeff.qsx5 = 0.0;
    m_PacCoeff.qsx6 = 0.0;
    m_PacCoeff.qsx7 = 0.0;
    m_PacCoeff.qsx8 = 0.0;
    m_PacCoeff.qsx9 = 0.0;
    m_PacCoeff.qsx10 = 0.0;
    m_PacCoeff.qsx11 = 0.0;

    // rolling coefficients
    m_PacCoeff.qsy1 = 0.0;
    m_PacCoeff.qsy2 = 0.0;
    m_PacCoeff.qsy3 = 0.0;
    m_PacCoeff.qsy4 = 0.0;
    m_PacCoeff.qsy5 = 0.0;
    m_PacCoeff.qsy6 = 0.0;
    m_PacCoeff.qsy7 = 0.0;
    m_PacCoeff.qsy8 = 0.0;

    // Lateral Coefficients
    m_PacCoeff.pcy1 = 0.0;
    m_PacCoeff.pdy1 = 0.0;
    m_PacCoeff.pdy2 = 0.0;
    m_PacCoeff.pdy3 = 0.0;
    m_PacCoeff.pey1 = 0.0;
    m_PacCoeff.pey2 = 0.0;
    m_PacCoeff.pey3 = 0.0;
    m_PacCoeff.pey4 = 0.0;
    m_PacCoeff.pey5 = 0.0;
    m_PacCoeff.phy1 = 0.0;
    m_PacCoeff.phy2 = 0.0;
    m_PacCoeff.phy3 = 0.0;
    m_PacCoeff.pky1 = 0.0;
    m_PacCoeff.pky2 = 0.0;
    m_PacCoeff.pky3 = 0.0;
    m_PacCoeff.pvy1 = 0.0;
    m_PacCoeff.pvy2 = 0.0;
    m_PacCoeff.pvy3 = 0.0;
    m_PacCoeff.pvy4 = 0.0;
    m_PacCoeff.rby1 = 0.0;
    m_PacCoeff.rby2 = 0.0;
    m_PacCoeff.rby3 = 0.0;
    m_PacCoeff.rby4 = 0.0;
    m_PacCoeff.rcy1 = 0.0;
    m_PacCoeff.rey1 = 0.0;
    m_PacCoeff.rey2 = 0.0;
    m_PacCoeff.rhy1 = 0.0;
    m_PacCoeff.rhy2 = 0.0;
    m_PacCoeff.rvy1 = 0.0;
    m_PacCoeff.rvy2 = 0.0;
    m_PacCoeff.rvy3 = 0.0;
    m_PacCoeff.rvy4 = 0.0;
    m_PacCoeff.rvy5 = 0.0;
    m_PacCoeff.rvy6 = 0.0;
    m_PacCoeff.pty1 = 0.0;
    m_PacCoeff.pty2 = 0.0;

    // alignment coefficients
    m_PacCoeff.qbz1 = 0.0;
    m_PacCoeff.qbz2 = 0.0;
    m_PacCoeff.qbz3 = 0.0;
    m_PacCoeff.qbz4 = 0.0;
    m_PacCoeff.qbz5 = 0.0;
    m_PacCoeff.qbz6 = 0.0;
    m_PacCoeff.qbz9 = 0.0;
    m_PacCoeff.qbz10 = 0.0;
    m_PacCoeff.qcz1 = 0.0;
    m_PacCoeff.qdz1 = 0.0;
    m_PacCoeff.qdz2 = 0.0;
    m_PacCoeff.qdz3 = 0.0;
    m_PacCoeff.qdz4 = 0.0;
    m_PacCoeff.qdz5 = 0.0;
    m_PacCoeff.qdz6 = 0.0;
    m_PacCoeff.qdz7 = 0.0;
    m_PacCoeff.qdz8 = 0.0;
    m_PacCoeff.qdz9 = 0.0;
    m_PacCoeff.qez1 = 0.0;
    m_PacCoeff.qez2 = 0.0;
    m_PacCoeff.qez3 = 0.0;
    m_PacCoeff.qez4 = 0.0;
    m_PacCoeff.qez5 = 0.0;
    m_PacCoeff.qhz1 = 0.0;
    m_PacCoeff.qhz2 = 0.0;
    m_PacCoeff.qhz3 = 0.0;
    m_PacCoeff.qhz4 = 0.0;
    m_PacCoeff.ssz1 = 0.0;
    m_PacCoeff.ssz2 = 0.0;
    m_PacCoeff.ssz3 = 0.0;
    m_PacCoeff.ssz4 = 0.0;
    m_PacCoeff.qtz1 = 0.0;
    m_PacCoeff.mbelt = 0.0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetPac02Params();
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_PacCoeff.R0, m_areaDep);

    // all parameters are known now pepare mirroring
    if (m_allow_mirroring) {
        if (wheel->GetSide() != m_measured_side) {
            // we flip the sign of some parameters
            m_PacCoeff.rhx1 *= -1.0;
            m_PacCoeff.qsx1 *= -1.0;
            m_PacCoeff.pey3 *= -1.0;
            m_PacCoeff.phy1 *= -1.0;
            m_PacCoeff.phy2 *= -1.0;
            m_PacCoeff.pvy1 *= -1.0;
            m_PacCoeff.pvy2 *= -1.0;
            m_PacCoeff.rby3 *= -1.0;
            m_PacCoeff.rvy1 *= -1.0;
            m_PacCoeff.rvy2 *= -1.0;
            m_PacCoeff.qbz4 *= -1.0;
            m_PacCoeff.qdz3 *= -1.0;
            m_PacCoeff.qdz6 *= -1.0;
            m_PacCoeff.qdz7 *= -1.0;
            m_PacCoeff.qez4 *= -1.0;
            m_PacCoeff.qhz1 *= -1.0;
            m_PacCoeff.qhz2 *= -1.0;
            m_PacCoeff.ssz1 *= -1.0;
            if (m_measured_side == LEFT) {
                GetLog() << "Tire is measured as left tire but mounted on the right vehicle side -> mirroring.\n";
            } else {
                GetLog() << "Tire is measured as right tire but mounted on the lleft vehicle side -> mirroring.\n";
            }
        }
    }

    // Initialize contact patch state variables to 0
    m_data.normal_force = 0;
    m_states.R_eff = m_PacCoeff.R0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.vx = 0;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.disc_normal = ChVector<>(0, 0, 0);

    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = m_PacCoeff.R0;
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0);
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
    m_wheel->GetSpindle()->AddVisualShape(m_cyl_shape);
}

void ChPac02Tire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChPac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPac02Tire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();
    CalculateKinematics(time, wheel_state, terrain);

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    double dum_cam;
    float mu;
    switch (m_collision_type) {
        case ChTire::CollisionType::SINGLE_POINT:
            m_data.in_contact =
                DiscTerrainCollision(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0, m_data.frame, m_data.depth, mu);
            break;
        case ChTire::CollisionType::FOUR_POINTS:
            m_data.in_contact = DiscTerrainCollision4pt(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                        m_PacCoeff.width, m_data.frame, m_data.depth, dum_cam, mu);
            break;
        case ChTire::CollisionType::ENVELOPE:
            m_data.in_contact = DiscTerrainCollisionEnvelope(terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                             m_areaDep, m_data.frame, m_data.depth, mu);
            break;
    }
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu = mu;

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
        m_states.R_eff = m_PacCoeff.R0 - m_data.depth;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();  // PAC89 is defined in a modified SAE coordinate system
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_PacCoeff.R0;
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

    // prevent singularity for kappa, when vx == 0
    const double epsilon = 0.1;
    m_states.cp_long_slip = -m_states.vsx / (m_states.vx + epsilon);

    if (m_states.omega != 0) {
        m_states.cp_side_slip = std::atan(m_states.vsy / std::abs(m_states.omega * (m_PacCoeff.R0 - m_data.depth)));
    } else {
        m_states.cp_side_slip = 0;
    }
    // Ensure that cp_lon_slip stays between -1 & 1
    ChClampValue(m_states.cp_long_slip, -1.0, 1.0);

    // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
    ChClampValue(m_states.cp_side_slip, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);

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

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
    // in rad too.
    double gamma = ChClamp(m_gamma, -m_gamma_limit, m_gamma_limit);

    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            Fx = CalcFx(m_kappa, Fz, gamma);
            break;
        case 2:
            // steady state pure lateral slip
            Fy = CalcFy(m_alpha, Fz, gamma);
            break;
        case 3:
            // steady state pure lateral slip uncombined
            Fx = CalcFx(m_kappa, Fz, gamma);
            Fy = CalcFy(m_alpha, Fz, gamma);
            Mx = CalcMx(Fy, Fz, gamma);
            My = CalcMy(Fx, Fz, gamma);
            Mz = CalcMz(m_alpha, Fz, gamma, Fy);
            break;
        case 4:
            // steady state combined slip
            if (m_use_friction_ellipsis) {
                double Fx_u = CalcFx(m_kappa, Fz, gamma);
                double Fy_u = CalcFy(m_alpha, Fz, gamma);
                double as = sin(m_alpha_c);
                double beta = acos(std::abs(m_kappa_c) / sqrt(pow(m_kappa_c, 2) + pow(as, 2)));
                double mux = 1.0 / sqrt(pow(1.0 / m_mu_x_act, 2) + pow(tan(beta) / m_mu_y_max, 2));
                double muy = tan(beta) / sqrt(pow(1.0 / m_mu_x_max, 2) + pow(tan(beta) / m_mu_y_act, 2));
                Fx = mux / m_mu_x_act * Fx_u;
                Fy = muy / m_mu_y_act * Fy_u;
                Mx = CalcMx(Fy, Fz, gamma);
                My = CalcMy(Fx, Fz, gamma);
                Mz = CalcMz(m_alpha, Fz, gamma, Fy);
            } else {
                Fx = CalcFxComb(m_kappa, m_alpha, Fz, gamma);
                Fy = CalcFyComb(m_kappa, m_alpha, Fz, gamma);
                Mx = CalcMx(Fy, Fz, gamma);
                My = CalcMy(Fx, Fz, gamma);
                Mz = CalcMzComb(m_kappa, m_alpha, Fz, gamma, Fx, Fy);
            }
            break;
    }

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

double ChPac02Tire::CalcFx(double kappa, double Fz, double gamma) {
    // calculates the longitudinal force based on a limited parameter set.
    // Pi is not considered
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
    double Mu = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * pow(gamma, 2)) * m_PacScal.lmux;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * dFz * dFz) * m_PacScal.lex;
    if (E > 1.0)
        E = 1.0;
    double BCD = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2 * dFz) * m_PacScal.lkx;  // BCD = Kx
    double B = BCD / (C * D);
    double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
    double Sv = Fz * (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * m_PacScal.lvx * m_PacScal.lmux;
    m_kappa_c = kappa + Sh + Sv / BCD;
    double X1 = B * (kappa + Sh);
    double Fx0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    m_mu_x_act = std::abs((Fx0 - Sv) / Fz);
    m_mu_x_max = std::abs(D / Fz);
    return Fx0;
}

double ChPac02Tire::CalcFy(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
    m_Cy = C;
    double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * pow(gamma, 2)) * m_PacScal.lmuy;
    double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
               (1.0 + m_PacCoeff.pey5 * pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * ChSignum(alpha)) *
               m_PacScal.ley;
    if (E > 1.0)
        E = 1.0;
    double Ky0 = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * sin(2.0 * atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                 m_PacScal.lfz0 * m_PacScal.lky;
    double BCD = Ky0;  // BCD = Ky
    double B = BCD / (C * D);
    m_By = B;
    double Sh = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double Sv = Fz * ((m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy) * m_PacScal.lmuy;
    m_alpha_c = alpha + Sh + Sv / BCD;
    double X1 = ChClamp(B * (alpha + Sh), -CH_C_PI_2 + 0.001,
                        CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double Fy0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    m_Shf = Sh + Sv / BCD;
    m_mu_y_act = std::abs((Fy0 - Sv) / Fz);
    m_mu_y_max = std::abs(D / Fz);
    return Fy0;
}

// Oeverturning Couple
double ChPac02Tire::CalcMx(double Fy, double Fz, double gamma) {
    double Mx = m_PacCoeff.R0 * Fz *
                (m_PacCoeff.qsx1 * m_PacScal.lvx - m_PacCoeff.qsx2 * gamma + m_PacCoeff.qsx3 * Fy / m_PacCoeff.FzNomin +
                 m_PacCoeff.qsx4 * cos(m_PacCoeff.qsx5 * pow(atan(m_PacCoeff.qsx6 * Fz / m_PacCoeff.FzNomin), 2)) *
                     sin(m_PacCoeff.qsx7 * gamma + m_PacCoeff.qsx8 * atan(m_PacCoeff.qsx9 * Fy / m_PacCoeff.FzNomin)) +
                 m_PacCoeff.qsx10 * atan(m_PacCoeff.qsx11 * Fz / m_PacCoeff.FzNomin) * gamma);
    return Mx;
}

// Rolling Resistance
double ChPac02Tire::CalcMy(double Fx, double Fz, double gamma) {
    double v0 = sqrt(9.81 * m_PacCoeff.R0);
    double vstar = std::abs(m_states.vx / v0);
    double My = ChSineStep(std::abs(m_states.vx), 0.5, 0, 1.0, 1.0) * ChSignum(m_states.vx) * Fz * m_PacCoeff.R0 *
                (m_PacCoeff.qsy1 + m_PacCoeff.qsy2 * Fx / m_PacCoeff.FzNomin + m_PacCoeff.qsy3 * vstar +
                 m_PacCoeff.qsy4 * pow(vstar, 4) +
                 (m_PacCoeff.qsy5 + m_PacCoeff.qsy6 * Fz / m_PacCoeff.FzNomin) * pow(gamma, 2)) *
                pow(Fz / m_PacCoeff.FzNomin, m_PacCoeff.qsy7) * m_PacScal.lmuy;
    return My;
}

double ChPac02Tire::CalcTrail(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.qcz1;
    double gamma_z = gamma * m_PacScal.lgaz;
    double Sh = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma_z;
    double alpha_t = alpha + Sh;
    double B = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
               (1.0 + m_PacCoeff.qbz4 * gamma_z + m_PacCoeff.qbz5 * std::abs(gamma_z)) * m_PacScal.lky / m_PacScal.lmuy;
    double D = Fz * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) *
               (1.0 + m_PacCoeff.qdz3 * gamma_z + m_PacCoeff.qdz4 * pow(gamma_z, 2)) * m_PacCoeff.R0 / Fz0s *
               m_PacScal.ltr;
    double E = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
               (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_z) * atan(B * C * alpha_t) / CH_C_PI_2);
    double X1 = B * alpha_t;
    return D * cos(C * atan(B * X1 - E * (B * X1 - atan(B * X1)))) * cos(alpha);
}

double ChPac02Tire::CalcMres(double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double alpha_r = alpha + m_Shf;
    double gamma_z = gamma * m_PacScal.lgaz;
    double C = 1.0;
    double B = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * m_By * m_Cy);
    double D = Fz *
               ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * gamma_z) *
               m_PacCoeff.R0 * m_PacScal.lmuy;
    return D * cos(C * atan(B * alpha_r)) * cos(alpha);
}

double ChPac02Tire::CalcFxComb(double kappa, double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
    double Mux = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * pow(gamma, 2)) * m_PacScal.lmux;
    double D = Mux * Fz * m_mu / m_PacCoeff.mu0;
    double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * dFz * dFz) * m_PacScal.lex;
    if (E > 1.0)
        E = 1.0;
    double BCD = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2 * dFz) * m_PacScal.lkx;  // BCD = Kx
    double B = BCD / (C * D);
    double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
    double Sv = Fz * (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * m_PacScal.lvx * m_PacScal.lmux;
    double X1 = B * (kappa + Sh);
    double Fx0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    double Shxa = m_PacCoeff.rhx1;
    double alpha_s = tan(alpha) * ChSignum(m_data.vel.x()) + Shxa;
    double Bxa =
        (m_PacCoeff.rbx1 + m_PacCoeff.rbx3 * pow(sin(gamma), 2)) * cos(atan(m_PacCoeff.rbx2 * kappa)) * m_PacScal.lxal;
    double Cxa = m_PacCoeff.rcx1;
    double Exa = m_PacCoeff.rex1 + m_PacCoeff.rex2 * dFz;
    double Gxa = cos(Cxa * atan(Bxa * alpha_s) - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s))) /
                 cos(Cxa * atan(Bxa * Shxa) - Exa * (Bxa * Shxa - atan(Bxa * Shxa)));
    return Fx0 * Gxa;
}

double ChPac02Tire::CalcFyComb(double kappa, double alpha, double Fz, double gamma) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
    double Muy = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * pow(gamma, 2)) * m_PacScal.lmuy;
    double D = Muy * Fz * m_mu / m_PacCoeff.mu0;
    double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
               (1.0 + m_PacCoeff.pey5 * pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * ChSignum(alpha)) *
               m_PacScal.ley;
    if (E > 1.0)
        E = 1.0;
    double Ky0 = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * sin(2.0 * atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                 m_PacScal.lfz0 * m_PacScal.lky;
    double BCD = Ky0;  // BCD = Ky
    double B = BCD / (C * D);
    double Sh = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double Sv = Fz * ((m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy) * m_PacScal.lmuy;
    double X1 = ChClamp(B * (alpha + Sh), -CH_C_PI_2 + 0.001,
                        CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
    double Fy0 = D * sin(C * atan(X1 - E * (X1 - atan(X1)))) + Sv;
    double Shyk = m_PacCoeff.rhy1 + m_PacCoeff.rhy2 * dFz;
    m_Shf = Sh + Sv / BCD;
    double kappa_s = kappa + Shyk;
    double Byk = m_PacCoeff.rby1 * cos(atan(m_PacCoeff.rby2 * (tan(alpha) - m_PacCoeff.rby3)));
    double Cyk = m_PacCoeff.rcy1;
    double Dvyk = Muy * Fz * (m_PacCoeff.rvy1 + m_PacCoeff.rvy2 * dFz + m_PacCoeff.rvy3 * gamma) *
                  cos(atan(m_PacCoeff.rvy4 * tan(alpha)));
    double Svyk = Dvyk * sin(m_PacCoeff.rvy5 * atan(m_PacCoeff.rvy6 * kappa));
    double Gyk = cos(Cyk * atan(Byk * kappa_s)) / cos(Cyk * atan(Byk * Shyk));
    return Fy0 * Gyk + Svyk;
}

double ChPac02Tire::CalcMz(double alpha, double Fz, double gamma, double Fy) {
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double C = m_PacCoeff.qcz1;
    double gamma_z = gamma * m_PacScal.lgaz;
    double Sh = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma_z;
    double alpha_t = alpha + Sh;
    double B = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
               (1.0 + m_PacCoeff.qbz4 * gamma_z + m_PacCoeff.qbz5 * std::abs(gamma_z)) * m_PacScal.lky / m_PacScal.lmuy;
    double D = Fz * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) *
               (1.0 + m_PacCoeff.qdz3 * gamma_z + m_PacCoeff.qdz4 * pow(gamma_z, 2)) * m_PacCoeff.R0 / Fz0s *
               m_PacScal.ltr;
    double E = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
               (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_z) * atan(B * C * alpha_t) / CH_C_PI_2);
    double X1 = B * alpha_t;
    double t = D * cos(C * atan(B * X1 - E * (B * X1 - atan(B * X1)))) * cos(alpha);
    double Mz = -t * Fy + CalcMres(m_alpha, Fz, gamma);
    return Mz;
}

double ChPac02Tire::CalcMzComb(double kappa, double alpha, double Fz, double gamma, double Fx, double Fy) {
    double Mz = 0.0;
    double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
    double dFz = (Fz - Fz0s) / Fz0s;
    double Sht = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * sin(gamma);
    double alpha_s = alpha + (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
    double alpha_t = alpha_s + Sht;
    double Ct = m_PacCoeff.qcz1;
    double gamma_s = sin(gamma);
    // pneumatic trail
    double Dt0 = Fz * (m_PacCoeff.R0 / Fz0s) * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) * m_PacScal.ltr *
                 ChSignum(m_data.vel.x());
    double Dt = Dt0 * (1.0 + m_PacCoeff.qdz3 * std::abs(gamma_s) + m_PacCoeff.qdz4 * pow(gamma_s, 2));
    double Bt = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * pow(dFz, 2)) *
                (1.0 + m_PacCoeff.qbz5 * std::abs(gamma_s) + m_PacCoeff.qbz6 * pow(gamma_s, 2)) * m_PacScal.lky /
                m_PacScal.lmuy;
    double Et = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * pow(dFz, 2)) *
                ((1.0 + m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_s) * atan(Bt * Ct * alpha_t / CH_C_PI_2));
    double Kxk = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2) * m_PacScal.lkx;
    double Kya = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * sin(2.0 * atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                 m_PacScal.lfz0 * m_PacScal.lky;
    double t = Dt * (Ct * atan(Bt * alpha_t - Et * (Bt * alpha_t - atan(Bt * alpha_t))));
    // residual moment
    double Shf = 0.0;  // todo!!
    double alpha_r = alpha + Shf;
    double alpha_req = sqrt(pow(alpha_r, 2) + pow(Kxk, 2) * pow(kappa, 2) / pow(Kya, 2)) * ChSignum(alpha_r);
    double gamma_z = gamma * m_PacScal.lgaz;
    double Cr = 1.0;
    double Br = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * m_By * m_Cy);
    double Dr = Fz *
                ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                 (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * gamma_z) *
                m_PacCoeff.R0 * m_PacScal.lmuy;
    double Mr = Dr * cos(Cr * atan(Br * alpha_req));
    // moment caused by longitudinal force
    double s = m_PacCoeff.R0 *
               (m_PacCoeff.ssz1 + m_PacCoeff.ssz2 * (Fy / Fz0s) + (m_PacCoeff.ssz3 + m_PacCoeff.ssz4 * dFz) * gamma_s) *
               m_PacScal.ls;
    Mz = -t * Fy + Mr + s * Fx;
    return Mz;
}

}  // end namespace vehicle
}  // namespace chrono
