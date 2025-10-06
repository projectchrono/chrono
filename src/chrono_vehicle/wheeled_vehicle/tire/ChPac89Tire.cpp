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

#include "chrono/core/ChDataPath.h"
#include "chrono/functions/ChFunctionSineStep.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"

namespace chrono {
namespace vehicle {

ChPac89Tire::ChPac89Tire(const std::string& name)
    : ChForceElementTire(name), m_gamma(0), m_gamma_limit(3.0 * CH_DEG_TO_RAD), m_mu(0), m_mu0(0.8) {
    m_tireforce.force = ChVector3d(0, 0, 0);
    m_tireforce.point = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);
}

// -----------------------------------------------------------------------------
void ChPac89Tire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetPac89Params();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.R_eff = m_unloaded_radius;
}

void ChPac89Tire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector3d disc_normal = A.GetAxisY();

    // Assuming the tire is a disc, check contact with terrain
    float mu;
    m_data.in_contact = DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                             m_width, m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu = mu;

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector3d vel = wheel_state.lin_vel;
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
        m_states.brx = 0;
        m_states.bry = 0;
        m_states.disc_normal = ChVector3d(0, 0, 0);
    }
}

void ChPac89Tire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);

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
    ChClampValue(m_states.cp_side_slip, -CH_PI_2 + 0.001, CH_PI_2 - 0.001);

    double mu_scale = m_mu / m_mu0;

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // Express Fz in kN (note that all other forces and moments are in N and Nm).
    // See reference for details on the calculations.
    double Fx = 0, Fx0 = 0;
    double Fy = 0, Fy0 = 0;
    double Fz = m_data.normal_force / 1000;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    CombinedCoulombForces(Fx0, Fy0, m_data.normal_force, mu_scale);

    // Calculate alpha and gamma (in degrees) and kappa (percentage).
    // Clamp |gamma| to specified value (limit due to tire testing, avoids erratic extrapolation)
    // Flip sign of alpha to convert to PAC89 modified SAE coordinates.
    m_gamma = ChClamp(CH_PI_2 - std::acos(m_states.disc_normal.z()), -m_gamma_limit, m_gamma_limit);
    double gamma = m_gamma * CH_RAD_TO_DEG;
    double alpha = -m_states.cp_side_slip * CH_RAD_TO_DEG;
    double kappa = m_states.cp_long_slip * 100.0;

    // Longitudinal Force
    {
        double C = m_PacCoeff.B0;
        double D = (m_PacCoeff.B1 * std::pow(Fz, 2) + m_PacCoeff.B2 * Fz);
        double BCD = (m_PacCoeff.B3 * std::pow(Fz, 2) + m_PacCoeff.B4 * Fz) * std::exp(-m_PacCoeff.B5 * Fz);
        double B = BCD / (C * D);
        double Sh = m_PacCoeff.B9 * Fz + m_PacCoeff.B10;
        double Sv = 0.0;
        double X1 = (kappa + Sh);
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
        double X1 = alpha + Sh;
        double E = m_PacCoeff.A6 * Fz + m_PacCoeff.A7;

        // Ensure that X1 stays within +/-90 deg minus a little bit
        ChClampValue(X1, -89.5, 89.5);

        Fy = mu_scale * (D * std::sin(C * std::atan(B * X1 - E * (B * X1 - std::atan(B * X1))))) + Sv;
    }

    // Blend forces
    constexpr double frblend_begin = 1.0;
    constexpr double frblend_end = 3.0;
    double frblend = ChFunctionSineStep::Eval(m_data.vel.x(), frblend_begin, 0.0, frblend_end, 1.0);
    Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
    Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

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
        double X1 = alpha + Sh;
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
        double myStartUp = ChFunctionSineStep::Eval(std::abs(m_states.vx), vx_min, 0.0, vx_max, 1.0);
        My = myStartUp * m_rolling_resistance * m_data.normal_force * Lrad * ChSignum(m_states.omega);
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector3d(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector3d(Mx, -My, -Mz);
}

void ChPac89Tire::CombinedCoulombForces(double& fx, double& fy, double fz, double muscale) {
    ChVector2d F;
    /*
     The Dahl Friction Model elastic tread blocks represented by a single bristle. At tire stand still it acts
     like a spring which enables holding of a vehicle on a slope without creeping (hopefully). Damping terms
     have been added to calm down the oscillations of the pure spring.

     The time step h must be actually the same as for the vehicle system!

     This model is experimental and needs some testing.

     With bristle deformation z, Coulomb force fc, sliding velocity v and stiffness sigma we have this
     differential equation:
         dz/dt = v - sigma0*z*abs(v)/fc

     When z is known, the friction force F can be calculated to:
        F = sigma0 * z

     For practical use some damping is needed, that leads to:
        F = sigma0 * z + sigma1 * dz/dt

     Longitudinal and lateral forces are calculated separately and then combined. For stand still a friction
     circle is used.
     */
    double fc = fz * muscale;
    double h = this->m_stepsize;
    // Longitudinal Friction Force
    double brx_dot = m_states.vsx - m_PacCoeff.sigma0 * m_states.brx * fabs(m_states.vsx) / fc;  // dz/dt
    F.x() = -(m_PacCoeff.sigma0 * m_states.brx + m_PacCoeff.sigma1 * brx_dot);
    // Lateral Friction Force
    double bry_dot = m_states.vsy - m_PacCoeff.sigma0 * m_states.bry * fabs(m_states.vsy) / fc;  // dz/dt
    F.y() = -(m_PacCoeff.sigma0 * m_states.bry + m_PacCoeff.sigma1 * bry_dot);
    // Calculate the new ODE states (implicit Euler)
    m_states.brx = (fc * m_states.brx + fc * h * m_states.vsx) / (fc + h * m_PacCoeff.sigma0 * fabs(m_states.vsx));
    m_states.bry = (fc * m_states.bry + fc * h * m_states.vsy) / (fc + h * m_PacCoeff.sigma0 * fabs(m_states.vsy));

    // combine forces (friction circle)
    if (F.Length() > fz * muscale) {
        F.Normalize();
        F *= fz * muscale;
    }
    fx = F.x();
    fy = F.y();
}

}  // end namespace vehicle
}  // end namespace chrono
