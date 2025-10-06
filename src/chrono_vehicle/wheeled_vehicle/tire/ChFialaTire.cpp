// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Template for a tire model based on the MSC ADAMS Transient Fiala Tire Model
//
// Ref: Adams/Tire help - Adams 2014.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC10645&cat=2014_ADAMS_DOCS&actp=LIST
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor, Rainer Gericke
// =============================================================================
//
// Fiala tire model with Coulomb friction based stand-still algorithm
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/core/ChDataPath.h"
#include "chrono/functions/ChFunctionSineStep.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

namespace chrono {
namespace vehicle {

ChFialaTire::ChFialaTire(const std::string& name)
    : ChForceElementTire(name),
      m_mu(0.8),
      m_mu_0(0.8),
      m_c_slip(0),
      m_c_alpha(0) {
    m_tireforce.force = ChVector3d(0, 0, 0);
    m_tireforce.point = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);
}

// -----------------------------------------------------------------------------

void ChFialaTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetFialaParams();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0
    m_states.kappa = 0;
    m_states.alpha = 0;
}

void ChFialaTire::Synchronize(double time, const ChTerrain& terrain) {
    m_time = time;
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
        }

        m_data.normal_force = Fn_mag;
        m_states.abs_vx = std::abs(m_data.vel.x());
        m_states.abs_vt = std::abs(wheel_state.omega * (m_unloaded_radius - m_data.depth));
        m_states.vsx = m_data.vel.x() - wheel_state.omega * (m_unloaded_radius - m_data.depth);
        m_states.vsy = m_data.vel.y();
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.kappa = 0;
        m_states.alpha = 0;
        m_states.abs_vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.abs_vt = 0;
        m_states.disc_normal = ChVector3d(0, 0, 0);
    }
}

void ChFialaTire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector3d(0, 0, 0);
    m_tireforce.moment = ChVector3d(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // smoothing interval for My
    const double vx_min = 0.125;
    const double vx_max = 0.5;

    if (m_states.abs_vx != 0) {
        m_states.kappa = -m_states.vsx / m_states.abs_vx;
        m_states.alpha = std::atan2(m_states.vsy, m_states.abs_vx);
    } else {
        m_states.kappa = 0;
        m_states.alpha = 0;
    }

    // Now calculate the new force and moment values.
    // Normal force and moment have already been accounted for in Synchronize().
    // See reference for more detail on the calculations
    double Fx = 0;
    double Fy = 0;
    double My = 0;
    double Mz = 0;

    FialaPatchForces(Fx, Fy, Mz, m_states.kappa, m_states.alpha, m_data.normal_force);

    // Smoothing factor dependend on m_state.abs_vx, allows soft switching of My
    double myStartUp = ChFunctionSineStep::Eval(m_states.abs_vx, vx_min, 0.0, vx_max, 1.0);
    // Rolling Resistance
    My = -myStartUp * m_rolling_resistance * m_data.normal_force * ChSignum(m_states.omega);

    // compile the force and moment vectors so that they can be
    // transformed into the global coordinate system
    m_tireforce.force = ChVector3d(Fx, Fy, m_data.normal_force);
    m_tireforce.moment = ChVector3d(0, My, Mz);
}

// -----------------------------------------------------------------------------

void ChFialaTire::FialaPatchForces(double& fx, double& fy, double& mz, double kappa, double alpha, double fz) {
    double SsA = std::min<>(1.0, std::sqrt(std::pow(kappa, 2) + std::pow(std::tan(alpha), 2)));
    double U = m_u_max - (m_u_max - m_u_min) * SsA;
    double S_critical = std::abs(U * fz / (2 * m_c_slip));
    double Alpha_critical = std::atan(3 * U * fz / m_c_alpha);

    // modify U due to local friction
    U *= m_mu / m_mu_0;
    // Longitudinal Force:
    if (std::abs(kappa) < S_critical) {
        fx = m_c_slip * kappa;
    } else {
        double Fx1 = U * fz;
        double Fx2 = std::abs(std::pow((U * fz), 2) / (4 * kappa * m_c_slip));
        fx = ChSignum(kappa) * (Fx1 - Fx2);
    }

    // Lateral Force & Aligning Moment (Mz):
    if (std::abs(alpha) <= Alpha_critical) {
        double H = 1.0 - m_c_alpha * std::abs(std::tan(alpha)) / (3.0 * U * fz);

        fy = -U * fz * (1.0 - std::pow(H, 3)) * ChSignum(alpha);
        mz = U * fz * m_width * (1.0 - H) * std::pow(H, 3) * ChSignum(alpha);
    } else {
        fy = -U * fz * ChSignum(alpha);
        mz = 0;
    }
}

void ChFialaTire::WritePlots(const std::string& plFileName, const std::string& plTireFormat) {
    if (m_c_slip == 0.0 || m_c_alpha == 0.0) {
        std::cerr << "Fiala Tire Object is not yet initialized! No Plots available." << std::endl;
        return;
    }
    const double Fz_nom = 1.0;

    std::vector<double> kappa, Fx1;
    std::vector<double> alpha, Fy1;
    const double step = 0.005;
    for (int i = 0; i < 201; i++) {
        kappa.push_back(step * double(i));
        alpha.push_back(step * double(i));
    }

    for (int i = 0; i < kappa.size(); i++) {
        double adum = 0.0;
        double fxt, fyt, mzt;
        FialaPatchForces(fxt, fyt, mzt, kappa[i], adum, Fz_nom);
        Fx1.push_back(fxt);
    }

    for (int i = 0; i < alpha.size(); i++) {
        double kdum = 0.0;
        double fxt, fyt, mzt;
        FialaPatchForces(fxt, fyt, mzt, kdum, alpha[i], Fz_nom);
        Fy1.push_back(fyt);
    }

    std::string titleName = std::string("Fiala Tire ") + this->GetName() + ": ";
    titleName += plTireFormat;
    std::ofstream plot(plFileName);

    plot << "Mu     = " << m_mu << std::endl;
    plot << "Mu_0   = " << m_mu_0 << std::endl;
    plot << "RADIUS = " << m_unloaded_radius << std::endl;
    plot << "WIDTH  = " << m_width << std::endl;
    plot << "CSLIP  = " << m_c_slip << std::endl;
    plot << "CALPHA = " << m_c_alpha << std::endl;
    plot << "UMIN   = " << m_u_min << std::endl;
    plot << "UMAX   = " << m_u_max << std::endl;
    plot << "$Fx << EOD" << std::endl;
    for (int i = 0; i < kappa.size(); i++) {
        plot << kappa[i] << "\t" << Fx1[i] << std::endl;
    }
    plot << "EOD" << std::endl;

    plot << "$FyMz << EOD" << std::endl;
    for (int i = 0; i < alpha.size(); i++) {
        plot << alpha[i] << "\t" << Fy1[i] << std::endl;
    }
    plot << "EOD" << std::endl;
    plot << "set title '" << titleName << "'" << std::endl;
    plot << "set xlabel 'Longitudinal Slip Kappa []'" << std::endl;
    plot << "set ylabel 'Longitudinal Friction Coefficient Mux []'" << std::endl;
    plot << "set xrange [0:1]" << std::endl;
    plot << "plot $Fx u 1:2 with lines" << std::endl;

    plot << "pause -1" << std::endl;

    plot << "set title '" << titleName << "'" << std::endl;
    plot << "set xlabel 'Slip Angle Alpha [rad]'" << std::endl;
    plot << "set ylabel 'Lateral Friction Coefficient Muy []'" << std::endl;
    plot << "set xrange [0:1]" << std::endl;
    plot << "plot $FyMz u 1:2 with lines" << std::endl;

    plot << "pause -1" << std::endl;

    plot.close();
}

}  // end namespace vehicle
}  // end namespace chrono
