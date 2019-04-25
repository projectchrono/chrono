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
// Template for a tire model based on the MSC ADAMS Transient Fiala Tire Model
//
// Ref: Adams/Tire help - Adams 2014.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC10645&cat=2014_ADAMS_DOCS&actp=LIST
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Fiala tire model.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChFialaTire::ChFialaTire(const std::string& name)
    : ChTire(name), m_dynamic_mode(false), m_mu(0.8), m_mu_0(0.8), m_time_trans(0.2), m_c_slip(0), m_c_alpha(0) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetFialaParams();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0
    m_states.kappa = 0;
    m_states.alpha = 0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = std::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetVisualizationWidth() / 2, 0);
    m_wheel->AddAsset(m_cyl_shape);

    m_texture = std::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChFialaTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChFialaTire::AddVisualizationAssets.
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
void ChFialaTire::Synchronize(double time,
                              const WheelState& wheel_state,
                              const ChTerrain& terrain,
                              CollisionType collision_type) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain);

    m_time = time;

    // Clear the force accumulators and set the application point to the wheel center.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    m_tireforce.point = wheel_state.pos;

    m_mu = terrain.GetCoefficientFriction(m_tireforce.point.x(), m_tireforce.point.y());

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    double dum_cam = 0;
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
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Advance(double step) {
    if (m_data.in_contact) {
        ////Overwrite with steady-state alpha & kappa for debugging
        // if (m_states.abs_vx != 0) {
        //  m_states.kappa_l = -m_states.vsx / m_states.abs_vx;
        //  m_states.alpha_l = std::atan2(m_states.vsy , m_states.abs_vx);
        //}
        // else {
        //  m_states.kappa_l = 0;
        //  m_states.alpha_l = 0;
        //}

        const double vnum = 0.01;

        // smoothing interval for My
        const double vx_min = 0.125;
        const double vx_max = 0.5;

        // limits for time lags
        const double tau_min = 1.0e-4;
        const double tau_max = 0.25;

        // lag times for relaxation
        double tau_k;
        double tau_a;

        if (m_states.abs_vx != 0) {
            m_states.kappa = -m_states.vsx / m_states.abs_vx;
            m_states.alpha = std::atan2(m_states.vsy, m_states.abs_vx);
        } else {
            m_states.kappa = 0;
            m_states.alpha = 0;
        }
        // Relaxation time varies with rotational tire speed. Stand still or very low speed generates
        // unrealistic lags and causes bad  oscillations. Tau == 0 is not allowed in later calculations
        tau_k = ChClamp(m_relax_length_x / (m_states.abs_vt + vnum), tau_min, tau_max);
        tau_a = ChClamp(m_relax_length_y / (m_states.abs_vt + vnum), tau_min, tau_max);

        // Now calculate the new force and moment values.
        // Normal force and moment have already been accounted for in Synchronize().
        // See reference for more detail on the calculations
        double Fx = 0;
        double Fy = 0;
        double My = 0;
        double Mz = 0;

        FialaPatchForces(Fx, Fy, Mz, m_states.kappa, m_states.alpha, m_data.normal_force);

        // Smoothing factor dependend on m_state.abs_vx, allows soft switching of My
        double myStartUp = ChSineStep(m_states.abs_vx, vx_min, 0.0, vx_max, 1.0);
        // Rolling Resistance
        My = -myStartUp * m_rolling_resistance * m_data.normal_force * ChSignum(m_states.omega);

        if (m_dynamic_mode && (m_relax_length_x > 0.0) && (m_relax_length_y > 0.0)) {
            // Integration of the ODEs
            double t = 0;
            while (t < step) {
                // Ensure we integrate exactly to 'step'
                double h = std::min<>(m_stepsize, step - t);
                double gain_k = 1.0 / tau_k;
                double gain_a = 1.0 / tau_a;
                m_states.Fx_l += h / (1.0 - h * (-gain_k)) * gain_k * (Fx - m_states.Fx_l);
                m_states.Fy_l += h / (1.0 - h * (-gain_a)) * gain_a * (Fy - m_states.Fy_l);
                t += h;
            }
        } else {
            m_states.Fx_l = Fx;
            m_states.Fy_l = Fy;
        }

        // Smooth starting transients
        double tr_fact = ChSineStep(m_time, 0, 0, m_time_trans, 1.0);
        m_states.Fx_l *= tr_fact;
        m_states.Fy_l *= tr_fact;

        // compile the force and moment vectors so that they can be
        // transformed into the global coordinate system
        m_tireforce.force = ChVector<>(m_states.Fx_l, m_states.Fy_l, m_data.normal_force);
        m_tireforce.moment = ChVector<>(0, My, Mz);

        // Rotate into global coordinates
        m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
        m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

        // Move the tire forces from the contact patch to the wheel center
        m_tireforce.moment += Vcross(
            (m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
    }
    // Else do nothing since the "m_tireForce" force and moment values are already 0 (set in Synchronize())
}

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
        std::cout << "Fiala Tire Object is not yet initialized! No Plots available." << std::endl;
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
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Longitudinal Slip Kappa []'" << std::endl;
    plot << "set ylabel 'Longitudinal Friction Coefficient Mux []'" << std::endl;
    plot << "set xrange [0:1]" << std::endl;
    plot << "plot $Fx u 1:2 with lines" << std::endl;

    plot << "pause -1" << std::endl;

    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Slip Angle Alpha [rad]'" << std::endl;
    plot << "set ylabel 'Lateral Friction Coefficient Muy []'" << std::endl;
    plot << "set xrange [0:1]" << std::endl;
    plot << "plot $FyMz u 1:2 with lines" << std::endl;

    plot << "pause -1" << std::endl;

    plot.close();
}

}  // end namespace vehicle
}  // end namespace chrono
