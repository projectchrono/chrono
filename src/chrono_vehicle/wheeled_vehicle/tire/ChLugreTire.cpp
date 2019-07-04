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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for a tire model based on LuGre friction
//
// Ref: C.Canudas de Wit, P.Tsiotras, E.Velenis, M.Basset and
// G.Gissinger.Dynamics friction models for longitudinal road / tire
// interaction.Vehicle System Dynamics.Oct 14, 2002.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChLugreTire::ChLugreTire(const std::string& name) : ChTire(name) {
    m_tireForce.force = ChVector<>(0, 0, 0);
    m_tireForce.point = ChVector<>(0, 0, 0);
    m_tireForce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    m_data.resize(GetNumDiscs());
    m_state.resize(GetNumDiscs());

    SetLugreParams();

    // Initialize disc states
    for (int id = 0; id < GetNumDiscs(); id++) {
        m_state[id].z0 = 0;
        m_state[id].z1 = 0;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double discWidth = 0.04;
    double disc_radius = GetRadius();
    const double* disc_locs = GetDiscLocations();

    m_cyl_shapes.resize(GetNumDiscs());
    for (int id = 0; id < GetNumDiscs(); id++) {
        m_cyl_shapes[id] = std::make_shared<ChCylinderShape>();
        m_cyl_shapes[id]->GetCylinderGeometry().rad = disc_radius;
        m_cyl_shapes[id]->GetCylinderGeometry().p1 = ChVector<>(0, disc_locs[id] + discWidth / 2, 0);
        m_cyl_shapes[id]->GetCylinderGeometry().p2 = ChVector<>(0, disc_locs[id] - discWidth / 2, 0);
        m_wheel->AddAsset(m_cyl_shapes[id]);
    }

    m_texture = std::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChLugreTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChLugreTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    for (int id = 0; id < m_cyl_shapes.size(); id++) {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_cyl_shapes[id]);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_texture);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
    m_cyl_shapes.clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChLugreTire::GetWidth() const {
    return std::abs(GetDiscLocations()[0] - GetDiscLocations()[GetNumDiscs() - 1]);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Synchronize(double time,
                              const WheelState& wheel_state,
                              const ChTerrain& terrain,
                              CollisionType collision_type) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain);

    double disc_radius = GetRadius();
    const double* disc_locs = GetDiscLocations();

    // Clear the force accumulators and set the application point to the wheel
    // center.
    m_tireForce.force = ChVector<>(0, 0, 0);
    m_tireForce.moment = ChVector<>(0, 0, 0);
    m_tireForce.point = wheel_state.pos;

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Loop over all discs, check contact with terrain, accumulate normal tire
    // forces, and cache data that only depends on wheel state.
    double depth;

    for (int id = 0; id < GetNumDiscs(); id++) {
        // Calculate center of disk (expressed in global frame)
        ChVector<> disc_center = wheel_state.pos + disc_locs[id] * disc_normal;

        // Check contact with terrain and calculate contact points.
        m_data[id].in_contact =
            DiscTerrainCollision(terrain, disc_center, disc_normal, disc_radius, m_data[id].frame, depth);
        if (!m_data[id].in_contact)
            continue;

        // Relative velocity at contact point (expressed in the global frame and in
        // the contact frame)
        ChVector<> vel = wheel_state.lin_vel + Vcross(wheel_state.ang_vel, m_data[id].frame.pos - wheel_state.pos);
        m_data[id].vel = m_data[id].frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force and add to accumulators (recall, all forces
        // are reduced to the wheel center). If the resulting force is negative, the
        // disc is moving away from the terrain so fast that no contact force is
        // generated.
        double Fn_mag = (GetNormalStiffness() * depth - GetNormalDamping() * m_data[id].vel.z()) / GetNumDiscs();

        if (Fn_mag < 0)
            Fn_mag = 0;

        ChVector<> Fn = Fn_mag * m_data[id].frame.rot.GetZaxis();

        m_data[id].normal_force = Fn_mag;

        m_tireForce.force += Fn;
        m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Fn);

        // ODE coefficients for longitudinal direction: z' = a + b * z
        {
            double v = std::abs(m_data[id].vel.x());
            double g = m_Fc[0] + (m_Fs[0] - m_Fc[0]) * exp(-sqrt(v / m_vs[0]));
            m_data[id].ode_coef_a[0] = v;
            m_data[id].ode_coef_b[0] = -m_sigma0[0] * v / g;
        }

        // ODE coefficients for lateral direction: z' = a + b * z
        {
            double v = std::abs(m_data[id].vel.y());
            double g = m_Fc[1] + (m_Fs[1] - m_Fc[1]) * exp(-sqrt(v / m_vs[1]));
            m_data[id].ode_coef_a[1] = v;
            m_data[id].ode_coef_b[1] = -m_sigma0[1] * v / g;
        }

    }  // end loop over discs
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Advance(double step) {
    for (int id = 0; id < GetNumDiscs(); id++) {
        // Nothing to do if this disc is not in contact
        if (!m_data[id].in_contact)
            continue;

        // Advance disc states, for longitudinal and lateral directions, using the
        // trapezoidal integration scheme, written in the form:
        //         z_{n+1} = alpha * z_{n} + beta
        double denom;
        double alpha;
        double beta;

        // Start from the current cached values
        double z0 = m_state[id].z0;
        double z1 = m_state[id].z1;

        // Take as many integration steps as needed to reach the value 'step'
        double t = 0;
        while (t < step) {
            // Ensure we integrate exactly to 'step'
            double h = std::min<>(m_stepsize, step - t);

            // Advance state for longitudinal direction
            denom = (2 - m_data[id].ode_coef_b[0] * h);
            alpha = (2 + m_data[id].ode_coef_b[0] * h) / denom;
            beta = 2 * m_data[id].ode_coef_a[0] * h / denom;
            z0 = alpha * z0 + beta;

            // Advance state for lateral direction
            denom = (2 - m_data[id].ode_coef_b[1] * h);
            alpha = (2 + m_data[id].ode_coef_b[1] * h) / denom;
            beta = 2 * m_data[id].ode_coef_a[1] * h / denom;
            z1 = alpha * z1 + beta;

            t += h;
        }

        // Cache the states for use at subsequent calls.
        m_state[id].z0 = z0;
        m_state[id].z1 = z1;

        // Magnitude of normal contact force for this disc
        double Fn_mag = m_data[id].normal_force;

        // Evaluate friction force and add to accumulators for tire force
        {
            // Longitudinal direction
            double zd0 = m_data[id].ode_coef_a[0] + m_data[id].ode_coef_b[0] * z0;

            double v = m_data[id].vel.x();
            double Ft_mag = Fn_mag * (m_sigma0[0] * z0 + m_sigma1[0] * zd0 + m_sigma2[0] * std::abs(v));
            ChVector<> dir = (v > 0) ? m_data[id].frame.rot.GetXaxis() : -m_data[id].frame.rot.GetXaxis();
            ChVector<> Ft = -Ft_mag * dir;

            m_tireForce.force += Ft;
            m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Ft);
        }

        {
            // Lateral direction
            double zd1 = m_data[id].ode_coef_a[1] + m_data[id].ode_coef_b[1] * z1;

            double v = m_data[id].vel.y();
            double Ft_mag = Fn_mag * (m_sigma0[1] * z1 + m_sigma1[1] * zd1 + m_sigma2[1] * std::abs(v));
            ChVector<> dir = (v > 0) ? m_data[id].frame.rot.GetYaxis() : -m_data[id].frame.rot.GetYaxis();
            ChVector<> Ft = -Ft_mag * dir;

            m_tireForce.force += Ft;
            m_tireForce.moment += Vcross(m_data[id].frame.pos - m_tireForce.point, Ft);
        }

    }  // end loop over discs
}

}  // end namespace vehicle
}  // end namespace chrono
