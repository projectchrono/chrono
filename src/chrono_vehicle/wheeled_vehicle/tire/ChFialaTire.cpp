// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
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
// =============================================================================
// STILL UNDERDEVELOPMENT - DO NOT USE
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#define fialaUseSmallAngle 0

namespace chrono {
namespace vehicle {

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChFialaTire::ChFialaTire(const std::string& name) : ChTire(name), m_stepsize(1e-6) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetFialaParams();

    // Initialize contact patach state variables to 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
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
void ChFialaTire::Synchronize(double time, const WheelState& wheel_state, const ChTerrain& terrain) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain);

    ChCoordsys<> contact_frame;
    // Clear the force accumulators and set the application point to the wheel
    // center.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    m_tireforce.point = wheel_state.pos;

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuuming the tire is a disc, check contact with terrain
    m_data.in_contact =
        disc_terrain_contact(terrain, wheel_state.pos, disc_normal, m_unloaded_radius, m_data.frame, m_data.depth);
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
        m_states.vsx = m_data.vel.x() - wheel_state.omega * (m_unloaded_radius - m_data.depth);
        m_states.vsy = m_data.vel.y();
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.cp_long_slip = 0;
        m_states.cp_side_slip = 0;
        m_states.abs_vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFialaTire::Advance(double step) {
    if (m_data.in_contact) {

        // Take as many integration steps as needed to reach the value 'step'
        double t = 0;
        while (t < step) {
            // Ensure we integrate exactly to 'step'
            double h = std::min<>(m_stepsize, step - t);

            // Advance state for longitudinal direction
			// integrate using trapezoidal rule intergration since this equation is linear
			// ref: https://en.wikipedia.org/wiki/Trapezoidal_rule_(differential_equations)
			// cp_long_slip_dot = -1/m_relax_length_x*(Vsx+(abs(Vx)*cp_long_slip))
            m_states.cp_long_slip =
                ((2 * m_relax_length_x - h * m_states.abs_vx) * m_states.cp_long_slip - 2 * h * m_states.vsx) /
                (2 * m_relax_length_x + h * m_states.abs_vx);

#if(fialaUseSmallAngle == 0)
			// integrate using RK2 since this equation is non-linear
			// ref: http://mathworld.wolfram.com/Runge-KuttaMethod.html
			// cp_side_slip = 1/m_relax_length_y*(Vsy-(abs(Vx)*tan(cp_long_slip)))
			double k1 = h / m_relax_length_y*(m_states.vsy - m_states.abs_vx*std::tan(m_states.cp_side_slip));
			double temp = std::max<>(-CH_C_PI_2 + .0001, std::min<>(CH_C_PI_2 - .0001, m_states.cp_side_slip+k1/2));
			double k2 = h / m_relax_length_y*(m_states.vsy - m_states.abs_vx*std::tan(temp));
			m_states.cp_side_slip = m_states.cp_side_slip + k2;
#else
			// Advance state for lateral direction
			// integrate using trapezoidal rule intergration since this equation is linear
			//  after using a small angle approximation for tan(alpha)
			// ref: https://en.wikipedia.org/wiki/Trapezoidal_rule_(differential_equations)
			// cp_long_slip_dot = -1/m_relax_length_x*(Vsx+(abs(Vx)*cp_long_slip))
			m_states.cp_side_slip =
				((2 * m_relax_length_y - h * m_states.abs_vx) * m_states.cp_side_slip + 2 * h * m_states.vsy) /
				(2 * m_relax_length_y + h * m_states.abs_vx);
#endif

            // Ensure that cp_lon_slip stays between -1 & 1
            m_states.cp_long_slip = std::max<>(-1., std::min<>(1., m_states.cp_long_slip));

            // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
            m_states.cp_side_slip = std::max<>(-CH_C_PI_2+.0001, std::min<>(CH_C_PI_2-.0001, m_states.cp_side_slip));

            t += h;
        }

        ////Overwrite with steady-state alpha & kappa for debugging
        // if (m_states.abs_vx != 0) {
        //  m_states.cp_long_slip = -m_states.vsx / m_states.abs_vx;
        //  m_states.cp_side_slip = std::atan2(m_states.vsy , m_states.abs_vx);
        //}
        // else {
        //  m_states.cp_long_slip = 0;
        //  m_states.cp_side_slip = 0;
        //}

        // Now calculate the new force and moment values (normal force and moment has already been accounted for in
        // Synchronize())
        // See reference for more detail on the calculations
        double SsA = std::min<>(1.0,std::sqrt(std::pow(m_states.cp_long_slip, 2) + std::pow(std::tan(m_states.cp_side_slip), 2)));
        double U = m_u_max - (m_u_max - m_u_min) * SsA;
        double S_critical = std::abs(U * m_data.normal_force / (2 * m_c_slip));
        double Alpha_critical = std::atan(3 * U * m_data.normal_force / m_c_alpha);
        double Fx;
        double Fy;
        double My;
        double Mz;

        // Longitudinal Force:
        if (std::abs(m_states.cp_long_slip) < S_critical) {
            Fx = m_c_slip * m_states.cp_long_slip;
        } else {
            double Fx1 = U * m_data.normal_force;
            double Fx2 =
                std::abs(std::pow((U * m_data.normal_force), 2) / (4 * m_states.cp_long_slip * m_c_slip));
            Fx = sgn(m_states.cp_long_slip) * (Fx1 - Fx2);
        }

        // Lateral Force & Aligning Moment (Mz):
        if (std::abs(m_states.cp_side_slip) <= Alpha_critical) {
            double H = 1 - m_c_alpha * std::abs(std::tan(m_states.cp_side_slip)) / (3 * U * m_data.normal_force);

            Fy = -U * m_data.normal_force * (1 - std::pow(H, 3)) * sgn(m_states.cp_side_slip);
            Mz = U * m_data.normal_force * m_width * (1 - H) * std::pow(H, 3) * sgn(m_states.cp_side_slip);
        } else {
            Fy = -U * m_data.normal_force * sgn(m_states.cp_side_slip);
            Mz = 0;
        }

        // Rolling Resistance
        My = -m_rolling_resistance * m_data.normal_force * sgn(m_states.omega);


        // compile the force and moment vectors so that they can be 
		// transformed into the global coordinate system
        m_tireforce.force = ChVector<>(Fx, Fy, m_data.normal_force);
        m_tireforce.moment = ChVector<>(0, My, Mz);

        // Rotate into global coordinates
        m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
        m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

        // Move the tire forces from the contact patch to the wheel center
        m_tireforce.moment +=
            Vcross((m_data.frame.pos + m_data.depth*m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
    }
    // Else do nothing since the "m_tireForce" force and moment values are already 0 (set in Synchronize())
}

}  // end namespace vehicle
}  // end namespace chrono
