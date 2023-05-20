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

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTire.h"

namespace chrono {
namespace vehicle {

ChMFTire::ChMFTire(const std::string& name)
    : ChForceElementTire(name),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_use_friction_ellipsis(true),
      m_mu_road(0),
      m_Shf(0),
      m_measured_side(LEFT),
      m_allow_mirroring(false),
      m_use_mode(0) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

ChMFTire::ChMFTire(const std::string& name, std::string& tirFileName)
    : ChForceElementTire(name),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      m_use_friction_ellipsis(true),
      m_mu_road(0),
      m_Shf(0),
      m_measured_side(LEFT),
      m_allow_mirroring(false),
      m_use_mode(0) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    // set tire parameters defined in TIR (Tiem Orbit Format) file
    SetMFParamsByFile(tirFileName);
}

// -----------------------------------------------------------------------------

void ChMFTire::SetMFParamsByFile(std::string& tirFileName) {
    std::string dataFile = vehicle::GetDataFile(tirFileName);
    
    FILE *fp = fopen(dataFile.c_str(), "r+");
    if(fp == NULL) {
        GetLog() << "TIR File not found <" << dataFile << ">!\n";
        exit(1);
    }
    fclose(fp);
}

void ChMFTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetMFParams();
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_par.UNLOADED_RADIUS, m_areaDep);

    // all parameters are known now pepare mirroring
    if (m_allow_mirroring) {
        if (wheel->GetSide() != m_measured_side) {
            // we flip the sign of some parameters to compensate asymmetry
            m_par.RHX1 *= -1.0;
            m_par.QSX1 *= -1.0;
            m_par.PEY3 *= -1.0;
            m_par.PHY1 *= -1.0;
            m_par.PHY2 *= -1.0;
            m_par.PVY1 *= -1.0;
            m_par.PVY2 *= -1.0;
            m_par.RBY3 *= -1.0;
            m_par.RVY1 *= -1.0;
            m_par.RVY2 *= -1.0;
            m_par.QBZ4 *= -1.0;
            m_par.QDZ3 *= -1.0;
            m_par.QDZ6 *= -1.0;
            m_par.QDZ7 *= -1.0;
            m_par.QEZ4 *= -1.0;
            m_par.QHZ1 *= -1.0;
            m_par.QHZ2 *= -1.0;
            m_par.SSZ1 *= -1.0;
            if (m_measured_side == LEFT) {
                GetLog() << "Tire is measured as left tire but mounted on the right vehicle side -> mirroring.\n";
            } else {
                GetLog() << "Tire is measured as right tire but mounted on the lleft vehicle side -> mirroring.\n";
            }
        }
    }

    // Initialize contact patch state variables to 0
    m_data.normal_force = 0;
    m_states.R_eff = m_par.UNLOADED_RADIUS;
    m_states.kappa = 0;
    m_states.alpha = 0;
    m_states.gamma = 0;
    m_states.vx = 0;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.disc_normal = ChVector<>(0, 0, 0);
}

void ChMFTire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu;
    m_data.in_contact =
        DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_par.UNLOADED_RADIUS,
                             m_par.WIDTH, m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu_road = mu;

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

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
        m_states.gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
        // R_eff is a Rill estimation, not Pacejka. Advantage: it works well with speed = zero.
        m_states.R_eff = (2.0 * m_par.UNLOADED_RADIUS + (m_par.UNLOADED_RADIUS - m_data.depth)) / 3.0;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = -m_data.vel.y();
        // prevent singularity for kappa, when vx == 0
        const double epsilon = 0.1;
        m_states.kappa = -m_states.vsx / (m_states.vx + epsilon);
        m_states.alpha = std::atan2(m_states.vsy, m_states.vx + epsilon);
        m_states.gamma = CH_C_PI_2 - std::acos(m_states.disc_normal.z());
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;
        // Ensure that kappa stays between -1 & 1
        ChClampValue(m_states.kappa, -1.0, 1.0);
        // Ensure that alpha stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
        ChClampValue(m_states.alpha, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);
        // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
        // in rad too.
        ChClampValue(m_states.gamma, -m_gamma_limit, m_gamma_limit);
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_par.UNLOADED_RADIUS;
        m_states.kappa = 0;
        m_states.alpha = 0;
        m_states.gamma = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChMFTire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // Calculate the new force and moment values (normal force and moment have already been accounted for in
    // Synchronize()).
    // See reference for details on the calculations.
    double Fx = 0;
    double Fy = 0;
    double Fz = m_data.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            break;
        case 2:
            // steady state pure lateral slip
            break;
        case 3:
            // steady state pure lateral slip uncombined
            break;
        case 4:
            // steady state combined slip
            break;
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch.
    m_tireforce.force = ChVector<>(Fx, -Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, -My, -Mz);
}

// -----------------------------------------------------------------------------

void ChMFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape =
        ChVehicleGeometry::AddVisualizationCylinder(m_wheel->GetSpindle(),                                        //
                                                    ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0),  //
                                                    ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0),  //
                                                    GetRadius());
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void ChMFTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChMFTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------

}  // end namespace vehicle
}  // namespace chrono
