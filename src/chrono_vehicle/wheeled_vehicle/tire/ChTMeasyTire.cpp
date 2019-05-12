// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2017 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Template for the "Tire Model Made Easy"
//
// Ref: Georg Rill, "Road Vehicle Dynamics - Fundamentals and Modelling",
//          @2012 CRC Press, ISBN 978-1-4398-3898-3
//      Georg Rill, "An Engineer's Guess On Tyre Model Parameter Mmade Possible With TMeasy",
//          https://hps.hs-regensburg.de/rig39165/Rill_Tyre_Coll_2015.pdf
//
// No parking slip calculations.
//
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace chrono {
namespace vehicle {

const double kN2N = 1000.0;
const double N2kN = 0.001;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTMeasyTire::ChTMeasyTire(const std::string& name)
    : ChTire(name),
      m_vnum(0.01),
      m_gamma(0),
      m_gamma_limit(5),
      m_begin_start_transition(0.1),
      m_end_start_transition(0.25),
      m_use_startup_transition(false) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    m_TMeasyCoeff.pn = 0.0;
    m_TMeasyCoeff.mu_0 = 0.8;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTMeasyTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetTMeasyParams();

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0;
    m_states.sx = 0;
    m_states.sy = 0;
    m_states.vta = m_vnum;
    m_states.xe = 0;
    m_states.ye = 0;
    m_states.Fx_dyn = 0;
    m_states.Fy_dyn = 0;
    m_states.Mb_dyn = 0;
    m_states.R_eff = m_unloaded_radius;
    m_consider_relaxation = true;
    m_use_Reff_fallback_calculation = false;
    m_integration_method = 2;

    if (!m_use_Reff_fallback_calculation) {
        // calculate critical values
        m_fz_rdynco_crit = (m_TMeasyCoeff.pn * (m_TMeasyCoeff.rdynco_p2n - 2.0 * m_TMeasyCoeff.rdynco_pn + 1.0)) /
                           (2.0 * (m_TMeasyCoeff.rdynco_p2n - m_TMeasyCoeff.rdynco_pn));
        m_rdynco_crit = InterpL(m_fz_rdynco_crit, m_TMeasyCoeff.rdynco_pn, m_TMeasyCoeff.rdynco_p2n);
    }
}

// -----------------------------------------------------------------------------
void ChTMeasyTire::AddVisualizationAssets(VisualizationType vis) {
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

void ChTMeasyTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChTMeasyTire::AddVisualizationAssets.
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
void ChTMeasyTire::Synchronize(double time,
                               const WheelState& wheel_state,
                               const ChTerrain& terrain,
                               CollisionType collision_type) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain);

    m_time = time;

    ChCoordsys<> contact_frame;
    // Clear the force accumulators and set the application point to the wheel
    // center.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
    m_tireforce.point = wheel_state.pos;
    m_mu = terrain.GetCoefficientFriction(m_tireforce.point.x(), m_tireforce.point.y());
    // Ensure that m_mu stays realistic and the formulae don't degenerate
    ChClampValue(m_mu, 0.1, 1.0);

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    switch (collision_type) {
        case CollisionType::SINGLE_POINT:
            m_data.in_contact = DiscTerrainCollision(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                     m_data.frame, m_data.depth);
            m_gamma = GetCamberAngle();
            break;
        case CollisionType::FOUR_POINTS:
            m_data.in_contact = DiscTerrainCollision4pt(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                        m_width, m_data.frame, m_data.depth, m_gamma);
            break;
        case CollisionType::ENVELOPE:
            m_data.in_contact = DiscTerrainCollisionEnvelope(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                             m_areaDep, m_data.frame, m_data.depth);
            m_gamma = GetCamberAngle();
            break;
    }
    UpdateVerticalStiffness();
    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector<> vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force. If the resulting force is negative, the disc
        // is moving away from the terrain so fast that no contact force is generated.
        // The sign of the velocity term in the damping function is negative since a
        // positive velocity means a decreasing depth, not an increasing depth.
        double Fn_mag = m_TMeasyCoeff.cz * m_data.depth - m_TMeasyCoeff.dz * m_data.vel.z();

        // Skip Force and moment calculations when the normal force = 0
        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;
        }

        m_data.normal_force = Fn_mag;
        double r_stat = m_unloaded_radius - m_data.depth;
        m_states.omega = wheel_state.omega;
        if (m_use_Reff_fallback_calculation) {
            m_states.R_eff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
        } else {
            if (Fn_mag <= m_fz_rdynco_crit) {
                m_rdynco = InterpL(Fn_mag, m_TMeasyCoeff.rdynco_pn, m_TMeasyCoeff.rdynco_p2n);
                m_states.R_eff = m_rdynco * m_unloaded_radius + (1.0 - m_rdynco) * (r_stat);
            } else {
                m_rdynco = m_rdynco_crit;
                m_states.R_eff = m_rdynco * m_unloaded_radius + (1.0 - m_rdynco) * (r_stat);
            }
        }
        m_states.vta = m_states.R_eff * std::abs(m_states.omega) + m_vnum;
        m_states.vsx = m_data.vel.x() - m_states.omega * m_states.R_eff;
        m_states.vsy = m_data.vel.y();
        m_states.sx = -m_states.vsx / m_states.vta;
        m_states.sy = -m_states.vsy / m_states.vta;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_unloaded_radius;
        m_states.sx = 0;
        m_states.sy = 0;
        m_states.vta = m_vnum;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.Fx_dyn = 0;
        m_states.Fy_dyn = 0;
        m_states.Mb_dyn = 0;
        m_states.xe = 0;
        m_states.ye = 0;
        m_states.Fx = 0;
        m_states.Fy = 0;
        m_states.Mb = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTMeasyTire::Advance(double step) {
    // Return now if no contact.  Tire force and moment are already set to 0 in Synchronize().
    if (!m_data.in_contact)
        return;

    double sc;              // combined slip
    double calpha, salpha;  // cos(alpha) rsp. sin(alpha), alpha = slip angle
    double muscale;         // factor for considering local friction

    muscale = m_mu / m_TMeasyCoeff.mu_0;

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation.
    double gamma = ChClamp(GetCamberAngle(), -m_gamma_limit * CH_C_DEG_TO_RAD, m_gamma_limit * CH_C_DEG_TO_RAD);

    // Limit the effect of Fz on handling forces and torques to avoid nonsensical extrapolation of the curve
    // coefficients
    // m_data.normal_force is nevertheless still taken as the applied vertical tire force
    double Fz = std::min(m_data.normal_force, m_TMeasyCoeff.pn_max);
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    // Calculate Fz dependend Curve Parameters
    double dfx0 = InterpQ(Fz, m_TMeasyCoeff.dfx0_pn, m_TMeasyCoeff.dfx0_p2n);
    double dfy0 = InterpQ(Fz, m_TMeasyCoeff.dfy0_pn, m_TMeasyCoeff.dfy0_p2n);

    double fxm = muscale * InterpQ(Fz, m_TMeasyCoeff.fxm_pn, m_TMeasyCoeff.fxm_p2n);
    double fym = muscale * InterpQ(Fz, m_TMeasyCoeff.fym_pn, m_TMeasyCoeff.fym_p2n);

    double sxm = muscale * InterpL(Fz, m_TMeasyCoeff.sxm_pn, m_TMeasyCoeff.sxm_p2n);
    double sym = muscale * InterpL(Fz, m_TMeasyCoeff.sym_pn, m_TMeasyCoeff.sym_p2n);

    double fxs = muscale * InterpQ(Fz, m_TMeasyCoeff.fxs_pn, m_TMeasyCoeff.fxs_p2n);
    double fys = muscale * InterpQ(Fz, m_TMeasyCoeff.fys_pn, m_TMeasyCoeff.fys_p2n);

    double sxs = muscale * InterpL(Fz, m_TMeasyCoeff.sxs_pn, m_TMeasyCoeff.sxs_p2n);
    double sys = muscale * InterpL(Fz, m_TMeasyCoeff.sys_pn, m_TMeasyCoeff.sys_p2n);

    // slip normalizing factors
    double hsxn = sxm / (sxm + sym) + (fxm / dfx0) / (fxm / dfx0 + fym / dfy0);
    double hsyn = sym / (sxm + sym) + (fym / dfy0) / (fxm / dfx0 + fym / dfy0);

    double sxn = m_states.sx / hsxn;
    double syn = m_states.sy / hsyn;

    sc = hypot(sxn, syn);

    if (sc > 0) {
        calpha = sxn / sc;
        salpha = syn / sc;
    } else {
        calpha = sqrt(2.0) / 2.0;
        salpha = sqrt(2.0) / 2.0;
    }

    double nto0 = InterpL(Fz, m_TMeasyCoeff.nto0_pn, m_TMeasyCoeff.nto0_p2n);
    double synto0 = muscale * InterpL(Fz, m_TMeasyCoeff.synto0_pn, m_TMeasyCoeff.synto0_p2n);
    double syntoE = muscale * InterpL(Fz, m_TMeasyCoeff.syntoE_pn, m_TMeasyCoeff.syntoE_p2n);

    // Calculate resultant Curve Parameters
    double df0 = hypot(dfx0 * calpha * hsxn, dfy0 * salpha * hsyn);
    double fm = hypot(fxm * calpha, fym * salpha);
    double sm = hypot(sxm * calpha / hsxn, sym * salpha / hsyn);
    double fs = hypot(fxs * calpha, fys * salpha);
    double ss = hypot(sxs * calpha / hsxn, sys * salpha / hsyn);
    double f = 0.0;
    double fos = 0.0;

    // consider camber effects
    // Calculate length of tire contact patch
    double plen = 2.0 * sqrt(m_unloaded_radius * m_data.depth);

    // tire bore radius  (estimated from length l and width b of contact patch)
    double rb = 2.0 / 3.0 * 0.5 * ((plen / 2.0) + (m_width / 2.0));

    // bore slip due to camber
    double sb = -rb * m_states.omega * sin(gamma) / m_states.vta;

    // generalzed slip
    double sg = hypot(sc, sb);

    tmxy_combined(f, fos, sg, df0, sm, fm, ss, fs);
    if (sg > 0.0) {
        m_states.Fx = f * m_states.sx / sg;
        m_states.Fy = f * m_states.sy / sg;
    } else {
        m_states.Fx = 0.0;
        m_states.Fy = 0.0;
    }
    // Calculate dimensionless lever arm
    double levN = tmy_tireoff(m_states.sy, nto0, synto0, syntoE);

    // Bore Torque
    if (sg > 0.0) {
        m_states.Mb = rb * f * sb / sg;
    } else {
        m_states.Mb = 0.0;
    }

    //   camber slip and force
    double sy_c = -0.5 * plen * m_states.omega * sin(gamma) / m_states.vta;
    double fy_c = fos / 3.0 * sy_c;

    m_states.Fy += fy_c;

    // Overturning Torque
    {
        double cg = std::pow(m_width, 2.0) * m_TMeasyCoeff.cz / 12.0;
        Mx = -cg * gamma;
    }

    // Rolling Resistance, Ramp Like Signum inhibits 'switching' of My
    {
        // smoothing interval for My
        const double vx_min = 0.125;
        const double vx_max = 0.5;

        double Lrad = (m_unloaded_radius - m_data.depth);
        m_rolling_resistance = InterpL(Fz, m_TMeasyCoeff.rrcoeff_pn, m_TMeasyCoeff.rrcoeff_p2n);
        My = -ChSineStep(m_states.vta, vx_min, 0.0, vx_max, 1.0) * m_rolling_resistance * m_data.normal_force * Lrad *
             ChSignum(m_states.omega);
    }

    double Ms = 0.0;

    double startup = 1;
    if (m_use_startup_transition) {
        startup = ChSineStep(m_time, m_begin_start_transition, 0.0, m_end_start_transition, 1.0);
    }

    if (m_consider_relaxation) {
        double vtxs = m_states.vta * hsxn;
        double vtys = m_states.vta * hsyn;
        double relax = 0.66 * CH_C_2PI * m_states.R_eff;

        //-------------------
        // Self Alignment Torque
        Ms = -plen * levN * m_states.Fy_dyn;
        Mz = Ms + m_states.Mb_dyn;

        // Take as many integration steps as needed to reach the value 'step'
        double t = 0;
        while (t < step) {
            // Ensure we integrate exactly to 'step'
            double h = std::min<>(m_stepsize, step - t);
            // limit delay time of bore torque Mb to realistic values
            double tau = ChClamp(relax / m_states.vta, 1.0e-4, 0.25);
            double gain = 1.0 / tau;

            switch (m_integration_method) {
                case 1: {
                    // explicit Euler, may be unstable
                    // 1. oder tire dynamics
                    m_states.xe = m_states.xe +
                                  h * (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                                      (vtxs * m_TMeasyCoeff.dx + fos);
                    m_states.ye = m_states.ye +
                                  h * (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                                      (vtys * m_TMeasyCoeff.dy + fos);
                    // 0. order tire dynamics
                    m_states.Mb_dyn = m_states.Mb_dyn + h * (m_states.Mb - m_states.Mb_dyn) * m_states.vta / relax;
                    break;
                }
                case 2: {
                    // semi-implicit Euler, absolutely stable
                    // 1. oder tire dynamics
                    double dFx = -vtxs * m_TMeasyCoeff.cx / (vtxs * m_TMeasyCoeff.dx + fos);
                    m_states.xe = m_states.xe +
                                  h / (1.0 - h * dFx) * (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                                      (vtxs * m_TMeasyCoeff.dx + fos);
                    double dFy = -vtys * m_TMeasyCoeff.cy / (vtys * m_TMeasyCoeff.dy + fos);
                    m_states.ye = m_states.ye +
                                  h / (1.0 - h * dFy) * (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                                      (vtys * m_TMeasyCoeff.dy + fos);
                    // 0. order tire dynamics
                    double dMb = -gain;
                    m_states.Mb_dyn = m_states.Mb_dyn + h / (1.0 - h * dMb) * (m_states.Mb - m_states.Mb_dyn) * gain;
                    break;
                }
            }
            t += h;
        }

        m_states.Fx_dyn = m_TMeasyCoeff.dx * (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                              (vtxs * m_TMeasyCoeff.dx + fos) +
                          m_TMeasyCoeff.cx * m_states.xe;
        m_states.Fy_dyn = m_TMeasyCoeff.dy * (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                              (vtys * m_TMeasyCoeff.dy + fos) +
                          m_TMeasyCoeff.cy * m_states.ye;
        // Calculate result of alignment torque and bore torque
        // Compile the force and moment vectors so that they can be
        // transformed into the global coordinate system.
        m_tireforce.force = ChVector<>(startup * m_states.Fx_dyn, startup * m_states.Fy_dyn, m_data.normal_force);
        m_tireforce.moment = startup * ChVector<>(Mx, My, Mz);
        // Info data (not used in the algorithm)
        m_tau_x = m_TMeasyCoeff.dx / m_TMeasyCoeff.cx + fos / (vtxs * m_TMeasyCoeff.cx);
        m_tau_y = m_TMeasyCoeff.dy / m_TMeasyCoeff.cy + fos / (vtys * m_TMeasyCoeff.cy);
        m_relaxation_lenght_x = m_states.R_eff * std::abs(m_states.omega) * m_tau_x;
        m_relaxation_lenght_y = m_states.R_eff * std::abs(m_states.omega) * m_tau_y;
        // could be changed to user input, if needed
        m_relaxation_lenght_phi = relax;
    } else {
        // Self Alignment Torque
        m_tau_x = m_tau_y = 0;
        m_relaxation_lenght_x = m_relaxation_lenght_y = 0;
        Ms = -plen * levN * m_states.Fy;
        // Calculate result of alignment torque and bore torque
        Mz = Ms + m_states.Mb;
        // Compile the force and moment vectors so that they can be
        // transformed into the global coordinate system.
        m_tireforce.force = ChVector<>(startup * m_states.Fx, startup * m_states.Fy, m_data.normal_force);
        m_tireforce.moment = startup * ChVector<>(Mx, My, Mz);
    }
    // Rotate into global coordinates
    m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
    m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

    // Move the tire forces from the contact patch to the wheel center
    m_tireforce.moment +=
        Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
}

void ChTMeasyTire::tmxy_combined(double& f,
                                 double& fos,
                                 double s,
                                 double df0,
                                 double sm,
                                 double fm,
                                 double ss,
                                 double fs) {
    double df0loc = 0.0;
    if (sm > 0.0) {
        df0loc = std::max(2.0 * fm / sm, df0);
    }

    if (s > 0.0 && df0loc > 0.0) {  // normal operating conditions
        if (s > ss) {               // full sliding
            f = fs;
            fos = f / s;
        } else {
            if (s < sm) {  // adhesion
                double p = df0loc * sm / fm - 2.0;
                double sn = s / sm;
                double dn = 1.0 + (sn + p) * sn;
                f = df0loc * sm * sn / dn;
                fos = df0loc / dn;
            } else {
                double a = std::pow(fm / sm, 2.0) / (df0loc * sm);  // parameter from 2. deriv. of f @ s=sm
                double sstar = sm + (fm - fs) / (a * (ss - sm));    // connecting point
                if (sstar <= ss) {                                  // 2 parabolas
                    if (s <= sstar) {
                        // 1. parabola sm < s < sstar
                        f = fm - a * (s - sm) * (s - sm);
                    } else {
                        // 2. parabola sstar < s < ss
                        double b = a * (sstar - sm) / (ss - sstar);
                        f = fs + b * (ss - s) * (ss - s);
                    }
                } else {
                    // cubic fallback function
                    double sn = (s - sm) / (ss - sm);
                    f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn);
                }
                fos = f / s;
            }
        }
    } else {
        f = 0.0;
        fos = 0.0;
    }

    // scale up from kN to N
    f *= kN2N;
    fos *= kN2N;
}

double ChTMeasyTire::tmy_tireoff(double sy, double nto0, double synto0, double syntoE) {
    double nto = 0.0;

    double sy_a = std::abs(sy);  // absolute slip value

    double syntoE_loc = std::max(syntoE, synto0);  // ensure appropriate data

    if (sy_a >= syntoE_loc) {
        nto = 0.0;  //   very high slip values --> pure sliding
    } else {
        double wf = synto0 / syntoE_loc;  // weighting function for 2 approximations
        if (sy_a <= synto0) {             // low and moderate slip values
            double sy_n = sy_a / synto0;
            double nto1 = nto0 * (1.0 - sy_n);
            double nto2 = nto0 * (1.0 - (3.0 - 2.0 * sy_n) * sy_n * sy_n);
            nto = (1.0 - wf) * nto1 + wf * nto2;
        } else {  //  high slip values
            double sy_n = (syntoE_loc - sy_a) / (syntoE_loc - synto0);
            nto = -nto0 * (1.0 - wf) * (sy_a - synto0) / synto0 * sy_n * sy_n;
        }
    }

    return nto;
}

void ChTMeasyTire::SetVerticalStiffness(double Cz1, double Cz2) {
    double cz1 = N2kN * Cz1;
    double cz2 = N2kN * Cz2;

    if (m_TMeasyCoeff.pn <= 0.0) {
        std::cout << "Fatal error in TMeasyTire: nominal tire load has not been set." << std::endl;
        exit(99);
    }

    m_a1 = sqrt(2.0 * pow(cz1, 2) - pow(cz2, 2));
    m_a2 = (pow(cz2, 2) - pow(cz1, 2)) / (4.0 * N2kN * m_TMeasyCoeff.pn);
}

void ChTMeasyTire::SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc) {
    // for numerical reasons we scale down the force values form N to kN
    // polynomial regression of the type y = a*t + b*t^2
    // at least 3 table entries, no identical pairs, no 0,0 pair
    double sat2 = 0.0, sat3 = 0.0, sbt3, sbt4 = 0.0, syt = 0.0, syt2 = 0.0;

    m_tire_test_defl.resize(defl.size());
    m_tire_test_frc.resize(defl.size());

    for (int i = 0; i < defl.size(); i++) {
        m_tire_test_defl[i] = defl[i];  // needed for plotting
        m_tire_test_frc[i] = frc[i];
        sat2 += defl[i] * defl[i];
        sat3 += defl[i] * defl[i] * defl[i];
        sbt4 += defl[i] * defl[i] * defl[i] * defl[i];
        syt += N2kN * frc[i] * defl[i];
        syt2 += N2kN * frc[i] * defl[i] * defl[i];
    }
    sbt3 = sat3;

    double A[2][2], Aa[2][2], Ab[2][2];
    A[0][0] = sat2;
    A[0][1] = sbt3;
    A[1][0] = sat3;
    A[1][1] = sbt4;

    Aa[0][0] = syt;
    Aa[0][1] = sbt3;
    Aa[1][0] = syt2;
    Aa[1][1] = sbt4;

    Ab[0][0] = sat2;
    Ab[0][1] = syt;
    Ab[1][0] = sat3;
    Ab[1][1] = syt2;

    double dA = A[0][0] * A[1][1] - A[1][0] * A[0][1];

    if (dA == 0.0) {
        std::cout << "There is a problem with the Force/Deflection Table of the Tire!" << std::endl;
        return;
    }

    double a = (Aa[0][0] * Aa[1][1] - Aa[1][0] * Aa[0][1]) / dA;
    double b = (Ab[0][0] * Ab[1][1] - Ab[1][0] * Ab[0][1]) / dA;

    // set stiffness ploynomial coefficients
    m_a1 = a;
    m_a2 = b;
}

void ChTMeasyTire::UpdateVerticalStiffness() {
    m_TMeasyCoeff.cz = kN2N * (m_a1 + 2.0 * m_a2 * m_data.depth);
}

double ChTMeasyTire::GetTireMaxLoad(unsigned int li) {
    double Weight_per_Tire[] = {
        45,    46.5,  47.5,   48.7,   50,     51.5,   53,     54.5,   56,     58,     60,     61.5,   63,     65,
        67,    69,    71,     73,     75,     77.5,   80.0,   82.5,   85.0,   87.5,   90.0,   92.5,   95.0,   97.5,
        100.0, 103,   106,    109,    112,    115,    118,    121,    125,    128,    132,    136,    140,    145,
        150,   155,   160,    165,    170,    175,    180,    185,    190,    195,    200,    206,    212,    218,
        224,   230,   236,    243,    250,    257,    265,    272,    280,    290,    300,    307,    315,    325,
        335,   345,   355,    365,    375,    387,    400,    412,    425,    437,    450,    462,    475,    487,
        500,   515,   530,    545,    560,    580,    600,    615,    630,    650,    670,    690,    710,    730,
        750,   775,   800,    825,    850,    875,    900,    925,    950,    975,    1000,   1030,   1060,   1090,
        1120,  1150,  1180,   1215,   1250,   1285,   1320,   1360,   1400,   1450,   1500,   1550,   1600,   1650,
        1700,  1750,  1800,   1850,   1900,   1950,   2000,   2060,   2120,   2180,   2240,   2300,   2360,   2430,
        2500,  2575,  2650,   2725,   2800,   2900,   3000,   3075,   3150,   3250,   3350,   3450,   3550,   3650,
        3750,  3875,  4000,   4125,   4250,   4375,   4500,   4625,   4750,   4875,   5000,   5150,   5300,   5450,
        5600,  5850,  6000,   6150,   6300,   6500,   6700,   6900,   7100,   7300,   7500,   7750,   8000,   8250,
        8500,  8750,  9000,   9250,   9500,   9750,   10000,  10300,  10600,  10900,  11200,  11500,  11800,  12150,
        12500, 12850, 13200,  13600,  14000,  14500,  15000,  15550,  16000,  16500,  17000,  17500,  18000,  18500,
        19000, 19500, 20000,  20600,  21200,  21800,  22400,  23000,  23600,  24300,  25000,  25750,  26500,  27250,
        28000, 29000, 30000,  30750,  31500,  32500,  33500,  34500,  35500,  36500,  37500,  38750,  40000,  41250,
        42500, 43750, 45000,  46250,  47500,  48750,  50000,  51500,  53000,  54500,  56000,  58000,  60000,  61500,
        63000, 65000, 67000,  69000,  71000,  73000,  75000,  77500,  80000,  82500,  85000,  87500,  90000,  92500,
        95000, 97500, 100000, 103000, 106000, 109000, 112000, 115000, 118000, 121000, 125000, 128500, 132000, 136000};

    unsigned int nw = sizeof(Weight_per_Tire) / sizeof(double);
    const double g = 9.81;
    double fmax;
    if (li < nw) {
        fmax = Weight_per_Tire[li] * g;
    } else {
        fmax = Weight_per_Tire[nw - 1] * g;
    }
    return fmax;
}

void ChTMeasyTire::WritePlots(const std::string& plName, const std::string& plTitle) {
    double fz[5], dfx0[5], sxm[5], sxs[5], fxm[5], fxs[5];
    double dfy0[5], sym[5], sys[5], fym[5], fys[5];
    double nto0[5], synto0[5], syntoE[5];
    double len[5];

    for (int i = 0; i < 5; i++) {
        fz[i] = 0.5 * double(i + 1) * m_TMeasyCoeff.pn;

        dfx0[i] = InterpQ(fz[i], m_TMeasyCoeff.dfx0_pn, m_TMeasyCoeff.dfx0_p2n);
        fxm[i] = InterpQ(fz[i], m_TMeasyCoeff.fxm_pn, m_TMeasyCoeff.fxm_p2n);
        fxs[i] = InterpQ(fz[i], m_TMeasyCoeff.fxs_pn, m_TMeasyCoeff.fxs_p2n);
        sxm[i] = InterpL(fz[i], m_TMeasyCoeff.sxm_pn, m_TMeasyCoeff.sxm_p2n);
        sxs[i] = InterpL(fz[i], m_TMeasyCoeff.sxs_pn, m_TMeasyCoeff.sxs_p2n);

        dfy0[i] = InterpQ(fz[i], m_TMeasyCoeff.dfy0_pn, m_TMeasyCoeff.dfy0_p2n);
        fym[i] = InterpQ(fz[i], m_TMeasyCoeff.fym_pn, m_TMeasyCoeff.fym_p2n);
        fys[i] = InterpQ(fz[i], m_TMeasyCoeff.fys_pn, m_TMeasyCoeff.fys_p2n);
        sym[i] = InterpL(fz[i], m_TMeasyCoeff.sym_pn, m_TMeasyCoeff.sym_p2n);
        sys[i] = InterpL(fz[i], m_TMeasyCoeff.sys_pn, m_TMeasyCoeff.sys_p2n);

        nto0[i] = InterpL(fz[i], m_TMeasyCoeff.nto0_pn, m_TMeasyCoeff.nto0_p2n);
        synto0[i] = InterpL(fz[i], m_TMeasyCoeff.synto0_pn, m_TMeasyCoeff.synto0_p2n);
        syntoE[i] = InterpL(fz[i], m_TMeasyCoeff.syntoE_pn, m_TMeasyCoeff.syntoE_p2n);

        double d = (-m_a1 + sqrt(m_a1 * m_a1 + 4.0 * m_a2 * fz[i] / 1000.0)) / (2.0 * m_a2);
        len[i] = 2.0 * sqrt(m_unloaded_radius * d);
        std::cout << "d = " << d << "   len = " << len[i] << std::endl;
    }

    if (m_TMeasyCoeff.mu_0 == 0.0 || m_a1 == 0.0) {
        std::cout << "Tire Object not initialised - nothing to plot!" << std::endl;
        return;
    }
    std::string titleName = std::string("TMEasy Tire ") + this->GetName() + ": ";
    titleName += plTitle;
    std::ofstream plot(plName);
    double plen1 = 2.0 * sqrt(m_unloaded_radius * m_TMeasyCoeff.pn / m_TMeasyCoeff.cz);
    double plen2 = 2.0 * sqrt(m_unloaded_radius * 2.0 * m_TMeasyCoeff.pn / m_TMeasyCoeff.cz);
    int ndat = 101;
    double step = 0.01;
    plot << "a = " << m_a1 << std::endl;
    plot << "b = " << m_a2 << std::endl;
    plot << "f(x) = 1000.0*(a*x+b*x*x)" << std::endl;
    if (m_tire_test_defl.size() > 2) {
        plot << "$TestDat << EOD" << std::endl;
        for (int i = 0; i < m_tire_test_defl.size(); i++) {
            plot << m_tire_test_defl[i] << "\t" << m_tire_test_frc[i] << std::endl;
        }
        plot << "EOD" << std::endl;
    }
    plot << "$Fx << EOD" << std::endl;
    for (int i = 0; i < ndat; i++) {
        double sx = step * double(i);
        double fx[5], fos;
        plot << sx;
        for (int j = 0; j < 5; j++) {
            tmxy_combined(fx[j], fos, std::abs(sx), dfx0[j], sxm[j], fxm[j], sxs[j], fxs[j]);
            plot << "\t" << fx[j];
        }
        plot << std::endl;
    }
    plot << "EOD" << std::endl;

    plot << "$Fy << EOD" << std::endl;
    for (int i = 0; i < ndat; i++) {
        double sy = step * double(i);
        double fy[5], fos;
        plot << sy;
        for (int j = 0; j < 5; j++) {
            tmxy_combined(fy[j], fos, std::abs(sy), dfy0[j], sym[j], fym[j], sys[j], fys[j]);
            plot << "\t" << fy[j];
        }
        plot << std::endl;
    }
    plot << "EOD" << std::endl;

    plot << "$Mz << EOD" << std::endl;
    for (int i = 0; i < ndat; i++) {
        double sy = step * double(i);
        double mz[5], fy[5], fos;
        plot << sy;
        for (int j = 0; j < 5; j++) {
            tmxy_combined(fy[j], fos, std::abs(sy), dfy0[j], sym[j], fym[j], sys[j], fys[j]);
            mz[j] = fy[j] * len[j] * tmy_tireoff(std::abs(sy), nto0[j], synto0[j], syntoE[j]);
            plot << "\t" << mz[j];
        }
        plot << std::endl;
    }
    plot << "EOD" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Vertcal Tire Deflection z []'" << std::endl;
    plot << "set ylabel 'Vertical Force Fz [N]'" << std::endl;
    plot << "set xrange [0:0.1]" << std::endl;
    if (m_tire_test_defl.size() > 2) {
        plot << "plot $TestDat t 'Test Data' with points, f(x) t 'Interpolation' with lines" << std::endl;
    } else {
        plot << "plot $TestDat f(x) with lines" << std::endl;
    }
    plot << "pause -1" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Longitudinal Slip sx []'" << std::endl;
    plot << "set ylabel 'Longitudinal Force Fx [N]'" << std::endl;
    plot << "set xrange [0:1]" << std::endl;
    plot << "plot ";
    for (int i = 0; i < 5; i++) {
        if (i < 5 - 1) {
            plot << " $Fx u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines, \\" << std::endl;
        } else {
            plot << " $Fx u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines" << std::endl;
        }
    }
    plot << "pause -1" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Lateral Slip sy []'" << std::endl;
    plot << "set ylabel 'Lateral Force Fy [N]'" << std::endl;
    plot << "plot ";
    for (int i = 0; i < 5; i++) {
        if (i < 5 - 1) {
            plot << " $Fy u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines, \\" << std::endl;
        } else {
            plot << " $Fy u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines" << std::endl;
        }
    }
    plot << "pause -1" << std::endl;

    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Lateral Slip sy []'" << std::endl;
    plot << "set ylabel 'Self Aligning Torque Mz [Nm]'" << std::endl;
    plot << "plot ";
    for (int i = 0; i < 5; i++) {
        if (i < 5 - 1) {
            plot << " $Mz u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines, \\" << std::endl;
        } else {
            plot << " $Mz u 1:" << (i + 2) << " title 'Fz = " << fz[i] << " N' with lines" << std::endl;
        }
    }
    plot << "pause -1" << std::endl;

    plot.close();
}

// No Data available? Try this to get a working truck tire
void ChTMeasyTire::GuessTruck80Par(unsigned int li,   // tire load index
                                   double tireWidth,  // [m]
                                   double ratio,      // [] = use 0.75 meaning 75%
                                   double rimDia,     // rim diameter [m]
                                   double pinfl_li,   // inflation pressure at load index
                                   double pinfl_use)  // inflation pressure in this configuration
{
    double tireLoad = GetTireMaxLoad(li);
    GuessTruck80Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use);
}

// No Data available? Try this to get a working truck tire
void ChTMeasyTire::GuessTruck80Par(double tireLoad,   // tire load force [N]
                                   double tireWidth,  // [m]
                                   double ratio,      // [] = use 0.75 meaning 75%
                                   double rimDia,     // rim diameter [m]
                                   double pinfl_li,   // inflation pressure at load index
                                   double pinfl_use)  // inflation pressure in this configuration
{
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload
    double xi = 0.5;                   // damping ratio

    m_TMeasyCoeff.pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    m_TMeasyCoeff.pn_max = 3.5 * m_TMeasyCoeff.pn;

    double CZ = tireLoad / defl_max;
    double DZ = 2.0 * xi * sqrt(CZ * GetMass());

    SetVerticalStiffness(CZ);

    SetRollingResistanceCoefficients(0.015, 0.015);

    SetDynamicRadiusCoefficients(0.375, 0.75);

    m_TMeasyCoeff.dz = DZ;
    m_TMeasyCoeff.cx = 0.9 * CZ;
    m_TMeasyCoeff.dx = xi * sqrt(m_TMeasyCoeff.cx * GetMass());
    m_TMeasyCoeff.cy = 0.8 * CZ;
    m_TMeasyCoeff.dy = xi * sqrt(m_TMeasyCoeff.cy * GetMass());

    m_rim_radius = 0.5 * rimDia;
    m_roundness = 0.1;

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_TMeasyCoeff.mu_0 = 0.8;

    m_TMeasyCoeff.dfx0_pn = 17.6866 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxm_pn = 0.12;
    m_TMeasyCoeff.fxm_pn = 0.88468 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxs_pn = 0.9;
    m_TMeasyCoeff.fxs_pn = 0.54397 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfx0_p2n = 13.8046 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxm_p2n = 0.15;
    m_TMeasyCoeff.fxm_p2n = 0.7479 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxs_p2n = 0.95;
    m_TMeasyCoeff.fxs_p2n = 0.50365 * 2.0 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfy0_pn = 5.948 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sym_pn = 0.38786;
    m_TMeasyCoeff.fym_pn = 0.77253 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sys_pn = 0.82534;
    m_TMeasyCoeff.fys_pn = 0.71139 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfy0_p2n = 5.506 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sym_p2n = 0.38786;
    m_TMeasyCoeff.fym_p2n = 0.73048 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sys_p2n = 0.91309;
    m_TMeasyCoeff.fys_p2n = 0.66823 * 2.0 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.nto0_pn = 0.178;
    m_TMeasyCoeff.synto0_pn = 0.40726;
    m_TMeasyCoeff.syntoE_pn = 0.82534;

    m_TMeasyCoeff.nto0_p2n = 0.19;
    m_TMeasyCoeff.synto0_p2n = 0.40726;
    m_TMeasyCoeff.syntoE_p2n = 0.91309;
}

void ChTMeasyTire::GuessPassCar70Par(unsigned int li,   // tire load index
                                     double tireWidth,  // [m]
                                     double ratio,      // [] = use 0.75 meaning 75%
                                     double rimDia,     // rim diameter [m]
                                     double pinfl_li,   // inflation pressure at load index
                                     double pinfl_use)  // inflation pressure in this configuration
{
    double tireLoad = GetTireMaxLoad(li);
    GuessPassCar70Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use);
}

void ChTMeasyTire::GuessPassCar70Par(double tireLoad,   // tire load force [N]
                                     double tireWidth,  // [m]
                                     double ratio,      // [] = use 0.75 meaning 75%
                                     double rimDia,     // rim diameter [m]
                                     double pinfl_li,   // inflation pressure at load index
                                     double pinfl_use)  // inflation pressure in this configuration
{
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload
    double xi = 0.5;                   // damping ration

    m_TMeasyCoeff.pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    m_TMeasyCoeff.pn_max = 3.5 * m_TMeasyCoeff.pn;

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_TMeasyCoeff.mu_0 = 0.8;

    double CZ = tireLoad / defl_max;
    double DZ = 2.0 * xi * sqrt(CZ * GetMass());

    SetVerticalStiffness(CZ);

    SetRollingResistanceCoefficients(0.015, 0.015);

    SetDynamicRadiusCoefficients(0.375, 0.75);

    m_TMeasyCoeff.dz = DZ;
    m_TMeasyCoeff.cx = 0.9 * CZ;
    m_TMeasyCoeff.dx = xi * sqrt(m_TMeasyCoeff.cx * GetMass());
    m_TMeasyCoeff.cy = 0.8 * CZ;
    m_TMeasyCoeff.dy = xi * sqrt(m_TMeasyCoeff.cy * GetMass());

    m_rim_radius = 0.5 * rimDia;
    m_roundness = 0.1;

    m_TMeasyCoeff.dfx0_pn = 18.6758 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxm_pn = 0.17;
    m_TMeasyCoeff.fxm_pn = 1.1205 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxs_pn = 0.9;
    m_TMeasyCoeff.fxs_pn = 0.8766 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfx0_p2n = 20.1757 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxm_p2n = 0.15;
    m_TMeasyCoeff.fxm_p2n = 1.072 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sxs_p2n = 0.95;
    m_TMeasyCoeff.fxs_p2n = 0.8245 * 2.0 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfy0_pn = 14.9858 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sym_pn = 0.18197;
    m_TMeasyCoeff.fym_pn = 1.0084 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sys_pn = 0.82534;
    m_TMeasyCoeff.fys_pn = 0.83941 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.dfy0_p2n = 10.0505 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sym_p2n = 0.24472;
    m_TMeasyCoeff.fym_p2n = 0.90003 * 2.0 * m_TMeasyCoeff.pn * N2kN;
    m_TMeasyCoeff.sys_p2n = 0.91309;
    m_TMeasyCoeff.fys_p2n = 0.76782 * 2.0 * m_TMeasyCoeff.pn * N2kN;

    m_TMeasyCoeff.nto0_pn = 0.178;
    m_TMeasyCoeff.synto0_pn = 0.19107;
    m_TMeasyCoeff.syntoE_pn = 0.82534;

    m_TMeasyCoeff.nto0_p2n = 0.19;
    m_TMeasyCoeff.synto0_p2n = 0.25695;
    m_TMeasyCoeff.syntoE_p2n = 0.91309;
}

// Do some rough constency checks
bool ChTMeasyTire::CheckParameters() {
    bool isOk = false;

    // Nominal Load set?
    if (m_TMeasyCoeff.pn < GetTireMaxLoad(0)) {
        std::cout << "TMeasyCheckParameters(): Tire Nominal Load Problem!" << std::endl;
        return isOk;
    }

    // Stiffness parameters, spring
    if (m_a1 <= 0.0) {
        std::cout << "TMeasyCheckParameters(): Tire Vertical Stiffness Problem!" << std::endl;
        return isOk;
    }

    // Stiffness parameters, spring
    if (m_TMeasyCoeff.mu_0 <= 0.0) {
        std::cout << "TMeasyCheckParameters(): Friction Coefficien Mu_0 unset!" << std::endl;
        return isOk;
    }

    if (m_TMeasyCoeff.dz <= 0.0) {
        std::cout << "TMeasyCheckParameters(): Tire Vertical Damping Problem!" << std::endl;
        return isOk;
    }

    // Stiffness sx
    if (m_TMeasyCoeff.dfx0_pn <= 0.0 || m_TMeasyCoeff.dfx0_pn < (2.0 * m_TMeasyCoeff.fxm_pn / m_TMeasyCoeff.sxm_pn)) {
        std::cout << "TMeasyCheckParameters(): fx(sx) slope at sx=0 too low, load level 1!" << std::endl;
        std::cout << m_TMeasyCoeff.dfx0_pn << " < " << (2.0 * m_TMeasyCoeff.fxm_pn / m_TMeasyCoeff.sxm_pn) << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.dfx0_p2n <= 0.0 ||
        m_TMeasyCoeff.dfx0_p2n < (2.0 * m_TMeasyCoeff.fxm_p2n / m_TMeasyCoeff.sxm_p2n)) {
        std::cout << "TMeasyCheckParameters(): fx(sx) slope at sx=0 too low, load level 2!" << std::endl;
        return isOk;
    }

    // Stiffness sy
    if (m_TMeasyCoeff.dfy0_pn <= 0.0 || m_TMeasyCoeff.dfy0_pn < (2.0 * m_TMeasyCoeff.fym_pn / m_TMeasyCoeff.sym_pn)) {
        std::cout << "TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 1!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.dfy0_p2n <= 0.0 ||
        m_TMeasyCoeff.dfy0_p2n < (2.0 * m_TMeasyCoeff.fym_p2n / m_TMeasyCoeff.sym_p2n)) {
        std::cout << "TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 2!" << std::endl;
        return isOk;
    }

    // Check single curve parameters
    if (m_TMeasyCoeff.sxm_pn <= 0.0) {
        std::cout << "TMeasyCheckParameters(): sxm load level 1 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.sxm_p2n <= 0.0) {
        std::cout << "TMeasyCheckParameters(): sxm load level 2 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.fxm_pn <= 0.0) {
        std::cout << "TMeasyCheckParameters(): fxm load level 1 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.fxm_p2n <= 0.0) {
        std::cout << "TMeasyCheckParameters(): fxm load level 2 not set!" << std::endl;
        return isOk;
    }

    if (m_TMeasyCoeff.sym_pn <= 0.0) {
        std::cout << "TMeasyCheckParameters(): sym load level 1 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.sym_p2n <= 0.0) {
        std::cout << "TMeasyCheckParameters(): sym load level 2 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.fym_pn <= 0.0) {
        std::cout << "TMeasyCheckParameters(): fym load level 1 not set!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.fym_p2n <= 0.0) {
        std::cout << "TMeasyCheckParameters(): fym load level 2 not set!" << std::endl;
        return isOk;
    }

    if (m_TMeasyCoeff.sxm_pn >= m_TMeasyCoeff.sxs_pn) {
        std::cout << "TMeasyCheckParameters(): sxm >= sxs load level 1!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.sxm_p2n >= m_TMeasyCoeff.sxs_p2n) {
        std::cout << "TMeasyCheckParameters(): sxm >= sxs load level 2!" << std::endl;
        return isOk;
    }

    if (m_TMeasyCoeff.sym_pn >= m_TMeasyCoeff.sys_pn) {
        std::cout << "TMeasyCheckParameters(): sym >= sys load level 1!" << std::endl;
        return isOk;
    }
    if (m_TMeasyCoeff.sym_p2n >= m_TMeasyCoeff.sys_p2n) {
        std::cout << "TMeasyCheckParameters(): sym >= sys load level 2!" << std::endl;
        return isOk;
    }

    isOk = true;

    return isOk;
}

// set tire reference coefficient of friction
void ChTMeasyTire::SetFrictionCoefficient(double coeff) {
    m_TMeasyCoeff.mu_0 = coeff;
}

// Set Rolling Resistance Coefficients
void ChTMeasyTire::SetRollingResistanceCoefficients(double rr_coeff_1, double rr_coeff_2) {
    m_TMeasyCoeff.rrcoeff_pn = rr_coeff_1;
    m_TMeasyCoeff.rrcoeff_p2n = rr_coeff_2;
}

// Set Dynamic Radius Coefficients
void ChTMeasyTire::SetDynamicRadiusCoefficients(double rdyn_coeff_1, double rdyn_coeff_2) {
    m_TMeasyCoeff.rdynco_pn = rdyn_coeff_1;
    m_TMeasyCoeff.rdynco_p2n = rdyn_coeff_2;
}

void ChTMeasyTire::ExportParameterFile(std::string fileName) {
    // Generate a tire parameter file from programmatical setup
    std::ofstream tpf(fileName);
    if (!tpf.good()) {
        std::cout << GetName() << ": ChTMeasyTire::ExportParameterFile() no export possible!" << std::endl;
        return;
    }
    double cz1 = kN2N * sqrt(pow(m_a1, 2) + 4.0 * m_a2 * N2kN * m_TMeasyCoeff.pn);
    double cz2 = kN2N * sqrt(pow(m_a1, 2) + 8.0 * m_a2 * N2kN * m_TMeasyCoeff.pn);

    tpf << "[FRICTION]" << std::endl;
    tpf << "MU_0            = " << std::setprecision(4) << std::setw(12) << m_TMeasyCoeff.mu_0
        << "   $ coeff of friction at test conditions []" << std::endl;
    tpf << "[DIMENSION]" << std::endl;
    tpf << "MASS            = " << std::setprecision(4) << std::setw(12) << 71.1
        << "   $ mass of tire, wheel/rim not included [kg]" << std::endl;
    tpf << "THETA_XZ        = " << std::setprecision(4) << std::setw(12) << 9.62
        << "   $ inertia of tire around X and Z axis, wheel/rim not included [kg*m^2]" << std::endl;
    tpf << "THETA_Y         = " << std::setprecision(4) << std::setw(12) << 16.84
        << "   $ inertia of tire around Y axis, wheel/rim not included [kg*m^2]" << std::endl;
    tpf << "UNLOADED_RADIUS = " << std::setprecision(4) << std::setw(12) << m_unloaded_radius
        << "   $ unloaded radius [m]" << std::endl;
    tpf << "WIDTH           = " << std::setprecision(4) << std::setw(12) << m_width << "   $ width [m]" << std::endl;
    tpf << "RIM_RADIUS      = " << std::setprecision(4) << std::setw(12) << m_rim_radius << "   $ rim radius [m]"
        << std::endl;  // unused
    tpf << "ROUNDNESS       = " << std::setprecision(4) << std::setw(12) << m_roundness
        << "   $ roundness of tire cross-section [-]" << std::endl;  // unused
    tpf << "$--------------------------------------------------------------------load" << std::endl;
    tpf << "[TYRE_LOAD]" << std::endl;
    tpf << "FZ_NOM          = " << std::setprecision(8) << std::setw(12) << m_TMeasyCoeff.pn
        << "   $ nominal or payload [N]" << std::endl;
    tpf << "FZ_MAX          = " << std::setprecision(8) << std::setw(12) << (3.5 * m_TMeasyCoeff.pn)
        << "   $ max extrapolation load [N]" << std::endl;
    tpf << "$---------------------------------------------------------------stiffness" << std::endl;
    tpf << "[STIFFNESS]" << std::endl;
    tpf << "CLONG           = " << std::setprecision(8) << std::setw(12) << m_TMeasyCoeff.cx
        << "   $ longitudinal [N/m]" << std::endl;
    tpf << "CLAT            = " << std::setprecision(8) << std::setw(12) << m_TMeasyCoeff.cy << "   $ lateral [N/m]"
        << std::endl;
    tpf << "CVERT_1         = " << std::setprecision(8) << std::setw(12) << cz1 << "   $ vertical @ Fz=Fz_nom [N/m]"
        << std::endl;
    tpf << "CVERT_2         = " << std::setprecision(8) << std::setw(12) << cz2 << "   $ vertical @ Fz=2*Fz_nom [N/m]"
        << std::endl;
    tpf << "$-----------------------------------------------------------------damping" << std::endl;
    tpf << "[DAMPING]" << std::endl;
    tpf << "DLONG           = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.dx
        << "   $ longitudinal [N/(m/s)]" << std::endl;
    tpf << "DLAT            = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.dy << "   $ lateral [N/(m/s)]"
        << std::endl;
    tpf << "DVERT           = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.dz
        << "   $ vertical [N/(m/s)]" << std::endl;
    tpf << "$------------------------------------------------------rolling resistance" << std::endl;
    tpf << "[ROLLING_RESISTANCE]" << std::endl;
    tpf << "RRCOEFF_1       = " << std::setprecision(4) << std::setw(12) << m_TMeasyCoeff.rrcoeff_pn
        << "   $ roll. rest. coefficient @ Fz=Fz_nom [-]" << std::endl;
    tpf << "RRCOEFF_2       = " << std::setprecision(4) << std::setw(12) << m_TMeasyCoeff.rrcoeff_p2n
        << "   $ roll. rest. coefficient @ Fz=2*Fz_nom [-]" << std::endl;
    tpf << "$----------------------------------------------------------dynamic radius" << std::endl;
    tpf << "[DYNAMIC_RADIUS]" << std::endl;
    tpf << "RDYNCO_1       = " << std::setprecision(4) << std::setw(12) << m_TMeasyCoeff.rdynco_pn
        << "   $ rdyn weighting coefficient @ Fz=Fz_nom [-]" << std::endl;
    tpf << "RDYNCO_2       = " << std::setprecision(4) << std::setw(12) << m_TMeasyCoeff.rdynco_p2n
        << "   $ rdyn weighting coefficient @ Fz=2*Fz_nom [-]" << std::endl;

    tpf << "$------------------------------------------------------longitudinal force" << std::endl;
    tpf << "[LONGITUDINAL_PARAMETERS]" << std::endl;
    tpf << "DFX0_1         = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.dfx0_pn
        << "   $ initial slope @ Fz=Fz_nom [N/-]" << std::endl;
    tpf << "FXMAX_1        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fxm_pn
        << "   $ maximum force @ Fz=Fz_nom [N]" << std::endl;
    tpf << "SXMAX_1        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sxm_pn
        << "   $ slip sx where Fx=Fxmax_1 [-]" << std::endl;
    tpf << "FXSLD_1        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fxs_pn
        << "   $ sliding force @ Fz=Fz_nom [N]" << std::endl;
    tpf << "SXSLD_1        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sxs_pn
        << "   $ slip sx where Fx=Fxsld_1 [-]" << std::endl;
    tpf << "$------------------------------------------------------------------------" << std::endl;
    tpf << "DFX0_2         = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.dfx0_p2n
        << "   $ initial slope @ Fz=2*Fz_nom [N/-]" << std::endl;
    tpf << "FXMAX_2        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fxm_p2n
        << "   $ maximum force @ Fz=2*Fz_nom [N]" << std::endl;
    tpf << "SXMAX_2        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sxm_p2n
        << "   $ slip sx where Fx=Fxmax_2 [-]" << std::endl;
    tpf << "FXSLD_2        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fxs_p2n
        << "   $ sliding force @ Fz=2*Fz_nom [N]" << std::endl;
    tpf << "SXSLD_2        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sxs_p2n
        << "   $ slip sx where Fx=Fxsld_2 [-]" << std::endl;

    tpf << "$-----------------------------------------------------------lateral force" << std::endl;
    tpf << "[LATERAL_PARAMETERS]" << std::endl;
    tpf << "DFY0_1         = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.dfy0_pn
        << "   $ initial slope @ Fz=Fz_nom [N/-]" << std::endl;
    tpf << "FYMAX_1        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fym_pn
        << "   $ maximum force @ Fz=Fz_nom [N]" << std::endl;
    tpf << "SYMAX_1        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sym_pn
        << "   $ slip sy where Fy=Fymax_1 [-]" << std::endl;
    tpf << "FYSLD_1        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fys_pn
        << "   $ sliding force @ Fz=Fz_nom [N]" << std::endl;
    tpf << "SYSLD_1        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sys_pn
        << "   $ slip sy where Fy=Fysld_1 [-]" << std::endl;
    tpf << "$------------------------------------------------------------------------" << std::endl;
    tpf << "DFY0_2         = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.dfy0_p2n
        << "   $ initial slope @ Fz=2*Fz_nom [N/-]" << std::endl;
    tpf << "FYMAX_2        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fym_p2n
        << "   $ maximum force @ Fz=2*Fz_nom [N]" << std::endl;
    tpf << "SYMAX_2        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sym_p2n
        << "   $ slip sy where Fy=Fymax_2 [-]" << std::endl;
    tpf << "FYSLD_2        = " << std::setprecision(8) << std::setw(12) << kN2N * m_TMeasyCoeff.fys_p2n
        << "   $ sliding force @ Fz=2*Fz_nom [N]" << std::endl;
    tpf << "SYSLD_2        = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.sys_p2n
        << "   $ slip sy where Fy=Fysld_2 [-]" << std::endl;

    tpf << "$-----------------------------------------------------------pneumatic trail" << std::endl;
    tpf << "[ALIGNING_PARAMETERS]" << std::endl;
    tpf << "PT_NORM_1     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.nto0_pn
        << "   $ norm. pneumatic trail @ sy=0 & Fz=Fz_nom [-]" << std::endl;
    tpf << "SY_CHSI_1     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.synto0_pn
        << "   $ sy where trail changes sign @ Fz=Fz_nom [-]" << std::endl;
    tpf << "SY_ZERO_1     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.syntoE_pn
        << "   $ sy where trail tends to zero @ Fz=Fz_nom [-]" << std::endl;
    tpf << "$------------------------------------------------------------------------" << std::endl;
    tpf << "PT_NORM_2     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.nto0_p2n
        << "   $ norm. pneumatic trail @ sy=0 & Fz=2*Fz_nom [-]" << std::endl;
    tpf << "SY_CHSI_2     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.synto0_p2n
        << "   $ sy where trail changes sign @ Fz=2*Fz_nom [-]" << std::endl;
    tpf << "SY_ZERO_2     = " << std::setprecision(5) << std::setw(12) << m_TMeasyCoeff.syntoE_p2n
        << "   $ sy where trail tends to zero @ Fz=2*Fz_nom [-]" << std::endl;

    tpf.close();
}

using namespace rapidjson;

void ChTMeasyTire::ExportJSONFile(std::string jsonFileName) {
    const char* json =
        "{"
        "\"Name\":\"Truck Tire\","
        "\"Type\":\"Tire\","
        "\"Template\":\"TMeasyTire\","
        "\"Unloaded Radius\":0.0,"
        "\"Tire Width\":0.0,"
        "\"Tire Mass\":0.0,"
        "\"Tire Inertia\":[0.0, 0.0, 0.0],"

        "\"mu_0\":0"
        "}";
    Document d;
    d.Parse(json);
    // 2. Modify it by DOM.
    {
        Value& s = d["mu_0"];
        s.SetDouble(m_TMeasyCoeff.mu_0);
    }
    {
        Value& s = d["Unloaded Radius"];
        s.SetDouble(m_unloaded_radius);
    }
    // 3. Stringify the DOM
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    d.Accept(writer);
    // Output {"project":"rapidjson","stars":11}
    std::cout << buffer.GetString() << std::endl;
}

}  // end namespace vehicle
}  // end namespace chrono
