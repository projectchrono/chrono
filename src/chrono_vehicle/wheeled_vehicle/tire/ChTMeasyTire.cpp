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
// Authors: Rainer Gericke
// =============================================================================
//
// Template for the "Tire Model made Easy". Our implementation is a basic version
// of the algorithms in http://www.tmeasy.de/, a comercial tire simulation code
// developed by Prof. Dr. Georg Rill.
//
//
// Ref: Georg Rill, "Road Vehicle Dynamics - Fundamentals and Modelling",
//          https://www.routledge.com/Road-Vehicle-Dynamics-Fundamentals-and-Modeling-with-MATLAB/Rill-Castro/p/book/9780367199739
//      Georg Rill, "An Engineer's Guess On Tyre Model Parameter Made Possible With TMeasy",
//          https://www.researchgate.net/publication/317036908_An_Engineer's_Guess_on_Tyre_Parameter_made_possible_with_TMeasy
//      Georg Rill, "Simulation von Kraftfahrzeugen",
//          https://www.researchgate.net/publication/317037037_Simulation_von_Kraftfahrzeugen
//
// Known differences to the comercial version:
//  - No parking slip calculations
//  - No dynamic parking torque
//  - No dynamic tire inflation pressure
//  - No belt dynamics
//  - Simplified stand still handling
//  - Optional tire contact smoothing based on "A New Analytical Tire Model for Vehicle Dynamic Analysis" by
//      J. Shane Sui & John A Hirshey II
//
// This implementation has been validated with:
//  - FED-Alpha vehicle model
//  - Tire data sets gained by conversion of Pac02 TIR parameter files
//  - Steady state cornering test and test results from Keweenah Research Center (KRC)
//  - unvalidateble functionality has been removed
// ===================================================================================

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChLog.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace chrono {
namespace vehicle {

ChTMeasyTire::ChTMeasyTire(const std::string& name)
    : ChForceElementTire(name),
      m_vnum(0.01),
      m_gamma_limit(4),
      m_begin_start_transition(0.1),
      m_end_start_transition(0.25),
      m_use_startup_transition(false),
      m_vcoulomb(1.0),
      m_frblend_begin(1.0),
      m_frblend_end(3.0),
      m_bottom_radius(0.0),
      m_bottom_stiffness(0.0),
      m_rolling_resistance(0.01) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    m_par.pn = 0.0;
    m_par.mu_0 = 0.8;
}

// -----------------------------------------------------------------------------

void ChTMeasyTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetTMeasyParams();

    // unset bottoming parameters?
    if (m_bottom_radius == 0) {
        m_bottom_radius = m_rim_radius + 0.01;  // consider thickness of the carcass
    }
    if (m_bottom_stiffness == 0.0) {
        m_bottom_stiffness = 5.0 * m_d1;
    }

    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    // Initialize contact patch state variables to 0;
    m_states.sx = 0;
    m_states.sy = 0;
    m_states.vta = m_vnum;
    m_states.R_eff = m_unloaded_radius;
}

// -----------------------------------------------------------------------------

void ChTMeasyTire::Synchronize(double time, const ChTerrain& terrain) {
    m_time = time;
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu_road;
    m_data.in_contact = DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                             m_width, m_areaDep, m_data.frame, m_data.depth, mu_road);
    ChClampValue(mu_road, 0.1f, 1.0f);

    m_states.muscale = mu_road / m_par.mu_0;

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

    if (m_par.pn <= 0.0) {
        GetLog() << "FATAL error in " << __func__ << ": Nominal Force has not been set!\n";
        exit(99);
    }
    m_states.gamma = ChClamp(GetCamberAngle(), -m_gamma_limit * CH_C_DEG_TO_RAD, m_gamma_limit * CH_C_DEG_TO_RAD);

    if (m_data.in_contact) {
        // Wheel velocity in the ISO-C Frame
        ChVector<> vel = wheel_state.lin_vel;
        m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

        // Generate normal contact force. If the resulting force is negative, the disc
        // is moving away from the terrain so fast that no contact force is generated.
        // The sign of the velocity term in the damping function is negative since a
        // positive velocity means a decreasing depth, not an increasing depth.
        double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        // Skip Force and moment calculations when the normal force = 0
        if (Fn_mag < 0) {
            Fn_mag = 0;
            m_data.in_contact = false;
        }

        m_data.normal_force = Fn_mag;
        m_states.q = ChClamp(Fn_mag, 0.0, m_par.pn_max) / m_par.pn;
        double r_stat = m_unloaded_radius - m_data.depth;
        m_states.omega = wheel_state.omega;
        m_states.R_eff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
        m_states.P_len = 2.0 * sqrt(m_unloaded_radius * m_data.depth);
        m_states.vta = m_states.R_eff * std::abs(m_states.omega) + m_vnum;
        m_states.vsx = m_data.vel.x() - m_states.omega * m_states.R_eff;
        m_states.vsy = m_data.vel.y();
        m_states.sx = -m_states.vsx / m_states.vta;
        m_states.sy = -m_states.vsy / m_states.vta;
        m_states.dfx0 = InterpQ(m_par.dfx0_pn, m_par.dfx0_p2n);  // does not vary with road friction
        m_states.sxm = InterpL(m_par.sxm_pn, m_par.sxm_p2n);
        m_states.fxm = InterpQ(m_par.fxm_pn, m_par.fxm_p2n);
        m_states.sxs = InterpL(m_par.sxs_pn, m_par.sxs_p2n);
        m_states.fxs = InterpQ(m_par.fxs_pn, m_par.fxs_p2n);
        m_states.dfy0 = InterpQ(m_par.dfy0_pn, m_par.dfy0_p2n);  // does not vary with road friction
        m_states.sym = InterpL(m_par.sym_pn, m_par.sym_p2n);
        m_states.fym = InterpQ(m_par.fym_pn, m_par.fym_p2n);
        m_states.sys = InterpL(m_par.sys_pn, m_par.sys_p2n);
        m_states.fys = InterpQ(m_par.fys_pn, m_par.fys_p2n);
        m_states.nL0 = InterpL(m_par.nL0_pn, m_par.nL0_p2n);
        m_states.sq0 = InterpL(m_par.sq0_pn, m_par.sq0_p2n);
        m_states.sqe = InterpL(m_par.sqe_pn, m_par.sqe_p2n);
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = m_unloaded_radius;
        m_states.P_len = 0;
        m_states.q = 0;
        m_states.sx = 0;
        m_states.sy = 0;
        m_states.vta = m_vnum;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.nL0 = 0;
        m_states.sq0 = 0;
        m_states.sqe = 0;
        m_states.brx = 0;
        m_states.bry = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChTMeasyTire::Advance(double step) {
    // Set tire forces to zero.
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data.in_contact)
        return;

    // Limit the effect of Fz on handling forces and torques to avoid nonsensical extrapolation of the curve
    // coefficients
    // m_data.normal_force is nevertheless still taken as the applied vertical tire force
    double Fz = std::min(m_data.normal_force, m_par.pn_max);

    double Fx = 0, Fy = 0;
    double Fx0, Fy0;

    CombinedCoulombForces(Fx0, Fy0, Fz, m_states.muscale);

    double frblend = ChSineStep(m_data.vel.x(), m_frblend_begin, 0.0, m_frblend_end, 1.0);

    // TMeasy horizontal forces
    double sx = m_states.sx;
    double sy = m_states.sy;

    double sc = hypot(sx, sy);
    double salpha = 0, calpha = 0;

    if (sc > 0) {
        calpha = sx / sc;
        salpha = sy / sc;
    } else {
        calpha = sqrt(2.0) / 2.0;
        salpha = sqrt(2.0) / 2.0;
    }

    // Calculate resultant Curve Parameters
    double df0 = hypot(m_states.dfx0 * calpha, m_states.dfy0 * salpha);
    double fm = m_states.muscale * hypot(m_states.fxm * calpha, m_states.fym * salpha);
    double sm = m_states.muscale * hypot(m_states.sxm * calpha, m_states.sym * salpha);
    double fs = m_states.muscale * hypot(m_states.fxs * calpha, m_states.fys * salpha);
    double ss = m_states.muscale * hypot(m_states.sxs * calpha, m_states.sys * salpha);
    double f = 0.0;
    double fos = 0.0;

    tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);
    if (sc > 0.0) {
        Fx = f * calpha;
        Fy = f * salpha;
    } else {
        Fx = 0.0;
        Fy = 0.0;
    }

    Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
    Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

    double Mx = 0;
    double My = 0;
    double Mz = 0;

    if (m_data.vel.x() >= m_frblend_begin) {
        Mz = AlignmentTorque(Fy);
    }

    // Rolling Resistance, Ramp Like Signum inhibits 'switching' of My
    My = -m_rolling_resistance * m_data.normal_force * m_unloaded_radius * tanh(m_states.omega);

    // Overturning Torque
    double cg = std::pow(m_width, 2.0) * (m_d1 + 2.0 * m_d2 * m_data.depth) / 12.0;
    Mx = -cg * m_states.gamma;

    double startup = 1;
    if (m_use_startup_transition) {
        startup = ChSineStep(m_time, m_begin_start_transition, 0.0, m_end_start_transition, 1.0);
    }

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    m_tireforce.force = ChVector<>(startup * Fx, startup * Fy, m_data.normal_force);
    m_tireforce.moment = startup * ChVector<>(Mx, My, Mz);
}

void ChTMeasyTire::CombinedCoulombForces(double& fx, double& fy, double fz, double muscale) {
    ChVector2<> F;
    /*
     The Dahl Friction Model elastic tread blocks representated by a single bristle. At tire stand still it acts
     like a spring which enables holding of a vehicle on a slope without creeping (hopefully). Damping terms
     have been added to calm down the oscillations of the pure spring.

     The time step h must be actually the same as for the vehicle system!

     This model is experimental and needs some testing.

     With bristle deformation z, Coulomb force fc, sliding velocity v and stiffness sigma we have this
     differential equation:
         dz/dt = v - sigma0*z*abs(v)/fc

     When z is known, the friction force F can be calulated to:
        F = sigma0 * z

     For practical use some damping is needed, that leads to:
        F = sigma0 * z + sigma1 * dz/dt

     Longitudinal and lateral forces are calculated separately and then combined. For stand still a friction
     circle is used.
     */
    double fc = fz * muscale;
    double h = this->m_stepsize;
    // Longitudinal Friction Force
    double brx_dot = m_states.vsx - m_par.sigma0 * m_states.brx * fabs(m_states.vsx) / fc;  // dz/dt
    F.x() = -(m_par.sigma0 * m_states.brx + m_par.sigma1 * brx_dot);
    // Lateral Friction Force
    double bry_dot = m_states.vsy - m_par.sigma0 * m_states.bry * fabs(m_states.vsy) / fc;  // dz/dt
    F.y() = -(m_par.sigma0 * m_states.bry + m_par.sigma1 * bry_dot);
    // Calculate the new ODE states (implicit Euler)
    m_states.brx = (fc * m_states.brx + fc * h * m_states.vsx) / (fc + h * m_par.sigma0 * fabs(m_states.vsx));
    m_states.bry = (fc * m_states.bry + fc * h * m_states.vsy) / (fc + h * m_par.sigma0 * fabs(m_states.vsy));

    // combine forces (friction circle)
    if (F.Length() > fz * muscale) {
        F.Normalize();
        F *= fz * muscale;
    }
    fx = F.x();
    fy = F.y();
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
}

double ChTMeasyTire::AlignmentTorque(double fy) {
    double nto;
    double nto0 = m_states.nL0;
    double synto0 = m_states.sq0;
    double syntoE = m_states.sqe;

    double sy_a = fabs(m_states.sy);
    double syntoE_loc = std::max(syntoE, synto0);

    if (sy_a >= syntoE_loc) {
        nto = 0;
    } else {
        double wf = synto0 / syntoE_loc;
        double sy_n;
        if (sy_a <= synto0) {
            sy_n = sy_a / synto0;
            double nto1 = nto0 * (1.0 - sy_n);
            double nto2 = nto0 * (1.0 - (3.0 - 2.0 * sy_n) * pow(sy_n, 2));
            nto = (1.0 - wf) * nto1 + wf * nto2;
        } else {
            sy_n = (syntoE_loc - sy_a) / (syntoE_loc - synto0);
            nto = -nto0 * (1.0 - wf) * (sy_a - synto0) / synto0 * pow(sy_n, 2);
        }
    }
    return -fy * m_states.P_len * nto;
}

// -----------------------------------------------------------------------------

double ChTMeasyTire::GetNormalStiffnessForce(double depth) const {
    double F = depth * m_d1 + depth * depth * m_d2;  // tire force
    double free_depth = m_unloaded_radius - m_bottom_radius;
    if (depth - free_depth > 0) {
        F += (depth - free_depth) * m_bottom_stiffness;  // add bottom contact force
    }
    return F;
}

double ChTMeasyTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_par.dz * velocity;
}

// -----------------------------------------------------------------------------

void ChTMeasyTire::SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc) {
    // calculate polynomial coefficients from test data [m],[N/m]
    Eigen::MatrixXd A(defl.size(), 2);
    Eigen::VectorXd b(defl.size());
    Eigen::Vector2d r;
    // scale to [m],[kN/m]
    for (int k = 0; k < defl.size(); k++) {
        A(k, 0) = defl[k];
        A(k, 1) = defl[k] * defl[k];
        b(k) = frc[k];
    }
    r = A.colPivHouseholderQr().solve(b);
    m_d1 = r(0);
    m_d2 = r(1);
    GetLog() << "Stiffness Coeffs from test data d1 = " << m_d1 << "  d2 = " << m_d2 << "\n";
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
    std::ofstream plt(plName);
    plt.close();
}

void ChTMeasyTire::GuessTruck80Par(unsigned int li,       // tire load index
                                   double tireWidth,      // [m]
                                   double ratio,          // [] = use 0.75 meaning 75%
                                   double rimDia,         // rim diameter [m]
                                   double pinfl_li,       // inflation pressure at load index
                                   double pinfl_use,      // inflation pressure in this configuration
                                   double damping_ratio)  // damping ratio
{
    double tireLoad = GetTireMaxLoad(li);
    GuessTruck80Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use, damping_ratio);
}

// truck tire pattern  with ratio = 80%
void ChTMeasyTire::GuessTruck80Par(double tireLoad,       // tire load force [N]
                                   double tireWidth,      // [m]
                                   double ratio,          // [] = use 0.75 meaning 75%
                                   double rimDia,         // rim diameter [m]
                                   double pinfl_li,       // inflation pressure at load index
                                   double pinfl_use,      // inflation pressure in this configuration
                                   double damping_ratio)  // damping ratio
{
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload

    m_par.pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    m_par.pn_max = 3.5 * m_par.pn;

    double CZ = tireLoad / defl_max;
    double DZ = 2.0 * damping_ratio * sqrt(CZ * GetTireMass());

    SetVerticalStiffness(CZ);

    SetRollingResistanceCoefficient(0.015);

    m_par.dz = DZ;

    m_rim_radius = 0.5 * rimDia;

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_par.mu_0 = 0.8;

    // Normalized Parameters gained from data set containing original data from Pacejka book
    m_par.dfx0_pn = 17.7764 * m_par.pn;
    m_par.dfx0_p2n = 14.5301 * 2.0 * m_par.pn;
    m_par.fxm_pn = 0.89965 * m_par.pn;
    m_par.fxm_p2n = 0.77751 * 2.0 * m_par.pn;
    m_par.fxs_pn = 0.46183 * m_par.pn;
    m_par.fxs_p2n = 0.42349 * 2.0 * m_par.pn;
    m_par.sxm_pn = 0.10811;
    m_par.sxm_p2n = 0.12389;
    m_par.sxs_pn = 0.66667;
    m_par.sxs_p2n = 0.66667;
    m_par.dfy0_pn = 7.4013 * m_par.pn;
    m_par.dfy0_p2n = 6.8505 * 2.0 * m_par.pn;
    m_par.fym_pn = 0.75876 * m_par.pn;
    m_par.fym_p2n = 0.72628 * 2.0 * m_par.pn;
    m_par.fys_pn = 0.68276 * m_par.pn;
    m_par.fys_p2n = 0.65319 * 2.0 * m_par.pn;
    m_par.sym_pn = 0.33167;
    m_par.sym_p2n = 0.33216;
    m_par.sys_pn = 1.0296;
    m_par.sys_p2n = 1.0296;
    m_par.nL0_pn = 0.178;
    m_par.sq0_pn = 0.36484;
    m_par.sqe_pn = 1.0296;
    m_par.nL0_p2n = 0.19;
    m_par.sq0_p2n = 0.36538;
    m_par.sqe_p2n = 1.0296;
}

void ChTMeasyTire::GuessPassCar70Par(unsigned int li,       // tire load index
                                     double tireWidth,      // [m]
                                     double ratio,          // [] = use 0.75 meaning 75%
                                     double rimDia,         // rim diameter [m]
                                     double pinfl_li,       // inflation pressure at load index
                                     double pinfl_use,      // inflation pressure in this configuration
                                     double damping_ratio)  // damping ratio
{
    double tireLoad = GetTireMaxLoad(li);
    GuessPassCar70Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use, damping_ratio);
}

// tire pattern for passenger car with ratio = 70%
void ChTMeasyTire::GuessPassCar70Par(double tireLoad,       // tire load force [N]
                                     double tireWidth,      // [m]
                                     double ratio,          // [] = use 0.75 meaning 75%
                                     double rimDia,         // rim diameter [m]
                                     double pinfl_li,       // inflation pressure at load index
                                     double pinfl_use,      // inflation pressure in this configuration
                                     double damping_ratio)  // damping ratio
{
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload

    m_par.pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    m_par.pn_max = 3.5 * m_par.pn;

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_par.mu_0 = 0.8;

    double CZ = tireLoad / defl_max;
    double DZ = 2.0 * damping_ratio * sqrt(CZ * GetTireMass());

    SetVerticalStiffness(CZ);

    SetRollingResistanceCoefficient(0.015);

    m_par.dz = DZ;

    m_rim_radius = 0.5 * rimDia;

    // Normalized Parameters gained from data set containing original data from Pacejka book
    m_par.dfx0_pn = 18.3741 * m_par.pn;
    m_par.dfx0_p2n = 19.4669 * 2.0 * m_par.pn;
    m_par.fxm_pn = 1.1292 * m_par.pn;
    m_par.fxm_p2n = 1.0896 * 2.0 * m_par.pn;
    m_par.fxs_pn = 0.80149 * m_par.pn;
    m_par.fxs_p2n = 0.76917 * 2.0 * m_par.pn;
    m_par.sxm_pn = 0.13913;
    m_par.sxm_p2n = 0.13913;
    m_par.sxs_pn = 0.66667;
    m_par.sxs_p2n = 0.66667;
    m_par.dfy0_pn = 15.9826 * m_par.pn;
    m_par.dfy0_p2n = 12.8509 * 2.0 * m_par.pn;
    m_par.fym_pn = 1.0009 * m_par.pn;
    m_par.fym_p2n = 0.91367 * 2.0 * m_par.pn;
    m_par.fys_pn = 0.8336 * m_par.pn;
    m_par.fys_p2n = 0.77336 * 2.0 * m_par.pn;
    m_par.sym_pn = 0.14852;
    m_par.sym_p2n = 0.18504;
    m_par.sys_pn = 0.96524;
    m_par.sys_p2n = 1.0714;
    m_par.nL0_pn = 0.178;
    m_par.sq0_pn = 0.16337;
    m_par.sqe_pn = 0.96524;
    m_par.nL0_p2n = 0.19;
    m_par.sq0_p2n = 0.20355;
    m_par.sqe_p2n = 1.0714;
}

// Do some rough constency checks
bool ChTMeasyTire::CheckParameters() {
    bool isOk = false;

    // Nominal Load set?
    if (m_par.pn < GetTireMaxLoad(0)) {
        GetLog() << "TMsimpleCheckParameters(): Tire Nominal Load Problem!\n";
        return isOk;
    }

    // Stiffness parameters, spring
    if (m_d1 <= 0.0) {
        GetLog() << "TMsimpleCheckParameters(): Tire Vertical Stiffness Problem!\n";
        return isOk;
    }

    // Stiffness parameters, spring
    if (m_par.mu_0 <= 0.0) {
        GetLog() << "TMsimpleCheckParameters(): Friction Coefficien Mu_0 unset!\n";
        return isOk;
    }

    if (m_par.dz <= 0.0) {
        GetLog() << "TMsimpleCheckParameters(): Tire Vertical Damping Problem!\n";
        return isOk;
    }

    isOk = true;

    return isOk;
}

// set tire reference coefficient of friction
void ChTMeasyTire::SetFrictionCoefficient(double coeff) {
    m_par.mu_0 = coeff;
}

// Set Rolling Resistance Coefficients
void ChTMeasyTire::SetRollingResistanceCoefficient(double r_coef) {
    m_rolling_resistance = r_coef;
}

}  // end namespace vehicle
}  // end namespace chrono
