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
// Template for the "TMsimple" handling tire model.
//
// Original developer is Prof. em. Wolfgang Hirschberg TU Graz 2009
// The initial paper could not be accessed, but reconstructed from the
// Master Thesis of Eva Heimlich https://diglib.tugraz.at/download.php?id=5990d1fdae37a&location=browse
// The combined forces calculation in Chrono uses the same algorithm as TMeasy.
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

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace chrono {
namespace vehicle {

ChTMsimpleTire::ChTMsimpleTire(const std::string& name)
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

void ChTMsimpleTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetTMsimpleParams();

    // unset bottoming parameters?
    if (m_bottom_radius == 0) {
        m_bottom_radius = m_rim_radius + 0.01;  // consider thickness of the carcass
    }
    if (m_bottom_stiffness == 0) {
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
    m_integration_method = 2;
}

// -----------------------------------------------------------------------------

void ChTMsimpleTire::Synchronize(double time, const ChTerrain& terrain) {
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
        double r_stat = m_unloaded_radius - m_data.depth;
        m_states.omega = wheel_state.omega;
        m_states.R_eff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
        m_states.P_len = 2.0 * sqrt(m_unloaded_radius * m_data.depth);
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
        m_states.P_len = 0;
        m_states.sx = 0;
        m_states.sy = 0;
        m_states.vta = m_vnum;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChTMsimpleTire::Advance(double step) {
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

    TMcombinedForces(Fx, Fy, m_states.sx, m_states.sy, Fz, m_states.muscale);
    Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
    Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

    double Mx = 0;
    double My = 0;
    double Mz = 0;

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

void ChTMsimpleTire::CombinedCoulombForces(double& fx, double& fy, double fz, double muscale) {
    ChVector2<> F;
    F.x() = tanh(-2.0 * m_states.vsx / m_vcoulomb) * fz * muscale;
    F.y() = tanh(-2.0 * m_states.vsy / m_vcoulomb) * fz * muscale;
    if (F.Length() > fz * muscale) {
        F.Normalize();
        F *= fz * muscale;
    }
    fx = F.x();
    fy = F.y();
}

void ChTMsimpleTire::TMcombinedForces(double& fx, double& fy, double sx, double sy, double Fz, double muscale) {
    // fx and fy calculated from combined parameters (original TMeasy)
    // get curve parameters via interpolation
    double q = Fz / m_par.pn;
    double q2 = q * q;
    double Fx_max = m_ax1 * q + m_ax2 * q2;
    double Fy_max = m_ay1 * q + m_ay2 * q2;
    double dFx0 = m_bx1 * q + m_bx2 * q2;
    double dFy0 = m_by1 * q + m_by2 * q2;
    double Fxs = m_cx1 * q + m_cx2 * q2;
    double Fys = m_cy1 * q + m_cy2 * q2;

    double s = hypot(sx, sy);
    double cbeta;
    double sbeta;
    if (m_states.vsx == 0.0 && m_states.omega * m_states.R_eff == 0.0 && s == 0.0) {
        cbeta = 0.5 * sqrt(2.0);
        sbeta = cbeta;
    } else {
        cbeta = sx / s;
        sbeta = sy / s;
    }

    double F_max = hypot(Fx_max * cbeta, Fy_max * sbeta);
    double dF0 = hypot(dFx0 * cbeta, dFy0 * sbeta);
    double Fs = hypot(Fxs * cbeta, Fys * sbeta);

    double Qcrit = Fs / F_max;
    ChClampValue(Qcrit, -1.0, 1.0);
    double K = F_max;
    double B = CH_C_PI - asin(Qcrit);
    double A = K * B / dF0;
    double Fa = K * sin(B * (1.0 - exp(-s / A)));

    fx = Fa * cbeta;
    fy = Fa * sbeta;
}

// -----------------------------------------------------------------------------

double ChTMsimpleTire::GetNormalStiffnessForce(double depth) const {
    double F = depth * m_d1 + depth * depth * m_d2;  // tire force
    double free_depth = m_unloaded_radius - m_bottom_radius;
    if (depth - free_depth > 0) {
        F += (depth - free_depth) * m_bottom_stiffness;  // add bottom contact force
    }
    return F;
}

double ChTMsimpleTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_par.dz * velocity;
}

// -----------------------------------------------------------------------------

void ChTMsimpleTire::SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc) {
    // calculate coefficients from test data
    Eigen::MatrixXd A(defl.size(), 2);
    Eigen::VectorXd b(defl.size());
    Eigen::Vector2d r;
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

double ChTMsimpleTire::GetTireMaxLoad(unsigned int li) {
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

void ChTMsimpleTire::WritePlots(const std::string& plName, const std::string& plTitle) {
    std::ofstream plt(plName);
    plt << "$ForceX << EOD" << std::endl;
    for (int i = 0; i <= 100; i++) {
        double s = double(i) / 100.0;
        double fx1, fy1, fx2, fy2;
        TMcombinedForces(fx1, fy1, s, 0, m_par.pn, 1.0);
        TMcombinedForces(fx2, fy2, s, 0, 2.0 * m_par.pn, 1.0);
        plt << s << "\t" << fx1 << "\t" << fx2 << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "$ForceY << EOD" << std::endl;
    for (int i = 0; i <= 100; i++) {
        double s = double(i) / 100.0;
        double fx1, fy1, fx2, fy2;
        TMcombinedForces(fx1, fy1, 0, s, m_par.pn, 1.0);
        TMcombinedForces(fx2, fy2, 0, s, 2.0 * m_par.pn, 1.0);
        plt << s << "\t" << fy1 << "\t" << fy2 << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'TMsimple: " << plTitle << "'\n";
    plt << "set xlabel 'longitudinal slip sx ()'\n";
    plt << "set ylabel 'longitudinal force Fx (N)'\n";
    plt << "plot $ForceX with lines t 'Fz = " << int(m_par.pn)
        << " N', $ForceX u 1:3 with lines t 'Fz = " << int(m_par.pn * 2) << " N\n";
    plt << "pause -1\n";
    plt << "set xlabel 'lateral slip sy ()'\n";
    plt << "set ylabel 'lateral force Fx (N)'\n";
    plt << "plot $ForceY with lines t 'Fz = " << int(m_par.pn)
        << " N', $ForceY u 1:3 with lines t 'Fz = " << int(m_par.pn * 2) << " N\n";
    plt << "pause -1\n";
    plt.close();
}

// No Data available? Try this to get a working truck tire
void ChTMsimpleTire::GuessTruck80Par(unsigned int li,       // tire load index
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

// No Data available? Try this to get a working truck tire
void ChTMsimpleTire::GuessTruck80Par(double tireLoad,       // tire load force [N]
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
    m_par.dfx0_pn = 19.1677 * m_par.pn;
    m_par.dfx0_p2n = 15.1285 * 2.0 * m_par.pn;
    m_par.fxm_pn = 0.91432 * m_par.pn;
    m_par.fxm_p2n = 0.79538 * 2.0 * m_par.pn;
    m_par.fxs_pn = 0.52273 * m_par.pn;
    m_par.fxs_p2n = 0.48517 * 2.0 * m_par.pn;

    m_par.dfy0_pn = 6.8392 * m_par.pn;
    m_par.dfy0_p2n = 6.4235 * 2.0 * m_par.pn;
    m_par.fym_pn = 0.75948 * m_par.pn;
    m_par.fym_p2n = 0.72714 * 2.0 * m_par.pn;
    m_par.fys_pn = 0.68739 * m_par.pn;
    m_par.fys_p2n = 0.65594 * 2.0 * m_par.pn;

    SetHorizontalCoefficients();
}

void ChTMsimpleTire::SetHorizontalCoefficients() {
    m_ax1 = 2.0 * m_par.fxm_pn - 0.5 * m_par.fxm_p2n;
    m_ax2 = 0.5 * m_par.fxm_p2n - m_par.fxm_pn;

    m_bx1 = 2.0 * m_par.dfx0_pn - 0.5 * m_par.dfx0_p2n;
    m_bx2 = 0.5 * m_par.dfx0_p2n - m_par.dfx0_pn;

    m_cx1 = 2.0 * m_par.fxs_pn - 0.5 * m_par.fxs_p2n;
    m_cx2 = 0.5 * m_par.fxs_p2n - m_par.fxs_pn;

    m_ay1 = 2.0 * m_par.fym_pn - 0.5 * m_par.fym_p2n;
    m_ay2 = 0.5 * m_par.fym_p2n - m_par.fym_pn;

    m_by1 = 2.0 * m_par.dfy0_pn - 0.5 * m_par.dfy0_p2n;
    m_by2 = 0.5 * m_par.dfy0_p2n - m_par.dfy0_pn;

    m_cy1 = 2.0 * m_par.fys_pn - 0.5 * m_par.fys_p2n;
    m_cy2 = 0.5 * m_par.fys_p2n - m_par.fys_pn;
}

void ChTMsimpleTire::GuessPassCar70Par(unsigned int li,       // tire load index
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

void ChTMsimpleTire::GuessPassCar70Par(double tireLoad,       // tire load force [N]
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

    // TMsimple pattern gained from data from Pacejka book
    m_par.dfx0_pn = 19.9774 * m_par.pn;
    m_par.dfx0_p2n = 20.8424 * 2.0 * m_par.pn;
    m_par.fxm_pn = 1.1404 * m_par.pn;
    m_par.fxm_p2n = 1.1045 * 2.0 * m_par.pn;
    m_par.fxs_pn = 0.84482 * m_par.pn;
    m_par.fxs_p2n = 0.82041 * 2.0 * m_par.pn;
    m_par.dfy0_pn = 16.7895 * m_par.pn;
    m_par.dfy0_p2n = 12.9866 * 2.0 * m_par.pn;
    m_par.fym_pn = 1.0107 * m_par.pn;
    m_par.fym_p2n = 0.91902 * 2.0 * m_par.pn;
    m_par.fys_pn = 0.84864 * m_par.pn;
    m_par.fys_p2n = 0.78702 * 2.0 * m_par.pn;

    SetHorizontalCoefficients();
}

// Do some rough constency checks
bool ChTMsimpleTire::CheckParameters() {
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
void ChTMsimpleTire::SetFrictionCoefficient(double coeff) {
    m_par.mu_0 = coeff;
}

// Set Rolling Resistance Coefficients
void ChTMsimpleTire::SetRollingResistanceCoefficient(double r_coef) {
    m_rolling_resistance = r_coef;
}

}  // end namespace vehicle
}  // end namespace chrono
