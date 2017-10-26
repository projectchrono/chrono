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
// Template for the "Tire Model made Easy"
//
// Ref: Georg Rill, "Road Vehicle Dynamics - Fundamentals and Modelling",
//          @2012 CRC Press, ISBN 978-1-4398-3898-3
//      Georg Rill, "An Engineer's Guess On Tyre Model Parameter Mmade Possible With TMeasy",
//          https://hps.hs-regensburg.de/rig39165/Rill_Tyre_Coll_2015.pdf
//
// This implementation does not include transient slip state modifications.
// No parking slip calculations.
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT
//  - Still need to check F&M Outputs
// =============================================================================
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTMeasyTire::ChTMeasyTire(const std::string& name)
    : ChTire(name), m_kappa(0), m_alpha(0), m_gamma(0), m_gamma_limit(3), m_stepsize(1e-6) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTMeasyTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    SetTMeasyParams();

    // Initialize contact patch state variables to 0;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

// -----------------------------------------------------------------------------
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

double ChTMeasyTire::GetNormalStiffnessForce(double depth) {
    return depth * (m_TMeasyCoeff.cz + m_TMeasyCoeff.czq * depth);
}

double ChTMeasyTire::GetNormalDampingForce(double depth, double velocity) {
    return m_TMeasyCoeff.dz * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTMeasyTire::Synchronize(double time, const WheelState& wheel_state, const ChTerrain& terrain) {
    // Invoke the base class function.
    ChTire::Synchronize(time, wheel_state, terrain);

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
            m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
        }

        m_data.normal_force = Fn_mag;
        m_states.R_eff = m_unloaded_radius - m_data.depth;
        m_states.vx = std::abs(m_data.vel.x());
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;
        m_states.vsy = m_data.vel.y();
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
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTMeasyTire::Advance(double step) {
    // Return now if no contact.  Tire force and moment are already set to 0 in Synchronize().
    if (!m_data.in_contact)
        return;

    double vnum = 0.01;  // numerical value to prevent singularities
    double v;            // vector sum of slip velocities
    double vu;           // long. slip velocity
    double vt;           // transport velocity
    double vx;           // x velocity
    double vy;           // y velocity

    double sx;              // longitudinal slip
    double sy;              // lateral slip
    double sc;              // combined slip
    double calpha, salpha;  // cos(alpha) rsp. sin(alpha), alpha = slip angle
    double muscale;         // factor for considering local friction

    muscale = m_mu / m_TMeasyCoeff.mu_0;

    vx = m_data.vel.x();
    vy = m_data.vel.y();
    vt = m_states.omega * m_states.R_eff;
    vu = vx - vt;
    v = hypot(vu, vy);

    sx = -vu / (std::abs(vt) + vnum);
    sy = -vy / (std::abs(vt) + vnum);
    m_states.cp_long_slip = sx;
    m_states.cp_side_slip = atan(sy);

    // Ensure that cp_lon_slip stays between -1 & 1
    ChClampValue(m_states.cp_long_slip, -1.0, 1.0);

    // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
    ChClampValue(m_states.cp_side_slip, -CH_C_PI_2 + 0.001, CH_C_PI_2 - 0.001);

    // Express alpha and gamma in degrees. Express kappa as percentage.
    // m_gamma = 90.0 - std::acos(m_states.disc_normal.z()) * CH_C_RAD_TO_DEG; from Pac89
    m_gamma = GetCamberAngle() * CH_C_RAD_TO_DEG;
    m_alpha = m_states.cp_side_slip * CH_C_RAD_TO_DEG;
    m_kappa = m_states.cp_long_slip * 100.0;

    // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation.
    double gamma = ChClamp(GetCamberAngle(), -m_gamma_limit* CH_C_DEG_TO_RAD, m_gamma_limit* CH_C_DEG_TO_RAD);

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

    double sxn = sx / hsxn;
    double syn = sy / hsyn;

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
    double sb = -rb * m_states.omega * sin(gamma) / (std::abs(vt) + vnum);

    // generalzed slip
    double sg = hypot(sc, sb);

    tmxy_combined(f, fos, sg, df0, sm, fm, ss, fs);
    if (sg > 0.0) {
        Fx = f * sx / sg;
        Fy = f * sy / sg;
    } else {
        Fx = 0.0;
        Fy = 0.0;
    }
    // Calculate dimensionless lever arm
    double levN = tmy_tireoff(sy, nto0, synto0, syntoE);

    // Self Alignment Torque
    double Ms = -plen * levN * Fy;

    // Bore Torque
    double Mb;
    if (sg > 0.0) {
        Mb = rb * f * sb / sg;
    } else {
        Mb = 0.0;
    }
    // Calculate result of alignment torque and bore torque
    Mz = Ms + Mb;

    //   camber slip and force
    double sy_c = -0.5 * plen * m_states.omega * sin(gamma) / (std::abs(vt) + vnum);
    double fy_c = fos / 3.0 * sy_c;

    Fy += fy_c;

    // Overturning Torque
    {
        double c_t = m_TMeasyCoeff.cz + 2.0 * m_TMeasyCoeff.czq * m_data.depth;  // actual vertical tire stiffness
        double cg = std::pow(m_width, 2.0) * c_t / 12.0;
        Mx = -cg * gamma;
    }

    // Rolling Resistance, Ramp Like Signum inhibits 'switching' of My
    {
        double Lrad = (m_unloaded_radius - m_data.depth);
        My = -m_rolling_resistance * m_data.normal_force * Lrad * RampSignum(m_states.omega);
    }

    // std::cout << "Fx:" << Fx
    //    << " Fy:" << Fy
    //    << " Fz:" << Fz
    //    << " Mx:" << Mx
    //    << " My:" << My
    //    << " Mz:" << Mz
    //    << std::endl
    //    << " G:" << gamma
    //    << " A:" << alpha
    //    << " K:" << kappa
    //    << " O:" << m_states.omega
    //    << std::endl;

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    m_tireforce.force = ChVector<>(Fx, Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(Mx, My, Mz);

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

void ChTMeasyTire::VerticalStiffnessByTable(std::vector<double>& defl, std::vector<double>& frc) {
    // polynomial regression of the type y = a*t + b*t^2
    // at least 3 table entries, no identical pairs, no 0,0 pair
    double sat2 = 0.0, sat3 = 0.0, sbt3, sbt4 = 0.0, syt = 0.0, syt2 = 0.0;

    for (int i = 0; i < defl.size(); i++) {
        sat2 += defl[i] * defl[i];
        sat3 += defl[i] * defl[i] * defl[i];
        sbt4 += defl[i] * defl[i] * defl[i] * defl[i];
        syt += frc[i] * defl[i];
        syt2 += frc[i] * defl[i] * defl[i];
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

    std::cout << "Stiffness Function Fz = " << a << "*defl + " << b << "*defl^2" << std::endl;

    // Change Stiffness Model
    m_TMeasyCoeff.cz = a;
    m_TMeasyCoeff.czq = b;
}

double ChTMeasyTire::RampSignum(double x) {
    // Signum with a Ramp to smooth switching states
    double full_at = 0.1;
    double b = 1.0 / full_at;

    if (std::abs(x) < full_at) {
        return b * x;
    }
    if (x > full_at) {
        return 1.0;
    } else {
        return -1.0;
    }
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
    if (m_TMeasyCoeff.mu_0 == 0.0 || m_TMeasyCoeff.cz == 0.0) {
        std::cout << "Tire Object not initialised - nothing to plot!" << std::endl;
        return;
    }
    std::string titleName = std::string("TMEasy Tire ") + this->GetName() + ": ";
    titleName += plTitle;
    std::ofstream plot(plName);
    double plen1 = 2.0 * sqrt(m_unloaded_radius * m_TMeasyCoeff.pn / m_TMeasyCoeff.cz);
    double plen2 = 2.0 * sqrt(m_unloaded_radius * 2.0 * m_TMeasyCoeff.pn / m_TMeasyCoeff.cz);
    int ndat = 201;
    double step = 0.01;
    plot << "a = " << m_TMeasyCoeff.cz << std::endl;
    plot << "b = " << m_TMeasyCoeff.czq << std::endl;
    plot << "f(x) = a*x+b*x*x" << std::endl;
    plot << "$Fx << EOD" << std::endl;
    for (int i = 0; i < ndat; i++) {
        double sx = -1.0 + step * double(i);
        double fx1, fx2, fos;
        tmxy_combined(fx1, fos, std::abs(sx), m_TMeasyCoeff.dfx0_pn, m_TMeasyCoeff.sxm_pn, m_TMeasyCoeff.fxm_pn,
                      m_TMeasyCoeff.sxs_pn, m_TMeasyCoeff.fxs_pn);
        tmxy_combined(fx2, fos, std::abs(sx), m_TMeasyCoeff.dfx0_p2n, m_TMeasyCoeff.sxm_p2n, m_TMeasyCoeff.fxm_p2n,
                      m_TMeasyCoeff.sxs_p2n, m_TMeasyCoeff.fxs_p2n);
        if (sx >= 0.0) {
            plot << sx << "\t" << fx1 << "\t" << fx2 << std::endl;
        } else {
            plot << sx << "\t" << -fx1 << "\t" << -fx2 << std::endl;
        }
    }
    plot << "EOD" << std::endl;

    plot << "$Fy << EOD" << std::endl;
    for (int i = 0; i < ndat; i++) {
        double sy = -1.0 + step * double(i);
        double fy1, fy2, fos;
        double mz1, mz2, lev1, lev2;
        tmxy_combined(fy1, fos, std::abs(sy), m_TMeasyCoeff.dfy0_pn, m_TMeasyCoeff.sym_pn, m_TMeasyCoeff.fym_pn,
                      m_TMeasyCoeff.sys_pn, m_TMeasyCoeff.fys_pn);
        tmxy_combined(fy2, fos, std::abs(sy), m_TMeasyCoeff.dfy0_p2n, m_TMeasyCoeff.sym_p2n, m_TMeasyCoeff.fym_p2n,
                      m_TMeasyCoeff.sys_p2n, m_TMeasyCoeff.fys_p2n);
        lev1 = tmy_tireoff(std::abs(sy), m_TMeasyCoeff.nto0_pn, m_TMeasyCoeff.synto0_pn, m_TMeasyCoeff.syntoE_pn);
        lev2 = tmy_tireoff(std::abs(sy), m_TMeasyCoeff.nto0_p2n, m_TMeasyCoeff.synto0_p2n, m_TMeasyCoeff.syntoE_p2n);
        if (sy >= 0.0) {
            mz1 = -plen1 * lev1 * fy1;
            mz2 = -plen2 * lev2 * fy2;
            plot << sy << "\t" << fy1 << "\t" << fy2 << "\t" << mz1 << "\t" << mz2 << std::endl;
        } else {
            mz1 = plen1 * lev1 * fy1;
            mz2 = plen2 * lev2 * fy2;
            plot << sy << "\t" << -fy1 << "\t" << -fy2 << "\t" << mz1 << "\t" << mz2 << std::endl;
        }
    }
    plot << "EOD" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Vertcal Tire Deflection z []'" << std::endl;
    plot << "set ylabel 'Vertical Force Fz [N]'" << std::endl;
    plot << "set xrange [0:0.1]" << std::endl;
    plot << "plot f(x) notitle wit lines" << std::endl;
    plot << "pause -1" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Longitudinal Slip sx []'" << std::endl;
    plot << "set ylabel 'Longitudinal Force Fx [N]'" << std::endl;
    plot << "set xrange [-1:1]" << std::endl;
    plot << "plot $Fx title 'Fz = " << m_TMeasyCoeff.pn
         << " N' with lines, $Fx u 1:3 title 'Fz = " << (2.0 * m_TMeasyCoeff.pn) << " N' with lines" << std::endl;
    plot << "pause -1" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Lateral Slip sy []'" << std::endl;
    plot << "set ylabel 'Lateral Force Fy [N]'" << std::endl;
    plot << "plot $Fy title 'Fz = " << m_TMeasyCoeff.pn
         << " N' with lines, $Fy u 1:3 title 'Fz = " << (2.0 * m_TMeasyCoeff.pn) << " N' with lines" << std::endl;
    plot << "pause -1" << std::endl;
    plot << "set title '" << titleName.c_str() << "'" << std::endl;
    plot << "set xlabel 'Lateral Slip sy []'" << std::endl;
    plot << "set ylabel 'Self Aligning Torque Mz [Nm]'" << std::endl;
    plot << "plot $Fy u 1:4 title 'Fz = " << m_TMeasyCoeff.pn
         << " N' with lines, $Fy u 1:5 title 'Fz = " << (2.0 * m_TMeasyCoeff.pn) << " N' with lines" << std::endl;
    plot << "pause -1" << std::endl;
    plot.close();
}

// No Data available? Try this to get a working truck tire
void ChTMeasyTire::GuessTruck80Par(unsigned int li,   // tire load index
                                   double tireWidth,  // [m]
                                   double ratio,      // [] = use 0.75 meaning 75%
                                   double rimDia,     // rim diameter [m]
                                   double pinfl_li,   // inflation pressure at load index
                                   double pinfl_use   // inflation pressure in this configuration
) {
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload
    double xi = 0.05;                  // damping ration

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_rolling_resistance = 0.015;
    m_TMeasyCoeff.mu_0 = 0.8;

    m_TMeasyCoeff.pn = 0.5 * GetTireMaxLoad(li) * pow(pinfl_use / pinfl_li, 0.8);
    m_TMeasyCoeff.cz = 2.0 * m_TMeasyCoeff.pn / defl_max;
    m_TMeasyCoeff.czq = 0.0;  // linear function assumed
    m_TMeasyCoeff.dz = 2.0 * xi * sqrt(m_TMeasyCoeff.cz * GetMass());

    m_TMeasyCoeff.dfx0_pn = 17.6866 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxm_pn = 0.12;
    m_TMeasyCoeff.fxm_pn = 0.88468 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxs_pn = 0.9;
    m_TMeasyCoeff.fxs_pn = 0.54397 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfx0_p2n = 13.8046 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxm_p2n = 0.15;
    m_TMeasyCoeff.fxm_p2n = 0.7479 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxs_p2n = 0.95;
    m_TMeasyCoeff.fxs_p2n = 0.50365 * 2.0 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfy0_pn = 5.948 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sym_pn = 0.38786;
    m_TMeasyCoeff.fym_pn = 0.77253 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sys_pn = 0.82534;
    m_TMeasyCoeff.fys_pn = 0.71139 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfy0_p2n = 5.506 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sym_p2n = 0.38786;
    m_TMeasyCoeff.fym_p2n = 0.73048 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sys_p2n = 0.91309;
    m_TMeasyCoeff.fys_p2n = 0.66823 * 2.0 * m_TMeasyCoeff.pn;

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
                                     double pinfl_use   // inflation pressure in this configuration
) {
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload
    double xi = 0.05;                  // damping ration

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_rolling_resistance = 0.015;
    m_TMeasyCoeff.mu_0 = 0.8;

    m_TMeasyCoeff.pn = 0.5 * GetTireMaxLoad(li) * pow(pinfl_use / pinfl_li, 0.8);
    m_TMeasyCoeff.cz = 2.0 * m_TMeasyCoeff.pn / defl_max;
    m_TMeasyCoeff.czq = 0.0;  // linear function assumed
    m_TMeasyCoeff.dz = 2.0 * xi * sqrt(m_TMeasyCoeff.cz * GetMass());

    m_TMeasyCoeff.dfx0_pn = 18.6758 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxm_pn = 0.17;
    m_TMeasyCoeff.fxm_pn = 1.1205 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxs_pn = 0.9;
    m_TMeasyCoeff.fxs_pn = 0.8766 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfx0_p2n = 20.1757 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxm_p2n = 0.15;
    m_TMeasyCoeff.fxm_p2n = 1.072 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sxs_p2n = 0.95;
    m_TMeasyCoeff.fxs_p2n = 0.8245 * 2.0 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfy0_pn = 14.9858 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sym_pn = 0.18197;
    m_TMeasyCoeff.fym_pn = 1.0084 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sys_pn = 0.82534;
    m_TMeasyCoeff.fys_pn = 0.83941 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.dfy0_p2n = 10.0505 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sym_p2n = 0.24472;
    m_TMeasyCoeff.fym_p2n = 0.90003 * 2.0 * m_TMeasyCoeff.pn;
    m_TMeasyCoeff.sys_p2n = 0.91309;
    m_TMeasyCoeff.fys_p2n = 0.76782 * 2.0 * m_TMeasyCoeff.pn;

    m_TMeasyCoeff.nto0_pn = 0.178;
    m_TMeasyCoeff.synto0_pn = 0.19107;
    m_TMeasyCoeff.syntoE_pn = 0.82534;

    m_TMeasyCoeff.nto0_p2n = 0.19;
    m_TMeasyCoeff.synto0_p2n = 0.25695;
    m_TMeasyCoeff.syntoE_p2n = 0.91309;
}

}  // end namespace vehicle
}  // end namespace chrono
