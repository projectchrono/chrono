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
// Template for a TMsimple Tire Model
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor, Rainer Gericke
// =============================================================================
//
// TMsimple tire model.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChLog.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"

namespace chrono {
namespace vehicle {

ChTMsimpleTire::ChTMsimpleTire(const std::string& name)
    : ChForceElementTire(name),
      m_mu(0.8),
      m_mu_0(0.8),
      m_vcoulomb(0.2),
      m_Fx_max1(-1.0),
      m_Fx_inf1(0.0),
      m_dFx0_1(0.0),
      m_Fx_max2(0.0),
      m_Fx_inf2(0.0),
      m_dFx0_2(0.0),
      m_Fy_max1(0.0),
      m_Fy_inf1(0.0),
      m_dFy0_1(0.0),
      m_Fy_max2(0.0),
      m_Fy_inf2(0.0),
      m_dFy0_2(0.0),
      m_time_trans(0.2) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------

void ChTMsimpleTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetTMsimpleParams();

    GenerateWorkParameters();
 }

void ChTMsimpleTire::GenerateWorkParameters() {
    // convert input parameters into work parameters
    
    if(m_Fx_max1 < 0.0) {
        GetLog() << "No Input Parameters loaded!, Tires will not works as expected\n";
        return;
    }
    
    m_ax1 = 2.0*m_Fx_max1-0.5*m_Fx_max2;
    m_ax2 = 0.5*m_Fx_max2-m_Fx_max1;

    m_bx1 = 2.0*m_dFx0_1-0.5*m_dFx0_2;
    m_bx2 = 0.5*m_dFx0_2-m_dFx0_1;
    
    m_cx1 = 2.0*m_Fx_inf1-0.5*m_Fx_inf2;
    m_cx2 = 0.5*m_Fx_inf2-m_Fx_inf1;
    
    m_ay1 = 2.0*m_Fy_max1-0.5*m_Fy_max2;
    m_ay2 = 0.5*m_Fy_max2-m_Fy_max1;

    m_by1 = 2.0*m_dFy0_1-0.5*m_dFy0_2;
    m_by2 = 0.5*m_dFy0_2-m_dFy0_1;

    m_cy1 = 2.0*m_Fy_inf1-0.5*m_Fy_inf2;
    m_cy2 = 0.5*m_Fy_inf2-m_Fy_inf1;
    
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(m_unloaded_radius, m_areaDep);

    //WritePlots("tmsimple_plot.plt", "37x12.5x16.5");
    
    // Initialize contact patch state variables to 0
    m_states.kappa = 0;
    m_states.alpha = 0;
}

void ChTMsimpleTire::Synchronize(double time,
                              const ChTerrain& terrain) {
    m_time = time;
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu;
    m_data.in_contact = DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                             m_width, m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.0f);
    m_mu = mu;

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, m_data.frame);

    double r_stat = m_unloaded_radius - m_data.depth;

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
        m_states.Reff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
        m_states.abs_vx = std::abs(m_data.vel.x());
        m_states.abs_vt = std::abs(wheel_state.omega * m_states.Reff);
        m_states.vsy = m_data.vel.y();
        m_states.vta = m_states.Reff * std::abs(wheel_state.omega) + 0.01;
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.Reff;
        m_states.omega = wheel_state.omega;
        m_states.sx = -m_states.vsx / m_states.vta;
        m_states.sy = -m_states.vsy / m_states.vta;
        m_states.disc_normal = disc_normal;
    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.kappa = 0;
        m_states.alpha = 0;
        m_states.abs_vx = 0;
        m_states.vta = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.sx = 0;
        m_states.sy = 0;
        m_states.Reff = m_unloaded_radius;
        m_states.omega = 0;
        m_states.abs_vt = 0;
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

    const double vnum = 0.1;

    // smoothing interval for My
    const double vx_min = 0.125;
    const double vx_max = 0.5;

    if (m_states.abs_vx != 0) {
        m_states.kappa = -m_states.vsx / (m_states.abs_vx + vnum);
        m_states.alpha = std::atan2(m_states.vsy, m_states.abs_vx);
    } else {
        m_states.kappa = 0;
        m_states.alpha = 0;
    }

    // Now calculate the new force and moment values.
    // Normal force and moment have already been accounted for in Synchronize().
    // See reference for more detail on the calculations
    double Fx_low = 0;
    double Fy_low = 0;
    double My = 0;
    double Mz = 0;
    double Fx0 = 0;
    double Fy0 = 0;
    
    /*
     longitudinal speed <= vblend1: coulomb friction assumed
     longitudinal speed >= vblend2: slip dependent friction assumed
     slip dependent friction calculation avoided <= vblend1
     blending is applied in the interval [vblend1:vblend2]
     */
    double Fz_star = m_data.normal_force*m_mu/m_mu_0;
    Fx0 = Fx_low = tanh(-m_states.vsx/m_vcoulomb)*Fz_star;
    Fy0 = Fy_low = tanh(-m_states.vsy/m_vcoulomb)*Fz_star;
    Fy_low = sqrt(1.0-pow(Fx_low/Fz_star,2.0))*Fy0;
    Fx_low = sqrt(1.0-pow(Fy_low/Fz_star,2.0))*Fx0;

    double Fx_high = 0;     // 'high' speed longitudinal force
    double Fy_high = 0;     // 'high' speed lateral force
    double vblend1 = 3.0;
    double vblend2 = 5.0;
    if(m_states.abs_vx > vblend1) {
        // avoid calculation with noisy slip values
        TMsimplePatchForces(Fx_high, Fy_high, m_states.sx, m_states.sy, m_data.normal_force);
    }

    // Smoothing factor dependend on m_state.abs_vx, allows soft switching of My
    double myStartUp = ChSineStep(m_states.abs_vx, vx_min, 0.0, vx_max, 1.0);
    // Rolling Resistance
    My = -myStartUp * m_rolling_resistance * m_data.normal_force * ChSignum(m_states.omega);

    // Smooth starting transients
    double tr_fact = ChSineStep(m_time, 0, 0, m_time_trans, 1.0);
    double fblend = ChSineStep(m_states.abs_vx, vblend1, 1.0, vblend2, 0.0);
    double Fx = fblend*Fx_low + (1.0-fblend)*Fx_high;
    double Fy = fblend*Fy_low + (1.0-fblend)*Fy_high;
    Fx *= tr_fact;
    Fy *= tr_fact;
    // compile the force and moment vectors so that they can be
    // transformed into the global coordinate system
    m_tireforce.force = ChVector<>(Fx, Fy, m_data.normal_force);
    m_tireforce.moment = ChVector<>(0, My, Mz);
}

// -----------------------------------------------------------------------------

void ChTMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0);
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
    m_wheel->GetSpindle()->AddVisualShape(m_cyl_shape);
}

void ChTMsimpleTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChTMsimpleTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------

void ChTMsimpleTire::TMsimplePatchForces(double& fx, double& fy, double sx, double sy, double fz) {
    fx = 0.0;
    fy = 0.0;
    
    double q1 = fz/m_Fz_nom;
    double q2 = q1*q1;
    
    double Fx_max = m_ax1*q1 + m_ax2*q2;
    double dFx0   = m_bx1*q1 + m_bx2*q2;
    double Fx_inf = m_cx1*q1 + m_cx2*q2;
    
    double Fy_max = m_ay1*q1 + m_ay2*q2;
    double dFy0   = m_by1*q1 + m_by2*q2;
    double Fy_inf = m_cy1*q1 + m_cy2*q2;

    double Kx = Fx_max;
    double Bx = CH_C_PI - asin(Fx_inf/Fx_max);
    double Ax = Kx*Bx/dFx0;

    double Ky = Fy_max;
    double By = CH_C_PI - asin(Fy_inf/Fy_max);
    double Ay = Ky*By/dFy0;

    fx = Kx*sin(Bx*(1.0-exp(-fabs(sx)/Ax))*ChSignum(sx));
    fy = Ky*sin(By*(1.0-exp(-fabs(sy)/Ay))*ChSignum(sy));

    ChVector2<> F;
    F.x() = fx;
    F.y() = fy;
    double m = F.Length();
    if(m > fz) {
        F *= fz*m_mu/(m*m_mu_0);
    }
    fx = F.x();
    fy = F.y();
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

    m_Fz_nom = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);

    m_Cz = tireLoad / defl_max;
    m_Dz = 2.0 * damping_ratio * sqrt(m_Cz * GetTireMass());

    m_rolling_resistance = 0.015;

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_mu_0 = 0.8;

    m_dFx0_1 = 17.6866 * m_Fz_nom;
    m_Fx_max1 = 0.88468 * m_Fz_nom;
    m_Fx_inf1 = 0.54397 * m_Fz_nom;

    m_dFx0_2 = 13.8046 * 2.0 * m_Fz_nom;
    m_Fx_max2 = 0.7479 * 2.0 * m_Fz_nom;
    m_Fx_inf2 = 0.50365 * 2.0 * m_Fz_nom;

    m_dFy0_1 = 5.948 * m_Fz_nom;
    m_Fy_max1 = 0.77253 * m_Fz_nom;
    m_Fy_inf1 = 0.71139 * m_Fz_nom;

    m_dFy0_2 = 5.506 * 2.0 * m_Fz_nom;
    m_Fy_max2 = 0.73048 * 2.0 * m_Fz_nom;
    m_Fy_inf2 = 0.66823 * 2.0 * m_Fz_nom;
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

    m_Fz_nom = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);

    m_width = tireWidth;
    m_unloaded_radius = secth + rimDia / 2.0;
    m_mu_0 = 0.8;

    m_Cz = tireLoad / defl_max;
    m_Dz = 2.0 * damping_ratio * sqrt(m_Cz * GetTireMass());

    m_rolling_resistance = 0.015;

    m_dFx0_1 = 18.6758 * m_Fz_nom;
    m_Fx_max1 = 1.1205 * m_Fz_nom;
    m_Fx_inf1 = 0.8766 * m_Fz_nom;

    m_dFx0_2 = 20.1757 * 2.0 * m_Fz_nom;
    m_Fx_max2 = 1.072 * 2.0 * m_Fz_nom;
    m_Fx_inf2 = 0.8245 * 2.0 * m_Fz_nom;

    m_dFy0_1 = 14.9858 * m_Fz_nom;
    m_Fy_max1 = 1.0084 * m_Fz_nom;
    m_Fy_inf1 = 0.83941 * m_Fz_nom;

    m_dFy0_2 = 10.0505 * 2.0 * m_Fz_nom;
    m_Fy_max2 = 0.90003 * 2.0 * m_Fz_nom;
    m_Fy_inf2 = 0.76782 * 2.0 * m_Fz_nom;
}

void ChTMsimpleTire::WritePlots(const std::string& plFileName, const std::string& plTireFormat) {
    std::ofstream plt(plFileName);
    plt << "$datx << EOF" << std::endl;
    for(int i=-100; i<= 100; i++) {
        double s = double(i)/100.0;
        double K1 = m_Fx_max1;
        double B1 = CH_C_PI - asin(m_Fx_inf1/m_Fx_max1);
        double A1 = K1*B1/m_dFx0_1;
        double Y1 = K1*sin(B1*(1.0-exp(-fabs(s)/A1))*ChSignum(s));
        double K2 = m_Fx_max2;
        double B2 = CH_C_PI - asin(m_Fx_inf2/m_Fx_max2);
        double A2 = K2*B2/m_dFx0_2;
        double Y2 = K2*sin(B2*(1.0-exp(-fabs(s)/A2))*ChSignum(s));
        plt << s << "\t" << Y1 << "\t" << Y2 <<std::endl;
    }
    plt << "EOF" << std::endl;
    plt << "$daty << EOF" << std::endl;
    for(int i=-100; i<= 100; i++) {
        double s = double(i)/100.0;
        double K1 = m_Fy_max1;
        double B1 = CH_C_PI - asin(m_Fy_inf1/m_Fy_max1);
        double A1 = K1*B1/m_dFy0_1;
        double Y1 = K1*sin(B1*(1.0-exp(-fabs(s)/A1))*ChSignum(s));
        double K2 = m_Fy_max2;
        double B2 = CH_C_PI - asin(m_Fy_inf2/m_Fy_max2);
        double A2 = K2*B2/m_dFy0_2;
        double Y2 = K2*sin(B2*(1.0-exp(-fabs(s)/A2))*ChSignum(s));
        plt << s << "\t" << Y1 << "\t" << Y2 <<std::endl;
    }
    plt << "EOF" << std::endl;
    plt << "set title 'TMsimpleTire :" << plTireFormat << "'" << std::endl;
    plt << "set xlabel 'Slip ()'" << std::endl;
    plt << "set ylabel 'Patch Force (N)'" << std::endl;
    plt << "set xrange [-1:1]" << std::endl;
    plt << "plot $datx with lines t 'fx(fznom)', \\" << std::endl;
    plt << " $datx u 1:3 with lines t 'fx(2*fznom)', \\" << std::endl;
    plt << " $daty with dots t 'fy(2*fznom)', \\" << std::endl;
    plt << " $daty u 1:3 with dots t 'fy(2*fznom)'" << std::endl;
    plt.close();
}

}  // end namespace vehicle
}  // end namespace chrono
