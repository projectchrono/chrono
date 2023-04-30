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
// Authors: Marvin Struijk, Rainer Gericke
// =============================================================================
//
// Template for a tire model based on the Pacejka MF5.2-6.2 tire model
// Implementation is based on: https://mfeval.wordpress.com/usingmfeval/
//
// =============================================================================
// UNDER DEVELOPMENT
// =============================================================================

// TODO: List of development items not fully implemented/wip
// [ ]  Turnslip
// [ ]  Implement bottoming model (rim stiffness) in Fz calculation

#include <algorithm>
#include <cmath>

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTire.h"

namespace chrono {
namespace vehicle {

ChMFTire::ChMFTire(const std::string& name)
    : ChForceElementTire(name),
      m_kappa(0),
      m_alpha(0),
      m_gamma(0),
      m_gamma_limit(3.0 * CH_C_DEG_TO_RAD),
      // m_use_friction_ellipsis(true),
      m_mu(0),
      // m_Shf(0),
      // m_measured_side(LEFT),
      // m_allow_mirroring(false),
      m_use_mode(4) {
    m_tireforce.force = ChVector<>(0, 0, 0);
    m_tireforce.point = ChVector<>(0, 0, 0);
    m_tireforce.moment = ChVector<>(0, 0, 0);
}

// -----------------------------------------------------------------------------

void ChMFTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    SetMFParams();
    // Build the lookup table for penetration depth as function of intersection area
    // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
    ConstructAreaDepthTable(par.UNLOADED_RADIUS, m_areaDep);

    // all parameters are known now pepare mirroring
    // if (m_allow_mirroring) {
    if (wheel->GetSide() != par.TYRESIDE) {  // m_measured_side) {
        // we flip the sign of some parameters
        flipSide = true;

        if (par.TYRESIDE == LEFT) {
            GetLog() << "Tire is measured as left tire but mounted on the right vehicle side -> mirroring.\n";
        } else {
            GetLog() << "Tire is measured as right tire but mounted on the lleft vehicle side -> mirroring.\n";
        }
    }
    // }

    // Initialize contact patch state variables to 0
    m_data.normal_force = 0;
    m_states.R_eff = par.UNLOADED_RADIUS;
    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
    m_states.vx = 0;
    m_states.vsx = 0;
    m_states.vsy = 0;
    m_states.omega = 0;
    m_states.disc_normal = ChVector<>(0, 0, 0);
    m_states.p = 0.0;

    m_states.cp_long_slip = 0;
    m_states.cp_side_slip = 0;
}

double ChMFTire::CalculatePressureDifference(double p = 0.0) {
    // Inflation pressure
    if (p == 0.0) {
        p = par.INFLPRES;
    } else {
        // Check pressure limits
        auto isLowPressure = p < par.PRESMIN;
        p = isLowPressure ? par.PRESMIN : p;

        auto isHighPressure = p > par.PRESMAX;
        p = isHighPressure ? par.PRESMAX : p;
    }

    m_states.p = p;

    m_dpi = (p - par.NOMPRES) / par.NOMPRES;

    return m_dpi;
}

double ChMFTire::CalculateEffectiveRollingRadius(double omega, double dpi, double Fz) {
    const double epsilon = 1.0e-6;
    omega = abs(omega) + epsilon;  // rotational speed(rad / s)

    // Rename the TIR file variables in the Pacejka style
    auto Fz0 = par.FNOMIN;              // Nominal(rated) wheel load
    auto R0 = par.UNLOADED_RADIUS;      // Free tyre radius
    auto V0 = par.LONGVL;               // Nominal speed(LONGVL)
    auto Cz0 = par.VERTICAL_STIFFNESS;  // Vertical stiffness
    auto PFZ1 = par.PFZ1;               // Pressure effect on vertical stiffness
    auto BREFF = par.BREFF;             // Low load stiffness effective rolling radius
    auto DREFF = par.DREFF;             // Peak value of effective rolling radius
    auto FREFF = par.FREFF;             // High load stiffness effective rolling radius
    auto Q_RE0 = par.Q_RE0;             // Ratio of free tyre radius with nominal tyre radius
    auto Q_V1 = par.Q_V1;               // Tyre radius increase with speed

    // Excerpt from OpenTIRE MF6.1 implementation
    // Date: 2016 - 12 - 01
    // Prepared for Marco Furlan / JLR
    // Questions : henning.olsson@calspan.com

    // Nominal stiffness(pressure corrected)
    auto Cz =
        Cz0 * (1.0 + PFZ1 * dpi);  // [Eqn(5) Page 2 - Paper] - Vertical stiffness adapted for tyre inflation pressure
    auto Romega = R0 * (Q_RE0 + Q_V1 * pow((omega * R0) / V0,
                                           2));  // [Eqn(1) Page 2 - Paper] - Centrifugal growth of the free tyre radius

    // Eff.Roll.Radius
    auto Re = Romega - (Fz0 / Cz) * (DREFF * atan(BREFF * (Fz / Fz0)) + FREFF * (Fz / Fz0));  // [Eqn(7) Page 2 - Paper]

    return Re;
}

double ChMFTire::CalculateGamma(bool flip = false) {
    m_gamma = CH_C_PI_2 - std::acos(m_states_in.disc_normal.z());

    // Use gamma_star = sin(gamma) to support large gamma angles
    if (useStarInputs)
        m_gamma = sin(m_gamma);

    if (flip)
        m_gamma = -m_gamma;

    // Check camber limits
    if (m_gamma < par.CAMMIN) {
        m_gamma = par.CAMMIN;
    } else if (m_gamma > par.CAMMAX) {
        m_gamma = par.CAMMAX;
    }

    /*
    auto isLowCamber = m_gamma < par.CAMMIN;
    m_gamma = isLowCamber ? par.CAMMIN : m_gamma;

    auto isHighCamber = m_gamma > par.CAMMAX;
    m_gamma = isHighCamber ? par.CAMMAX : m_gamma;
    */

    return m_gamma;
}

double ChMFTire::CalculateFz(double depth, double omega, double gamma, double p, double Fx, double Fy) {
    const double epsilon = 1.0e-6;

    // TODO: Implement bottoming model

    // Calculate norm. pressure difference
    m_dpi = CalculatePressureDifference(m_states.p);

    auto dpi = m_dpi;

    omega = abs(omega) + epsilon;  // rotational speed(rad / s)

    // [MODEL]
    auto V0 = par.LONGVL;  // Nominal speed

    // [VERTICAL]
    auto Fz0 = par.FNOMIN;              // Nominal wheel load
    auto Cz0 = par.VERTICAL_STIFFNESS;  // Tyre vertical stiffness
    auto Q_V2 = par.Q_V2;               // Vertical stiffness increase with speed
    auto Q_FZ2 = par.Q_FZ2;             // Quadratic term in load vs.deflection
    auto Q_FCX = par.Q_FCX;             // Longitudinal force influence on vertical stiffness
    auto Q_FCY = par.Q_FCY;             // Lateral force influence on vertical stiffness
    auto PFZ1 = par.PFZ1;               // Pressure effect on vertical stiffness
    auto Q_FCY2 =
        par.Q_FCY2;  // Explicit load dependency for including the lateral force influence on vertical stiffness
    auto Q_CAM1 = par.Q_CAM1;  // Linear load dependent camber angle influence on vertical stiffness
    auto Q_CAM2 = par.Q_CAM2;  // Quadratic load dependent camber angle influence on vertical stiffness
    auto Q_CAM3 = par.Q_CAM3;  // Linear loadand camber angle dependent reduction on vertical stiffness
    auto Q_FYS1 = par.Q_FYS1;  // Combined camber angleand side slip angle effect on vertical stiffness(constant)
    auto Q_FYS2 = par.Q_FYS2;  // Combined camber angleand side slip angle linear effect on vertical stiffness
    auto Q_FYS3 = par.Q_FYS3;  // Combined camber angleand side slip angle quadratic effect on vertical stiffness
    auto Q_RE0 = par.Q_RE0;    // Ratio of free tyre radius with nominal tyre radius
    auto Q_V1 = par.Q_V1;      // Tyre radius increase with speed

    auto ASPECT_RATIO = par.ASPECT_RATIO;
    auto WIDTH = par.WIDTH;

    // [DIMENSION]
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius

    // Model parameters as QFZ1 that normally aren't present in the TIR files
    auto Q_FZ1 = sqrt(pow(Cz0 * R0 / Fz0, 2) - 4.0 * Q_FZ2);  // Rearranging[Eqn(4) Page 2 - Paper]

    double Fz;

    // Calculate loaded radius from depth (slightly redunandant, but fine for now)
    double rl = R0 - depth;

    // Check MF version
    if (par.FITTYP == 6 || par.FITTYP == 21) {  // MF5.2
        // Split Eqn(A3.3) Page 619 of the Book into different bits :
        auto speed_effect = Q_V2 * (R0 / V0) * abs(omega);
        auto fx_effect = pow(Q_FCX * Fx / Fz0, 2);
        auto fy_effect = pow(Q_FCY * Fy / Fz0, 2);
        auto pressure_effect = (1.0 + PFZ1 * dpi);

        // Joining all the effects except tyre deflection terms :
        auto external_effects = (1.0 + speed_effect - fx_effect - fy_effect) * pressure_effect * Fz0;

        // Calculate deflection
        auto rho = fmax(R0 - rl, 0.0);

        // Equation(A3.3) can be written as :
        Fz = (Q_FZ2 * pow(rho / R0, 2) + Q_FZ1 * (rho / R0)) * external_effects;
    } else {  // MF6.2
        // Then we calculate free - spinning radius
        auto Romega =
            R0 * (Q_RE0 + Q_V1 * pow((omega * R0) / V0,
                                     2));  // [Eqn(1) Page 2 - Paper] - Centrifugal growth of the free tyre radius

        // Model parameters as QFZ1 that normally aren't present in the TIR files
        // Q_FZ1 = sqrt((Cz0.*R0. / Fz0). ^ 2 - 4. * Q_FZ2); // Rearranging[Eqn(4) Page 2 - Paper]

        // Asymmetric effect for combinations of camberand lateral force
        auto Sfyg = (Q_FYS1 + Q_FYS2 * (rl / Romega) + Q_FYS3 * pow(rl / Romega, 2)) * gamma;

        // Tyre deflection for a free rolling tyre
        auto rho_zfr = fmax(Romega - rl, 0.0);

        // Reference tread width
        auto rtw = (1.075 - 0.5 * ASPECT_RATIO) * WIDTH;

        // Deflection caused by camber
        auto rho_zg = pow((Q_CAM1 * rl + Q_CAM2 * pow(rl, 2)) * gamma, 2) * (rtw / 8.0) * abs(tan(gamma)) /
                          fmax(pow((Q_CAM1 * Romega + Q_CAM2 * pow(Romega, 2)) * gamma, 2), 1.0e-9) -
                      (Q_CAM3 * rho_zfr * abs(gamma));

        // Change NaN to Zero
        // rho_zg(isnan(rho_zg)) = 0;

        // Vertical deflection
        auto rho_z = fmax(rho_zfr + rho_zg, 1.0e-12);

        // Correction term
        auto fcorr = (1.0 + Q_V2 * (R0 / V0) * abs(omega) - pow((Q_FCX * Fx) / Fz0, 2) -
                      pow(pow(rho_z / R0, Q_FCY2) * (Q_FCY * (Fy - Sfyg) / Fz0), 2)) *
                     (1.0 + PFZ1 * dpi);

        // std::cout << fcorr << rho_z << R0 << Q_FZ1 << rho_zfr << rho_zg << Romega << Fx << Fy << std::endl;
        /*
                fcorr = (1.0 + Q_V2 * (R0 / V0) * abs(omega) - pow((Q_FCX * 0.0) / Fz0, 2) -
                 pow(pow(rho_z / R0, Q_FCY2) * (Q_FCY * (0.0 - Sfyg) / Fz0), 2)) *
                (1.0 + PFZ1 * dpi);
                */
        // Vertical force
        Fz = fcorr * (Q_FZ1 + Q_FZ2 * (rho_z / R0)) * (rho_z / R0) * Fz0;
    }

    // TODO: Implement damping according to MFTire model
    Fz += (-m_data.vel.z() * par.VERTICAL_DAMPING);  // GetNormalDampingForce(m_data.depth, -m_data.vel.z());

    // No negative Fz possible
    if (Fz < 0.0) {
        m_data.in_contact = false;
        Fz = 0.0;
    }

    // This is maybe required to assure we don't fall through the ground.
    // The "official" model states that Fz should be clipped to Fz max.
    // We probably want to implement an option to extrapolate or not...

    // Check Fz limits
    m_fz_unlim = Fz;
    if (Fz > par.FZMAX) {
        Fz = par.FZMAX;
    } else if (Fz < par.FZMIN) {
        Fz = par.FZMIN;
    }

    // Calculate fz scaling for low load
    m_fz_scl = m_fz_unlim / Fz;

    // Clip on extrapolation
    m_fz_scl = fmin(m_fz_scl, 1.0);

    // Effect of having a tire with a different nominal load
    auto Fz0_prime = par.LFZO * par.FNOMIN;  // [Eqn(4.E1) Page 177 - Book]

    // Normalized change in vertical load
    m_dfz = (Fz - Fz0_prime) / Fz0_prime;  // [Eqn(4.E2a) Page 177 - Book]

    // Fz used for model evaluation should be limited within [FZMIN, FZMAX]
    m_fz = Fz;

    // Normal force should not be limited (at least on the lower bound...)
    return m_fz_unlim;
}

void ChMFTire::Synchronize(double time, const ChTerrain& terrain) {
    /*
        NOTE: To flip the side of the tire, we need to invert vy and gamma on the input.
        On the output side, we would need to mirror Fy, Mz and Mx
    */
    WheelState wheel_state = m_wheel->GetState();

    // Extract the wheel normal (expressed in global frame)
    ChMatrix33<> A(wheel_state.rot);
    ChVector<> disc_normal = A.Get_A_Yaxis();

    // Assuming the tire is a disc, check contact with terrain
    float mu;

    auto R0 = par.UNLOADED_RADIUS;
    auto cp_width = par.WIDTH;  // TODO: Get from TIR file or compute dynamically?

    m_data.in_contact = DiscTerrainCollision(m_collision_type, terrain, wheel_state.pos, disc_normal, R0, cp_width,
                                             m_areaDep, m_data.frame, m_data.depth, mu);
    ChClampValue(mu, 0.1f, 1.5f);
    m_mu = mu;

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

        // double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z());

        // TODO: Validate this part, perhaps a good idea to saturate or filter these.
        // auto Fx = m_fx_fb;
        // auto Fy = m_fy_fb;
        auto gamma = CalculateGamma(flipSide);

        // We probably want to use an unbounded or more loosely bounded gamma here...
        m_data.normal_force = CalculateFz(m_data.depth, m_states.omega, gamma, m_states.p, m_fx_fb, m_fy_fb);
        // m_data.normal_force = Fn_mag;
        m_states.R_eff = CalculateEffectiveRollingRadius(m_states.omega, m_dpi, m_fz);  // m_PacCoeff.R0 - m_data.depth;
        // m_states.vx = std::abs(m_data.vel.x());
        m_states.vx = m_data.vel.x();
        m_states.vsx = m_data.vel.x() - wheel_state.omega * m_states.R_eff;

        if (useIsoRef)
            m_states.vsy = m_data.vel.y();  // Use standard ISO instead of modified SAE
        else
            m_states.vsy = -m_data.vel.y();  // PAC89 is defined in a modified SAE coordinate system

        if (flipSide)
            m_states.vsy = -m_states.vsy;
        m_states.omega = wheel_state.omega;
        m_states.disc_normal = disc_normal;

        // Calculate friction
        // auto vs = sqrt(fmax(m_states.vsx * m_states.vsx + m_states.vsy * m_states.vsy, 1.0e-6));
        auto vs = 0.0;  // Friction decay with speed is not implemented anyway...
        CalculateFrictionScaling(m_mu, vs);

    } else {
        // Reset all states if the tire comes off the ground.
        m_data.normal_force = 0;
        m_states.R_eff = R0;
        m_states.cp_long_slip = 0;
        m_states.cp_side_slip = 0;
        m_states.vx = 0;
        m_states.vsx = 0;
        m_states.vsy = 0;
        m_states.omega = 0;
        m_states.disc_normal = ChVector<>(0, 0, 0);
    }
}

void ChMFTire::CopyContactData() {
    m_data_in.in_contact = m_data.in_contact;
    m_data_in.frame = m_data.frame;
    m_data_in.vel = m_data.vel;
    m_data_in.normal_force = m_data.normal_force;
    m_data_in.depth = m_data.depth;
}

void ChMFTire::CopyTireStates() {
    // TireStates m_states_out = {};
    m_states_in.cp_long_slip = m_states.cp_long_slip;
    m_states_in.cp_side_slip = m_states.cp_side_slip;
    m_states_in.vx = m_states.vx;
    m_states_in.vsx = m_states.vsx;
    m_states_in.vsy = m_states.vsy;
    m_states_in.omega = m_states.omega;
    m_states_in.R_eff = m_states.R_eff;
    m_states_in.disc_normal = m_states.disc_normal;
    m_states_in.p = m_states.p;
}

void ChMFTire::CalculateRelaxationLength(double Fz, double gamma, double dfz, double dpi) {
    // Unpack Parameters
    /*
    gamma = postProInputs.gamma;
    Fz = postProInputs.Fz;
    p = postProInputs.p;
    Kxk = varinf.Kxk;
    Kya = varinf.Kya;
    */

    auto Kxk = m_Kxk;
    auto Kya = m_Kya;

    auto PCFX1 = par.PCFX1;         // Tyre overall longitudinal stiffness vertical deflection dependency linear term
    auto PCFX2 = par.PCFX2;         // Tyre overall longitudinal stiffness vertical deflection dependency quadratic term
    auto PCFX3 = par.PCFX3;         // Tyre overall longitudinal stiffness pressure dependency
    auto PCFY1 = par.PCFY1;         // Tyre overall lateral stiffness vertical deflection dependency linear term
    auto PCFY2 = par.PCFY2;         // Tyre overall lateral stiffness vertical deflection dependency quadratic term
    auto PCFY3 = par.PCFY3;         // Tyre overall lateral stiffness pressure dependency
    auto LFZO = par.LFZO;           // Scale factor of nominal(rated) load
    auto Fz0 = par.FNOMIN;          // Nominal(rated) wheel load
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius
    // auto pi0 = par.NOMPRES;                 // Reference pressure
    auto cx0 = par.LONGITUDINAL_STIFFNESS;  // Tyre overall longitudinal stiffness
    auto cy0 = par.LATERAL_STIFFNESS;       // Tyre overall lateral stiffness
    auto PTX1 = par.PTX1;                   // Relaxation length SigKap0 / Fz at Fznom
    auto PTX2 = par.PTX2;                   // Variation of SigKap0 / Fz with load
    auto PTX3 = par.PTX3;                   // Variation of SigKap0 / Fz with exponent of load
    auto PTY1 = par.PTY1;                   // Peak value of relaxation length SigAlp0 / R0
    auto PTY2 = par.PTY2;                   // Value of Fz / Fznom where SigAlp0 is extreme
    auto LSGKP = par.LSGKP;                 // Scale factor of Relaxation length of Fx
    auto LSGAL = par.LSGAL;                 // Scale factor of Relaxation length of Fy
    auto PKY3 = par.PKY3;                   //  Variation of Kfy/Fznom with camber

    //  Basic calculations
    auto Fz0_prime = LFZO * Fz0;

    // Overall longitudinal Cx and lateral stiffness Cy
    auto Cx = cx0 * (1.0 + (PCFX1 + PCFX2 * dfz) * dfz) * (1.0 + PCFX3 * dpi);  // (Eqn 17 - Paper)
    auto Cy = cy0 * (1.0 + (PCFY1 + PCFY2 * dfz) * dfz) * (1.0 + PCFY3 * dpi);  // (Eqn 18 - Paper)

    auto sigmax = 0.0;
    auto sigmay = 0.0;

    // Check MF version
    if (par.FITTYP == 6 || par.FITTYP == 21)  // Load MF5 .2 parameters
    {
        // MF 5.2 equation for the longitudinal relaxation length
        sigmax = (PTX1 + PTX2 * dfz) * exp(-PTX3 * dfz) * LSGKP * R0 * Fz / Fz0;  // From the MF-Tyre equation manual

        // MF 5.2 equations for the lateral relaxation length
        auto sigmayg = 1.0 - PKY3 * abs(gamma);
        auto PTYfzn = PTY2 * Fz0_prime;
        sigmay = PTY1 * sin(2.0 * atan(Fz / PTYfzn)) * sigmayg * R0 * LFZO * LSGAL;  // From the MF-Tyre equation manual
    } else {
        //  MF 6.1 and 6.2 equations for the relaxation lengths
        sigmax = abs(Kxk / Cx);  // (Eqn 19 - Paper)
        sigmay = abs(Kya / Cy);  // (Eqn 19 - Paper)
    }

    // Assign to class variables
    m_sigma_x = sigmax;
    m_sigma_y = sigmay;

    // m_cx = Cx;
    // m_cy = Cy;

    // std::cout << "sigx: " << sigmax << " sigy: " << sigmay << std::endl;
}

void ChMFTire::CalculateFrictionScaling(double mu, double Vs) {
    auto V0 = par.LONGVL;  // Nominal speed
    auto LMUX = par.LMUX;  // Scale factor of Fx peak friction coefficient
    auto LMUY = par.LMUY;  // Scale factor of Fy peak friction coefficient
    auto LMUV = 0.0;       // Scale factor of friction coefficient with speed (= 0)

    // Calculate nominal friction scaling (i.e. road_mu * nominal_mu)
    auto muscl = mu * m_mu0;

    // Slippery surface with friction decaying with increasing(slip) speed
    LMUX_star = muscl * LMUX / (1.0 + LMUV * Vs / V0);  // [Eqn(4.E7) Page 179 - Book]
    LMUY_star = muscl * LMUY / (1.0 + LMUV * Vs / V0);  // [Eqn(4.E7) Page 179 - Book]

    // Digressive friction factor
    // On Page 179 of the book is suggested Amu = 10, but after
    // comparing the use of the scaling factors against TNO, Amu = 1
    // was giving perfect match
    auto Amu = 1.0;
    LMUX_prime = Amu * LMUX_star / (1.0 + (Amu - 1.0) * LMUX_star);  // [Eqn(4.E8) Page 179 - Book]
    LMUY_prime = Amu * LMUY_star / (1.0 + (Amu - 1.0) * LMUY_star);  // [Eqn(4.E8) Page 179 - Book]
}

double ChMFTire::ContactPatch(double dpi, double Fz) {
    // Unpack Parameters
    auto Fz_unlimited = Fz * m_fz_scl;

    // Rename the TIR file variables in the Pacejka style
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius
    // auto w = par.WIDTH;                 // Nominal width of the tyre
    auto Cz0 = par.VERTICAL_STIFFNESS;  // Vertical stiffness

    auto PFZ1 = par.PFZ1;  // Pressure effect on vertical stiffness

    // Nominal stiffness(pressure corrected)
    auto NCz =
        Cz0 * (1.0 + PFZ1 * dpi);  // [Eqn (5) Page 2 - Paper] - Vertical stiffness adapted for tyre inflation pressure

    // [CONTACT_PATCH]
    auto Q_A1 = par.Q_A1;    // MF5.2 Square root load term in contact length
    auto Q_A2 = par.Q_A2;    // MF5.2 Linear load term in contact length
    auto Q_RA1 = par.Q_RA1;  // Square root term in contact length equation
    auto Q_RA2 = par.Q_RA2;  // Linear term in contact length equation
    // auto Q_RB1 = par.Q_RB1;             // Root term in contact width equation
    // auto Q_RB2 = par.Q_RB2;             // Linear term in contact width equation

    // [DIMENSION]
    auto Rrim = par.RIM_RADIUS;  // Nominal rim radius

    // [VERTICAL]
    auto Fz0 = par.FNOMIN;         // Nominal wheel load
    auto Dbtm = par.BOTTOM_OFFST;  // Distance to rim when bottoming starts to occur

    // Approximated loaded Radius
    auto Rl = R0 - (Fz_unlimited / NCz);

    // Bottoming model(Empirically discovered) :
    // Check if bottoming has happened
    auto isBottoming = Rl - (Rrim + Dbtm) < 0.0;

    // Calculate the max Fz if bottoming happens to calculate the
    // contact patch
    auto maxFz = (R0 - Rrim - Dbtm) * NCz;

    double a = 0.0;
    // double b = 0.0;

    // Substitute max Fz for the calculations
    Fz_unlimited = isBottoming ? maxFz : Fz_unlimited;
    // Fz_unlimited(isBottoming) = real(maxFz(isBottoming));

    // Check MF version
    if (par.FITTYP == 6 || par.FITTYP == 21) {
        // Load MF5 .2 parameters
        if (Q_A1 == 0.0 && Q_A2 == 0.0) {
            // Set default vaules(Empirically discovered)
            auto y = log10(R0 * (Cz0 / Fz0));
            // Q_A1 = -0.0388 * y ^ 3 + 0.2509 * y ^ 2 + -0.6283 * y + 0.6279;
            Q_A1 = ((-0.0388 * y + 0.2509) * y + -0.6283) * y + 0.6279;
            Q_A2 = 1.693 * Q_A1 * Q_A1;
        }
        a = R0 * (Q_A2 * (Fz_unlimited / Fz0) + Q_A1 * sqrt(Fz_unlimited / Fz0));  // From the MF - Tyre equation manual
        // b = w / 2;                                                                 // From the MF - Tyre equation
        // manual
    } else {
        // MF6.1 and 6.2 equatons
        a = R0 *
            (Q_RA2 * (Fz_unlimited / (NCz * R0)) + Q_RA1 * sqrt(Fz_unlimited / (NCz * R0)));  // [Eqn(9) Page 3 - Paper]
        // b = w * (Q_RB2 * (Fz_unlimited / (NCz * R0)) + Q_RB1 * (Fz_unlimited / (NCz * R0)).^ (1.0 / 3.0));  //
        // [Eqn(10) Page 3 - Paper]
    }

    // We don't really need b right now, maybe in the future.

    return a;
}

void ChMFTire::TransientSlipNonlinear(double step,
                                      double vx,
                                      double vsx,
                                      double vsy,
                                      double Fx,
                                      double Fy,
                                      double Fz,
                                      double dpi,
                                      double dfz) {
    // This part should really be cleaned up.
    // There are a couple of tricky issues here, if we use only first order relaxation
    // the car will sway laterally at low speeds.
    // To solve the sway problem, we need to add some low speed damping and clamp the
    // undamped integration.

    auto vx_abs = abs(vx);

    auto kappa = m_states.cp_long_slip;  // m_states_in.cp_long_slip;
    auto alpha = m_states.cp_side_slip;  // m_states_in.cp_side_slip;

    // Long Slip Range
    auto KPUMIN = par.KPUMIN;
    auto KPUMAX = par.KPUMAX;

    // Slip Angle Range
    auto ALPMIN = par.ALPMIN;
    auto ALPMAX = par.ALPMAX;

    auto PCFX1 = par.PCFX1;  // Tyre overall longitudinal stiffness vertical deflection dependency linear term
    auto PCFX2 = par.PCFX2;  // Tyre overall longitudinal stiffness vertical deflection dependency quadratic term
    auto PCFX3 = par.PCFX3;  // Tyre overall longitudinal stiffness pressure dependency
    auto PCFY1 = par.PCFY1;  // Tyre overall lateral stiffness vertical deflection dependency linear term
    auto PCFY2 = par.PCFY2;  // Tyre overall lateral stiffness vertical deflection dependency quadratic term
    auto PCFY3 = par.PCFY3;  // Tyre overall lateral stiffness pressure dependency
    auto cx0 = par.LONGITUDINAL_STIFFNESS;  // Tyre overall longitudinal stiffness
    auto cy0 = par.LATERAL_STIFFNESS;       // Tyre overall lateral stiffness

    // Calculate contact patch deformation
    double a = ContactPatch(dpi, Fz);

    // Contact patch relaxation length(equals half of the contact length)
    auto sigma_c = fmax(a, 0.02);  // Eqn 9.29 pag 412
    // (0.02 is hard coded to avoid division between 0 at low vertical loads check Epsilon_Lim in Besselink PhD)
    // Sigma_min = Table 8.4 Page 400 from the book

    // Overall longitudinal Cx and lateral stiffness Cy
    auto cx = cx0 * (1.0 + (PCFX1 + PCFX2 * dfz) * dfz) * (1.0 + PCFX3 * dpi);  // (Eqn 17 - Paper)
    auto cy = cy0 * (1.0 + (PCFY1 + PCFY2 * dfz) * dfz) * (1.0 + PCFY3 * dpi);  // (Eqn 18 - Paper)

    if (m_Kxk == 0.0 || m_Kya == 0.0)
        return;

    // Carcass stiffness
    auto Ccx = (m_Kxk * cx) / (m_Kxk - cx * a);  // (Eqn 25 - Paper)
    auto Ccy = (m_Kya * cy) / (m_Kya - cy * a);  // (Eqn 25 - Paper)

    /*
    if (Fz == 0.0) {
        Ccx = cx;
        Ccy = cy;
    }
    */

    // Carcass damping
    auto kcx = par.DAMP_RESIDUAL * Ccx;
    auto kcy = par.DAMP_RESIDUAL * Ccy;

    auto depsilonx = (Fx - (epsilonx * Ccx)) / kcx;
    auto depsilony = (Fy - (epsilony * Ccy)) / kcy;

    // Low speed reduction
    /*
    if (vx_abs < par.VXLOW) {
        auto k_vlow = 0.5 * (1.0 + cos(3.14 * vx_abs / par.VXLOW));
        depsilonx *= (1.0 - k_vlow);
        depsilony *= (1.0 - k_vlow);
    }
    */

    epsilonx += depsilonx * step;
    epsilony += depsilony * step;

    auto dvsx = -vsx - kappa * vx_abs - depsilonx;
    auto dvsy = +vsy - alpha * vx_abs + depsilony;

    kappa += dvsx * (step / (sigma_c + step * vx_abs));
    alpha += dvsy * (step / (sigma_c + step * vx_abs));

    ChClampValue(kappa, KPUMIN, KPUMAX);
    ChClampValue(alpha, ALPMIN, ALPMAX);

    m_states_in.cp_long_slip = kappa;
    m_states_in.cp_side_slip = alpha;

    m_states.cp_long_slip = m_states_in.cp_long_slip;
    m_states.cp_side_slip = m_states_in.cp_side_slip;
}

void ChMFTire::TransientSlip(double step, double vx, double vsx, double vsy, double sigma_x, double sigma_y) {
    /*
        This part should really be cleaned up.
        There are a couple of tricky issues here, if we use only first order relaxation
        the car will sway laterally at low speeds.
        To solve the sway problem, we need to add some low speed damping and clamp the
        undamped integration.
    */
    auto vx_abs = abs(vx);

    auto kappa = m_states.cp_long_slip;  // m_states_in.cp_long_slip;
    auto alpha = m_states.cp_side_slip;  // m_states_in.cp_side_slip;

    // Long Slip Range
    auto KPUMIN = par.KPUMIN;
    auto KPUMAX = par.KPUMAX;

    // Slip Angle Range
    auto ALPMIN = par.ALPMIN;
    auto ALPMAX = par.ALPMAX;

    // This should be handled better for proper reverse driving
    // Negative slip while in reverse is actually positive slip
    // which would have a different saturation range.

    // Apply some limits to avoid singularities
    sigma_x = fmax(sigma_x, 1.0e-3);
    sigma_y = fmax(sigma_y, 1.0e-3);

    // Low speed limiter
    // auto k_vlow = 0.0;

    /*
    if (vx_abs < par.VXLOW) {
        auto gain = 0.70;
        k_vlow = 0.5 * (1.0 + cos(3.14 * vx_abs / par.VXLOW));
        k_vlow = (1.0 - gain) + (1.0 - k_vlow * gain);
        kappa = kappa * (1.0 - k_vlow);
    }
    */

    // vx_abs = fmax(vx_abs, par.VXLOW);

    // kappa = kappa_p;
    // alpha = alpha_p;

    // Calculate slip derivatives
    // auto kappa_dot = (-vsx - vx_abs * kappa) / sigma_x;
    // auto alpha_dot = (+vsy - vx_abs * alpha) / sigma_y;

    // Direct calculation with slip state integration
    // kappa += (-vsx - kappa * vx_abs) / sigma_x * step;
    // alpha += (+vsy - alpha * vx_abs) / sigma_y * step;

    auto Aksl = 0.15;
    auto Aasl = 0.30;

    // vx_abs_cap = fmax(vx_abs, 0.1);

    auto dvsx = -vsx - kappa * vx_abs;
    auto dvsy = +vsy - alpha * vx_abs;

    bool clamp_integ = false;

    if (clamp_integ) {
        if (vx_abs < par.VXLOW && abs(kappa) > Aksl && dvsx * kappa > 0.0) {
        } else {
            kappa += dvsx * (step / (sigma_x + step * vx_abs));
        }
        if (vx_abs < par.VXLOW && abs(alpha) > Aasl && dvsy * alpha > 0.0) {
        } else {
            alpha += dvsy * (step / (sigma_y + step * vx_abs));
        }
    } else {
        kappa += dvsx * (step / (sigma_x + step * vx_abs));
        alpha += dvsy * (step / (sigma_y + step * vx_abs));
    }

    // vx_abs = fmax(vx_abs, 0.1);

    // kappa += (-vsx - kappa * vx_abs) * (step / (sigma_x + step * vx_abs));
    // alpha += (+vsy - alpha * vx_abs) * (step / (sigma_y + step * vx_abs));

    ChClampValue(kappa, KPUMIN, KPUMAX);
    ChClampValue(alpha, ALPMIN, ALPMAX);

    m_states_in.cp_long_slip = kappa;
    m_states_in.cp_side_slip = alpha;

    m_states.cp_long_slip = m_states_in.cp_long_slip;
    m_states.cp_side_slip = m_states_in.cp_side_slip;

    if (vx_abs < par.VXLOW) {
        // Add some artificial damping
        auto gain = (1.0 - vx_abs / par.VXLOW) * step;
        m_states_in.cp_long_slip = kappa + dvsx * gain * 100.0;
        m_states_in.cp_side_slip = alpha + dvsy * gain * 1000.0;
    }
}

void ChMFTire::CalculateForcesMoments(double step) {
    // Set tire forces to zero.
    m_tireforce_out.force = ChVector<>(0, 0, 0);
    m_tireforce_out.moment = ChVector<>(0, 0, 0);

    // Return now if no contact.
    if (!m_data_in.in_contact)
        return;

    auto m_vcx = abs(m_states.vx);
    // bool is_backwards = m_states.vx < -0.0;
    bool is_backwards = m_states.omega < -0.0;
    // is_backwards = false;

    // Perhaps, it is better for transient slip if we use omega*re
    // instead of vx, which would be more reliable during launch

    // TODO: Calculate turnslip variable
    auto m_phi = 0.0;

    unsigned short transient_slip_model = 0;

    // Transient slip
    if (transient_slip_model) {
        auto vxs = m_states.omega * m_states.R_eff;
        vxs = m_vcx;

        if (transient_slip_model == 1)
            CalculateRelaxationLength(m_fz, m_gamma, m_dfz, m_dpi);
        else
            TransientSlip(step, vxs, m_states_in.vsx, m_states_in.vsy, m_sigma_x, m_sigma_y);

    } else {
        // prevent singularity for kappa, when vx == 0
        auto v_bar = fmax(m_vcx, par.VXLOW);
        m_states_in.cp_long_slip = -m_states_in.vsx / v_bar;
        m_states_in.cp_side_slip = +m_states_in.vsy / v_bar;

        m_states.cp_long_slip = m_states_in.cp_long_slip;
        m_states.cp_side_slip = m_states_in.cp_side_slip;
    }

    m_alpha = m_states_in.cp_side_slip;
    m_kappa = m_states_in.cp_long_slip;

    auto alpha_prime =
        m_states_in.vsy / sqrt(fmax(m_states_in.vx * m_states_in.vx + m_states_in.vsy * m_states_in.vsy, par.VXLOW));

    if (is_backwards) {
        m_kappa = -m_kappa;
        m_alpha = -m_alpha;
        alpha_prime = -m_alpha;
    }

    // Express alpha and gamma in rad. Express kappa as ratio.
    // m_gamma = CH_C_PI_2 - std::acos(m_states_in.disc_normal.z());
    // m_gamma = CalculateGamma(flipSide);
    // m_alpha = m_states_in.cp_side_slip;
    // m_kappa = m_states_in.cp_long_slip;
    // m_fz = m_fz;
    // m_dpi = m_dpi;
    // m_dfz = m_dfz;

    // If we don't use the star variables, we need to take the arctangent
    if (!useStarInputs)
        m_alpha = atan(m_alpha);

    // Long Slip Range
    auto KPUMIN = par.KPUMIN;
    auto KPUMAX = par.KPUMAX;

    // Slip Angle Range
    auto ALPMIN = par.ALPMIN;
    auto ALPMAX = par.ALPMAX;

    // Saturate slips (alpha star is used here for large angles)
    ChClampValue(m_kappa, KPUMIN, KPUMAX);
    ChClampValue(m_alpha, ALPMIN, ALPMAX);

    double Fx = 0;
    double Fy = 0;
    // double Fz = m_data_in.normal_force;
    double Mx = 0;
    double My = 0;
    double Mz = 0;

    switch (m_use_mode) {
        case 0:
            // vertical spring & damper mode
            break;
        case 1:
            // steady state pure longitudinal slip
            Fx = CalculateFx(m_fz, m_kappa, 0.0, m_gamma, m_dfz, m_dpi, m_phi);
            break;
        case 2:
            // steady state pure lateral slip
            Fy = CalculateFy(m_fz, 0.0, m_alpha, m_gamma, m_dfz, m_dpi, m_phi, m_vcx);
            break;
        case 3:
            // steady state pure lateral slip uncombined
            Fx = CalculateFx(m_fz, m_kappa, 0.0, m_gamma, m_dfz, m_dpi, m_phi);
            Fy = CalculateFy(m_fz, 0.0, m_alpha, m_gamma, m_dfz, m_dpi, m_phi, m_vcx);
            Mx = CalculateMx(m_fz, m_gamma, m_dpi, Fy);
            My = CalculateMy(m_fz, m_gamma, m_dpi, Fx, m_vcx);
            Mz = CalculateMz(m_fz, 0.0, m_alpha, alpha_prime, m_gamma, m_dfz, m_dpi, m_phi, Fx, Fy, m_vcx);
            break;
        case 4:
            // steady state combined slip
            Fx = CalculateFx(m_fz, m_kappa, m_alpha, m_gamma, m_dfz, m_dpi, m_phi);
            Fy = CalculateFy(m_fz, m_kappa, m_alpha, m_gamma, m_dfz, m_dpi, m_phi, m_vcx);
            Mx = CalculateMx(m_fz, m_gamma, m_dpi, Fy);
            My = CalculateMy(m_fz, m_gamma, m_dpi, Fx, m_vcx);
            Mz = CalculateMz(m_fz, m_kappa, m_alpha, alpha_prime, m_gamma, m_dfz, m_dpi, m_phi, Fx, Fy, m_vcx);
            break;
    }

    // Flip tire side
    if (flipSide) {
        Fy = -Fy;
        Mx = -Mx;
        Mz = -Mz;
    }

    if (is_backwards) {
        Fx = -Fx;
        Fy = -Fy;
        My = -My;
    }

    // Store longitudinal and lateral forces so that they can be used in the Fz calculation
    m_fx_fb = Fx;
    m_fy_fb = Fy;

    // Compile the force and moment vectors so that they can be
    // transformed into the global coordinate system.
    // Convert from SAE to ISO Coordinates at the contact patch

    if (useIsoRef) {
        m_tireforce_out.force = ChVector<>(Fx, Fy, m_data_in.normal_force);
        m_tireforce_out.moment = ChVector<>(Mx, My, Mz);
    } else {
        m_tireforce_out.force = ChVector<>(Fx, -Fy, m_data_in.normal_force);
        m_tireforce_out.moment = ChVector<>(-Mx, My, -Mz);
    }
}

void ChMFTire::Advance(double step) {
    CopyContactData();
    CopyTireStates();

    CalculateForcesMoments(step);

    m_tireforce.force = m_tireforce_out.force;
    m_tireforce.moment = m_tireforce_out.moment;

    return;
}

// -----------------------------------------------------------------------------

void ChMFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape =
        ChVehicleGeometry::AddVisualizationCylinder(m_wheel->GetSpindle(),                                        //
                                                    ChVector<>(0, GetOffset() + GetVisualizationWidth() / 2, 0),  //
                                                    ChVector<>(0, GetOffset() - GetVisualizationWidth() / 2, 0),  //
                                                    par.UNLOADED_RADIUS);
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void ChMFTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChPac02Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------

template <typename T>
double signp(T val) noexcept {
    return (val < T(0)) ? -1.0 : 1.0;
}

double ChMFTire::CalculateFx(double Fz, double kappa, double alpha, double gamma, double dfz, double dpi, double phi) {
    const double epsilon = 1e-6;  // [Eqn(4.E6a) Page 178 - Book]
    const double epsilonx = epsilon;

    // This is actually not true, alpha* is tan(alpha)*sign(Vcx) and gamma* is sin(gamma)
    // Thit is an alternative input for large angles and can be implemented later if desired.
    // It is more efficient to do this elsewhere though.
    auto alpha_star = alpha;
    auto gamma_star = gamma;

    // TODO:
    //  Handle reverse
    //  Apply low load scaling
    //  ...

    //
    // [SCALING_COEFFICIENTS]
    auto LCX = par.LCX;  // Scale factor of Fx shape factor
    auto LEX = par.LEX;  // Scale factor of Fx curvature factor
    auto LKX = par.LKX;  // Scale factor of Fx slip stiffness
    auto LHX = par.LHX;  // Scale factor of Fx horizontal shift
    auto LVX = par.LVX;  // Scale factor of Fx vertical shift

    auto LXAL = par.LXAL;  // Scale factor of alpha influence on Fx

    // [LONGITUDINAL_COEFFICIENTS]
    auto PCX1 = par.PCX1;  // Shape factor Cfx for longitudinal force
    auto PDX1 = par.PDX1;  // Longitudinal friction Mux at Fznom
    auto PDX2 = par.PDX2;  // Variation of friction Mux with load
    auto PDX3 = par.PDX3;  // Variation of friction Mux with camber squared
    auto PEX1 = par.PEX1;  // Longitudinal curvature Efx at Fznom
    auto PEX2 = par.PEX2;  // Variation of curvature Efx with load
    auto PEX3 = par.PEX3;  // Variation of curvature Efx with load squared
    auto PEX4 = par.PEX4;  // Factor in curvature Efx while driving
    auto PKX1 = par.PKX1;  // Longitudinal slip stiffness Kfx. / Fz at Fznom
    auto PKX2 = par.PKX2;  // Variation of slip stiffness Kfx. / Fz with load
    auto PKX3 = par.PKX3;  // Exponent in slip stiffness Kfx. / Fz with load
    auto PHX1 = par.PHX1;  // Horizontal shift Shx at Fznom
    auto PHX2 = par.PHX2;  // Variation of shift Shx with load
    auto PVX1 = par.PVX1;  // Vertical shift Svx. / Fz at Fznom
    auto PVX2 = par.PVX2;  // Variation of shift Svx. / Fz with load
    auto PPX1 = par.PPX1;  // linear influence of inflation pressure on longitudinal slip stiffness
    auto PPX2 = par.PPX2;  // quadratic influence of inflation pressure on longitudinal slip stiffness
    auto PPX3 = par.PPX3;  // linear influence of inflation pressure on peak longitudinal friction
    auto PPX4 = par.PPX4;  // quadratic influence of inflation pressure on peak longitudinal friction

    auto RBX1 = par.RBX1;  // Slope factor for combined slip Fx reduction
    auto RBX2 = par.RBX2;  // Variation of slope Fx reduction with kappa
    auto RBX3 = par.RBX3;  // Influence of camber on stiffness for Fx combined
    auto RCX1 = par.RCX1;  // Shape factor for combined slip Fx reduction
    auto REX1 = par.REX1;  // Curvature factor of combined Fx
    auto REX2 = par.REX2;  // Curvature factor of combined Fx with load
    auto RHX1 = par.RHX1;  // Shift factor for combined slip Fx reduction

    // [TURNSLIP]
    auto PDXP1 = par.PDXP1;
    auto PDXP2 = par.PDXP2;
    auto PDXP3 = par.PDXP3;

    auto R0 = par.UNLOADED_RADIUS;

    // Pre-calculate
    auto gamma_star_sqr = gamma_star * gamma_star;

    auto zeta1 = 1.0;

    if (useTurnSlip) {
        auto Bxp = PDXP1 * (1.0 + PDXP2 * dfz) * cos(atan(PDXP3 * kappa));  // [Eqn(4.106) Page 188 - Book]
        zeta1 = cos(atan(Bxp * R0 * phi));                                  // [Eqn(4.105) Page 188 - Book]
    }

    auto Cx = PCX1 * LCX;  // (> 0) (4.E11)
    // auto mux = (PDX1 + PDX2 * dfz) * (1.0 + PPX3 * dpi + PPX4 * pow(dpi, 2)) * (1.0 - PDX3 * pow(gamma, 2)) *
    // LMUX_star;  // (4.E13)
    auto mux =
        (PDX1 + PDX2 * dfz) * (1.0 + (PPX3 + PPX4 * dpi) * dpi) * (1.0 - PDX3 * gamma_star_sqr) * LMUX_star;  // (4.E13)

    auto Dx = mux * Fz * zeta1;  // (> 0) (4.E12)
    // auto Kxk = Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz) * (1.0 + PPX1 * dpi + PPX2 * pow(dpi, 2)) * LKX;  // (=
    // BxCxDx = dFxo. / dkx at kappax = 0) (= Cfk) (4.E15)
    auto Kxk = Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz) * (1.0 + (PPX1 + PPX2 * dpi) * dpi) *
               LKX;           // (= BxCxDx = dFxo. / dkx at kappax = 0) (= Cfk) (4.E15)
    auto signDx = signp(Dx);  // if (signDx == 0) signDx = 1; // If[Dx = 0] then[sign(0) = 0]. This is done to avoid[Kxk
                              // / 0 = NaN] in Eqn 4.E16
    auto Bx = Kxk / (Cx * Dx + epsilonx * signDx);                   // (4.E16)[sign(Dx) term explained on page 177]
    auto SHx = (PHX1 + PHX2 * dfz) * LHX;                            // (4.E17)
    auto SVx = Fz * (PVX1 + PVX2 * dfz) * LVX * LMUX_prime * zeta1;  // (4.E18)

    // Store as internal variable
    m_Kxk = Kxk;

    // Low speed model
    /* TODO: Implement low speed correction
    if (isLowSpeed) {
        SVx = SVx * reductionSmooth;
        SHx = SHx * reductionSmooth;
    }  // if isLowSpeed
    */

    auto kappax = kappa + SHx;  // (4.E10)

    // Only in Linear Transients mode
    /*
    if (userDynamics == 2) {
        if (Vx < 0)
            kappax = -kappax;
    }  // if linear transients
    */

    // auto Ex = fmin((PEX1 + PEX2 * dfz + PEX3 * pow(dfz, 2)) * (1.0 - PEX4 * signp(kappax)) * LEX, 1.0);  // (<= 1)
    // (4.E14) auto Ex = fmin((PEX1 + (PEX2 + PEX3 * dfz) * dfz) * (1.0 - PEX4 * sgn(kappax)) * LEX, 1.0);  // (<= 1)
    // (4.E14)
    auto Ex = fmin((PEX1 + (PEX2 + PEX3 * dfz) * dfz) * (1.0 - PEX4 * signp(kappax)) * LEX, 1.0);  // (<= 1) (4.E14)
    // (should actually be sign(kappa), but this difference is unlikely to cause discepency)

    // Pure longitudinal force
    double Fx0 = Dx * sin(Cx * atan(Bx * kappax - Ex * (Bx * kappax - atan(Bx * kappax)))) + SVx;  // (4.E9)

    // TODO: Low speed correction model
    // if (isLowSpeed && userDynamics == 2) {
    //      // Low speed model and Linear Transients
    //      Fx0 = Dx * (1 - reductionSmooth) * sign(kappax) + Fx0 * reductionSmooth;
    // }

    // Backward speed check
    /*
    if (userDynamics != 3) {
        if (Vx < 0) Fx0 = -Fx0;
    }
    */

    auto Cxa = RCX1;                          // (4.E55)
    auto Exa = fmin(REX1 + REX2 * dfz, 1.0);  // (<= 1) (4.E56)

    auto SHxa = RHX1;                                                            // (4.E57)
    auto Bxa = (RBX1 + RBX3 * gamma_star_sqr) * cos(atan(RBX2 * kappa)) * LXAL;  // (> 0) (4.E54)

    auto alphas = alpha_star + SHxa;  // (4.E53)

    auto Gxa0 = cos(Cxa * atan(Bxa * SHxa - Exa * (Bxa * SHxa - atan(Bxa * SHxa))));              // (4.E52)
    auto Gxa = cos(Cxa * atan(Bxa * alphas - Exa * (Bxa * alphas - atan(Bxa * alphas)))) / Gxa0;  // (> 0)(4.E51

    auto Fx = Gxa * Fx0;  // (4.E50)

    return Fx * m_fz_scl;
}

double ChMFTire::CalculateFy(double Fz,
                             double kappa,
                             double alpha,
                             double gamma,
                             double dfz,
                             double dpi,
                             double phi,
                             double Vcx) {
    const double epsilon = 1e-6;  // [Eqn(4.E6a) Page 178 - Book]
    // const double epsilonv = epsilon;
    // const double epsilonx = epsilon;
    const double epsilonk = epsilon;
    const double epsilony = epsilon;
    // const double epsilonr = epsilon;

    // This is actually not true, alpha* is tan(alpha)*sign(Vcx) and gamma* is sin(gamma)
    // Thit is an alternative input for large angles and can be implemented later if desired.
    // It is more efficient to do this elsewhere though.
    auto alpha_star = alpha;
    auto gamma_star = gamma;

    // [SCALING_COEFFICIENTS]
    auto LCY = par.LCY;    // Scale factor of Fy shape factor
    auto LEY = par.LEY;    // Scale factor of Fy curvature factor
    auto LKY = par.LKY;    // Scale factor of Fy cornering stiffness
    auto LHY = par.LHY;    // Scale factor of Fy horizontal shift
    auto LVY = par.LVY;    // Scale factor of Fy vertical shift
    auto LKYC = par.LKYC;  // Scale factor of camber force stiffness

    auto LYKA = par.LYKA;    // Scale factor of alpha influence on Fx
    auto LVYKA = par.LVYKA;  // Scale factor of kappa induced Fy

    // [LATERAL_COEFFICIENTS]
    auto PCY1 = par.PCY1;  // Shape factor Cfy for lateral forces
    auto PDY1 = par.PDY1;  // Lateral friction Muy
    auto PDY2 = par.PDY2;  // Variation of friction Muy with load
    auto PDY3 = par.PDY3;  // Variation of friction Muy with squared camber
    auto PEY1 = par.PEY1;  // Lateral curvature Efy at Fznom
    auto PEY2 = par.PEY2;  // Variation of curvature Efy with load
    auto PEY3 = par.PEY3;  // Zero order camber dependency of curvature Efy
    auto PEY4 = par.PEY4;  // Variation of curvature Efy with camber
    auto PEY5 = par.PEY5;  // Variation of curvature Efy with camber squared
    auto PKY1 = par.PKY1;  // Maximum value of stiffness Kfy. / Fznom
    auto PKY2 = par.PKY2;  // Load at which Kfy reaches maximum value
    auto PKY3 = par.PKY3;  // Variation of Kfy. / Fznom with camber
    auto PKY4 = par.PKY4;  // Curvature of stiffness Kfy
    auto PKY5 = par.PKY5;  // Peak stiffness variation with camber squared
    auto PKY6 = par.PKY6;  // Fy camber stiffness factor
    auto PKY7 = par.PKY7;  // Vertical load dependency of camber stiffness
    auto PHY1 = par.PHY1;  // Horizontal shift Shy at Fznom
    auto PHY2 = par.PHY2;  // Variation of shift Shy with load
    auto PHY3 = par.PHY3;  // Variation of shift Shy with camber
    auto PVY1 = par.PVY1;  // Vertical shift in Svy. / Fz at Fznom
    auto PVY2 = par.PVY2;  // Variation of shift Svy. / Fz with load
    auto PVY3 = par.PVY3;  // Variation of shift Svy. / Fz with camber
    auto PVY4 = par.PVY4;  // Variation of shift Svy. / Fz with camber and load
    auto PPY1 = par.PPY1;  // influence of inflation pressure on cornering stiffness
    auto PPY2 = par.PPY2;  // influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
    auto PPY3 = par.PPY3;  // linear influence of inflation pressure on lateral peak friction
    auto PPY4 = par.PPY4;  // quadratic influence of inflation pressure on lateral peak friction
    auto PPY5 = par.PPY5;  // Influence of inflation pressure on camber stiffness

    auto RBY1 = par.RBY1;  // Slope factor for combined Fy reduction
    auto RBY2 = par.RBY2;  // Variation of slope Fy reduction with alpha
    auto RBY3 = par.RBY3;  // Shift term for alpha in slope Fy reduction
    auto RBY4 = par.RBY4;  // Influence of camber on stiffness of Fy combined
    auto RCY1 = par.RCY1;  // Shape factor for combined Fy reduction
    auto REY1 = par.REY1;  // Curvature factor of combined Fy
    auto REY2 = par.REY2;  // Curvature factor of combined Fy with load
    auto RHY1 = par.RHY1;  // Shift factor for combined Fy reduction
    auto RHY2 = par.RHY2;  // Shift factor for combined Fy reduction with load
    auto RVY1 = par.RVY1;  // Kappa induced side force Svyk. / Muy.*Fz at Fznom
    auto RVY2 = par.RVY2;  // Variation of Svyk. / Muy.*Fz with load
    auto RVY3 = par.RVY3;  // Variation of Svyk. / Muy.*Fz with camber
    auto RVY4 = par.RVY4;  // Variation of Svyk. / Muy.*Fz with alpha
    auto RVY5 = par.RVY5;  // Variation of Svyk. / Muy.*Fz with kappa
    auto RVY6 = par.RVY6;  // Variation of Svyk. / Muy.*Fz with atan(kappa)

    // [TURNSLIP_COEFFICIENTS]
    auto PKYP1 = par.PKYP1;  // Cornering stiffness reduction due to spin
    auto PDYP1 = par.PDYP1;  // Peak Fy reduction due to spin parameter
    auto PDYP2 = par.PDYP2;  // Peak Fy reduction due to spin with varying load parameter
    auto PDYP3 = par.PDYP3;  // Peak Fy reduction due to spin with alpha parameter
    auto PDYP4 = par.PDYP4;  // Peak Fy reduction due to square root of spin parameter

    auto PHYP1 = par.PHYP1;  // Fy - alpha curve lateral shift limitation
    auto PHYP2 = par.PHYP2;  // Fy - alpha curve maximum lateral shift parameter
    auto PHYP3 = par.PHYP3;  // Fy - alpha curve maximum lateral shift varying with load parameter
    auto PHYP4 = par.PHYP4;  // Fy - alpha curve maximum lateral shift parameter

    auto PECP1 = par.PECP1;
    auto PECP2 = par.PECP2;

    // [DIMENSION]
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius

    // Pre-calculate
    auto Fz0_prime = par.LFZO * par.FNOMIN;  // [Eqn(4.E1) Page 177 - Book]

    // Copy the original gamma values
    auto gamma_star_copy = gamma_star;
    double gamma_star_sqr;

    auto Fy = 0.0;

    for (int ii = 0; ii < 2; ii++) {
        // Pre - allocate MF5.2 parameters for C code generation
        // auto PHY3 = 0; // Variation of shift Shy with camber
        auto zeta0 = 1.0;
        auto zeta2 = 1.0;
        auto zeta3 = 1.0;
        auto zeta4 = 1.0;

        double epsilong = 0.0;

        if (ii == 0) {
            useTurnSlip = false;
            gamma_star = 0.0;
            gamma_star_sqr = 0.0;
        } else {
            gamma_star = gamma_star_copy;
            gamma_star_sqr = gamma_star * gamma_star;
        }

        // Turn slip
        if (useTurnSlip) {
            epsilong = PECP1 * (1.0 + PECP2 * dfz);  // [Eqn(4.90) Page 186 - Book] Camber reduction factor

            zeta3 = cos(atan(PKYP1 * pow(R0, 2) * pow(phi, 2)));                     // [Eqn(4.79) Page 185 - Book]
            auto Byp = PDYP1 * (1.0 + PDYP2 * dfz) * cos(atan(PDYP3 * tan(alpha)));  // [Eqn(4.78) Page 185 - Book]
            zeta2 = cos(atan(Byp * (R0 * abs(phi) + PDYP4 * sqrt(R0 * abs(phi)))));  // [Eqn(4.77) Page 184 - Book]
        }

        auto Kya = PKY1 * Fz0_prime * (1.0 + PPY1 * dpi) * (1.0 - PKY3 * abs(gamma_star)) *
                   sin(PKY4 * atan((Fz / Fz0_prime) / ((PKY2 + PKY5 * gamma_star_sqr) * (1.0 + PPY2 * dpi)))) * zeta3 *
                   LKY;  // (= ByCyDy = dFyo. / dalphay at alphay = 0) (if gamma = 0: = Kya0 = CFa) (PKY4 = 2)(4.E25)
        auto SVyg = Fz * (PVY3 + PVY4 * dfz) * gamma_star * LKYC * LMUY_prime * zeta2;  // (4.E28)

        double Kyg0;
        // double Kya0;

        // Check MF version
        if (par.FITTYP == 6 || par.FITTYP == 21) {
            // Load MF5.2 parameters

            // MF5.2 equations from the MF - Tyre equation manual
            double Kya0 = PKY1 * Fz0_prime * sin(PKY4 * atan(Fz / (PKY2 * Fz0_prime))) *
                          LKYC;  // Simplified without pressure dependency
            Kyg0 = (PHY3 * Kya0 + Fz * (PVY3 + PVY4 * dfz)) * LKYC;
        } else {
            // MF6.1 and 6.2 equatons
            Kyg0 = Fz * (PKY6 + PKY7 * dfz) * (1.0 + PPY5 * dpi) *
                   LKYC;  // (= dFyo. / dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
        }

        if (useTurnSlip) {
            auto Kya0 = PKY1 * Fz0_prime * (1.0 + PPY1 * dpi) * (1.0 - PKY3 * abs(0.0)) *
                        sin(PKY4 * atan((Fz / Fz0_prime) / ((PKY2 + PKY5 * pow(0.0, 2)) * (1.0 + PPY2 * dpi)))) *
                        zeta3 * LKY;

            // IMPORTANT NOTE : Explanation of the above equation, Kya0
            // Kya0 is the cornering stiffness when the camber angle is zero
            // (gamma = 0) which is again the product of the coefficients By, Cyand
            // Dy at zero camber angle.Information from Kaustub Ragunathan, email :
            // carmaker - service - uk@ipg - automotive.com

            // signKya = sign(Kya);
            // signKya(signKya == 0) = 1; // If[Kya = 0] then[sign(0) = 0].This is done to avoid[num / 0 = NaN] in
            // Eqn 4.E39

            auto signKya =
                signp(Kya);  // If[Kya = 0] then[sign(0) = 0].This is done to avoid[num / 0 = NaN] in Eqn 4.E39
            auto signKya0 = signp(Kya0);  // If[Kya0 = 0] then[sign(0) = 0].This is done to avoid[num / 0 = NaN]

            auto Kya_prime = Kya + epsilonk * signKya;  // (4.E39)[sign(Kya) term explained on page 177]
            auto Kyao_prime =
                Kya0 + epsilonk * signKya0;  // epsilonk is a small factor added to avoid the singularity condition
                                             // during zero velocity(equation 308, CarMaker reference Manual).

            auto CHyp = PHYP1;  // (> 0)[Eqn(4.85) Page 186 - Book]
            // auto DHyp = (PHYP2 + PHYP3 * dfz) * sgn(Vcx);   // [Eqn(4.86) Page 186 - Book]
            auto DHyp = (PHYP2 + PHYP3 * dfz) * signp(Vcx);  // [Eqn(4.86) Page 186 - Book]
            auto EHyp = fmin(PHYP4, 1.0);                    // (<= 1)[Eqn(4.87) Page 186 - Book]

            auto KyRp0 = Kyg0 / (1.0 - epsilong);            // Eqn(4.89)
            auto BHyp = KyRp0 / (CHyp * DHyp * Kyao_prime);  // [Eqn(4.88) Page 186 - Book]
            // auto SHyp = DHyp * sin(CHyp * atan(BHyp * R0 * phi - EHyp * (BHyp * R0 * phi - atan(BHyp * R0 * phi)))) *
            // sgn(Vcx);  // [Eqn(4.80) Page 185 - Book]
            auto SHyp = DHyp * sin(CHyp * atan(BHyp * R0 * phi - EHyp * (BHyp * R0 * phi - atan(BHyp * R0 * phi)))) *
                        signp(Vcx);  // [Eqn(4.80) Page 185 - Book]

            zeta0 = 0.0;                            // [Eqn(4.83) Page 186 - Book]
            zeta4 = 1.0 + SHyp - SVyg / Kya_prime;  // [Eqn(4.84) Page 186 - Book]
        }

        // signKya = sign(Kya);
        // signKya(signKya == 0) = 1; // If[Kya = 0] then[sign(0) = 0].This is done to avoid[num / 0 = NaN] in Eqn 4.E27
        auto signKya = signp(Kya);

        double SHy;

        // Check MF version
        if (par.FITTYP == 6 || par.FITTYP == 21) {
            // MF5.2 equations
            SHy = (PHY1 + PHY2 * dfz) * LHY + PHY3 * gamma_star * LKYC;  // From the MF - Tyre equation manual
        } else {
            // MF6.1 and 6.2 equatons
            SHy = (PHY1 + PHY2 * dfz) * LHY + ((Kyg0 * gamma_star - SVyg) / (Kya + epsilonk * signKya)) * zeta0 +
                  zeta4 - 1.0;  // (4.E27)[sign(Kya) term explained on page 177]
        }
        auto SVy = Fz * (PVY1 + PVY2 * dfz) * LVY * LMUY_prime * zeta2 + SVyg;  // (4.E29)

        // Low speed model
        // isLowSpeed = modes.isLowSpeed;
        // TODO: Handle low speed properly
        /*
        if (isLowSpeed) {
            SVy = SVy * reductionSmooth;
            SHy = SHy * reductionSmooth;
        }  // if isLowSpeed
        */
        auto alphay = alpha_star + SHy;  // (4.E20)
        auto Cy = PCY1 * LCY;            // (> 0) (4.E21)
        auto muy = (PDY1 + PDY2 * dfz) * (1.0 + PPY3 * dpi + PPY4 * pow(dpi, 2)) * (1.0 - PDY3 * gamma_star_sqr) *
                   LMUY_star;        // (4.E23)
        auto Dy = muy * Fz * zeta2;  // (4.E22)
        // signAlphaY = sign(alphay);
        // signAlphaY(signAlphaY == 0) = 1;
        auto signAlphaY = signp(alphay);
        auto Ey =
            fmin((PEY1 + PEY2 * dfz) * (1.0 + PEY5 * gamma_star_sqr - (PEY3 + PEY4 * gamma_star) * signAlphaY) * LEY,
                 1.0);  // (<= 1)(4.E24)

        // signDy = sign(Dy);
        // signDy(signDy == 0) = 1; // If[Dy = 0] then[sign(0) = 0].This is done to avoid[Kya / 0 = NaN] in Eqn 4.E26
        auto signDy = signp(Dy);  // If[Dy = 0] then[sign(0) = 0].This is done to avoid[Kya / 0 = NaN] in Eqn 4.E26

        auto By = Kya / (Cy * Dy + epsilony * signDy);  // (4.E26)[sign(Dy) term explained on page 177]
        auto Fy0 = Dy * sin(Cy * atan(By * alphay - Ey * (By * alphay - atan(By * alphay)))) + SVy;  // (4.E19)

        // Backward speed check for alpha_star
        /*
        if (useAlphaStar) {
            if (Vcx < 0) {
                Fy0 = -Fy0;
            }
        }  // if useAlphaStar
        */
        // if (isLowSpeed && modes.userDynamics == 2) {
        //  //Low speed modeland Linear Transients
        //  Fy0 = Dy * (1 - reductionSmooth) * sign(-alphay + eps) + Fy0 * reductionSmooth;
        // }

        // calculateFy0

        // Evaluate Gyk with phit = 0 (Note: needs to take gamma into account to match TNO)
        // Note: in the above equation starVar is used instead of
        // starVar_sub0 beacuse it was found a better match with TNO

        // This part does not seem to match TNO documentation, not sure why.
        // This is taken from MFEval directly
        gamma_star = gamma_star_copy;
        gamma_star_sqr = gamma_star * gamma_star;

        auto DVyk =
            muy * Fz * (RVY1 + RVY2 * dfz + RVY3 * gamma_star) * cos(atan(RVY4 * alpha_star)) * zeta2;  // (4.E67)
        auto SVyk = DVyk * sin(RVY5 * atan(RVY6 * kappa)) * LVYKA;                                      // (4.E66)
        auto SHyk = RHY1 + RHY2 * dfz;                                                                  // (4.E65)
        auto Eyk = fmin(REY1 + REY2 * dfz, 1.0);  // (<= 1) (4.E64)

        auto Cyk = RCY1;                                                                           // (4.E63)
        auto Byk = (RBY1 + RBY4 * gamma_star_sqr) * cos(atan(RBY2 * (alpha_star - RBY3))) * LYKA;  // (> 0) (4.E62)
        auto kappas = kappa + SHyk;                                                                // (4.E61)

        auto Gyk0 = cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));              // (4.E60)
        auto Gyk = cos(Cyk * atan(Byk * kappas - Eyk * (Byk * kappas - atan(Byk * kappas)))) / Gyk0;  // (> 0)(4.E59)

        // Low speed model
        // isLowSpeed = modes.isLowSpeed;
        /*
        if (isLowSpeed)  // Line for Simulink
        {
            SVyk = SVyk * reductionSmooth;
        }
        */

        auto Fy_Gyk = Gyk * Fy0;
        Fy = Fy_Gyk + SVyk;  // (4.E58)

        if (ii == 0) {
            // Unsure if we are supposed to use Fy0
            m_Fy_prime = Fy_Gyk;
            // m_Fy0_sub0 = Fy0;
            // m_Gyk_sub0 = Gyk;
        } else {
            m_By = By;
            m_Cy = Cy;
            m_Gyk = Gyk;
            m_SHy = SHy;
            m_SVy = SVy;
            m_SVyk = SVyk;
            m_Kya = Kya;
            m_muy = muy;
        }
    }

    return Fy * m_fz_scl;
}

double ChMFTire::CalculateMx(double Fz, double gamma, double dpi, double Fy) {
    // [VERTICAL]
    auto Fz0 = par.FNOMIN;  // Nominal wheel load

    // [DIMENSION]
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius

    // [SCALING_COEFFICIENTS]
    auto LVMX = par.LVMX;  // Scale factor of Mx vertical shift
    auto LMX = par.LMX;    // Scale factor of overturning couple

    // [OVERTURNING_COEFFICIENTS]
    auto QSX1 = par.QSX1;    // Vertical shift of overturning moment
    auto QSX2 = par.QSX2;    // Camber induced overturning couple
    auto QSX3 = par.QSX3;    // Fy induced overturning couple
    auto QSX4 = par.QSX4;    // Mixed load lateral forceand camber on Mx
    auto QSX5 = par.QSX5;    // Load effect on Mx with lateral forceand camber
    auto QSX6 = par.QSX6;    // B - factor of load with Mx
    auto QSX7 = par.QSX7;    // Camber with load on Mx
    auto QSX8 = par.QSX8;    // Lateral force with load on Mx
    auto QSX9 = par.QSX9;    // B - factor of lateral force with load on Mx
    auto QSX10 = par.QSX10;  // Vertical force with camber on Mx
    auto QSX11 = par.QSX11;  // B - factor of vertical force with camber on Mx
    auto QSX12 = par.QSX12;  // Camber squared induced overturning moment
    auto QSX13 = par.QSX13;  // Lateral force induced overturning moment
    auto QSX14 = par.QSX14;  // Lateral force induced overturning moment with camber
    auto PPMX1 = par.PPMX1;  // Influence of inflation pressure on overturning moment

    // Probably not very important to implement this...
    // we instead simply scale the output with our force multiplier
    //
    // Empirically discovered :
    // If Fz is below FzMin a reduction factor is applied :
    // auto reduction_lowFz = Fz * pow(Fz / tirParams.FZMIN, 2);
    // Fz = Fz < tirParams.FZMIN ? reduction_lowFz : Fz;

    auto Mx = 0.0;

    if ((QSX12 || QSX13 || QSX14) &&
        (QSX1 || QSX2 || QSX3 || QSX4 || QSX5 || QSX6 || QSX7 || QSX8 || QSX9 || QSX10 || QSX11)) {
        // Draft paper definition :
        Mx = R0 * Fz * LMX *
                 (QSX1 * LVMX - QSX2 * gamma * (1.0 + PPMX1 * dpi) - QSX12 * gamma * abs(gamma) + QSX3 * Fy / Fz0 +
                  QSX4 * cos(QSX5 * atan(pow(QSX6 * Fz / Fz0, 2))) * sin(QSX7 * gamma + QSX8 * atan(QSX9 * Fy / Fz0)) +
                  QSX10 * atan(QSX11 * Fz / Fz0) * gamma) +
             R0 * Fy * LMX * (QSX13 + QSX14 * abs(gamma));  // (49)
        // warning('Solver:Mx:AlternativeEquation', 'The parameter sets QSX1 to QSX11 and QSX12 to QSX14 cannnot be both
        // non-zero')

        // IMPORTANT NOTE : Is not recommended to use both sets of
        // parameters(QSX1 to QSX11 and QSX12 to QSX14), so a warning
        // flag will appear.
        // However if this is the case, I found that equation(49)
        // described in the draft paper of Besselink(Not the official
        // paper) will match the TNO solver output.This draft can be
        // downloaded from :
        //
        // https ://pure.tue.nl/ws/files/3139488/677330157969510.pdf
        // purl.tue.nl / 677330157969510.pdf
    } else {
        // Book definition
        // Mx = R0.*Fz.*(QSX1.*LVMX - QSX2.*gamma.*(1 + PPMX1.*dpi) + QSX3.*((Fy) / Fz0)...
        //     +QSX4.*cos(QSX5.*atan((QSX6.*(Fz. / Fz0)). ^ 2)).*sin(QSX7.*gamma + QSX8.*atan...
        //     (QSX9.*((Fy) / Fz0))) + QSX10.*atan(QSX11.*(Fz. / Fz0)).*gamma).*LMX;  // (4.E69)
        // IMPORTANT NOTE : The above book equation(4.E69) is not used
        // because it does not contain the parameters QSX12 to QSX14.
        // Instead I have coded the equation from the TNO Equation
        // Manual to match the TNO results.

        // TNO Equation Manual definition
        Mx = R0 * Fz * LMX *
                 (QSX1 * LVMX - QSX2 * gamma * (1.0 + PPMX1 * dpi) + QSX3 * (Fy / Fz0) +
                  QSX4 * cos(QSX5 * atan(pow(QSX6 * (Fz / Fz0), 2))) *
                      sin(QSX7 * gamma + QSX8 * atan(QSX9 * (Fy / Fz0))) +
                  QSX10 * atan(QSX11 * (Fz / Fz0)) * gamma) +
             R0 * LMX * (Fy * (QSX13 + QSX14 * abs(gamma)) - Fz * QSX12 * gamma * abs(gamma));
    }

    return Mx * m_fz_scl;
}

double ChMFTire::CalculateMy(double Fz, double gamma, double dpi, double Fx, double Vcx) {
    // [OPERATING_CONDITIONS]
    auto pi0 = par.NOMPRES;  // Nominal tyre inflation pressure

    // [MODEL]
    auto V0 = par.LONGVL;  // Nominal speed

    // [VERTICAL]
    auto Fz0 = par.FNOMIN;  // Nominal wheel load

    // [DIMENSION]
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius

    // [SCALING_COEFFICIENTS]
    auto LMY = par.LMY;  // Scale factor of rolling resistance torque

    // [ROLLING_COEFFICIENTS]
    auto QSY1 = par.QSY1;  // Rolling resistance torque coefficient
    auto QSY2 = par.QSY2;  // Rolling resistance torque depending on Fx
    auto QSY3 = par.QSY3;  // Rolling resistance torque depending on speed
    auto QSY4 = par.QSY4;  // Rolling resistance torque depending on speed . ^ 4
    auto QSY5 = par.QSY5;  // Rolling resistance torque depending on camber squared
    auto QSY6 = par.QSY6;  // Rolling resistance torque depending on loadand camber squared
    auto QSY7 = par.QSY7;  // Rolling resistance torque coefficient load dependency
    auto QSY8 = par.QSY8;  // Rolling resistance torque coefficient pressure dependency

    // auto VXLOW = par.VXLOW;
    auto p = dpi * pi0 + pi0;  // [Eqn(4.E2b) Page 177 - Book] Inverted equation for consistency

    double My;

    // My = Fz.R0 * (QSY1 + QSY2.*(Fx. / Fz0) + QSY3.*abs(Vcx. / V0) + QSY4.*(Vcx. / V0). ^ 4 ...
    //    +(QSY5 + QSY6.*(Fz. / Fz0)).*gamma. ^ 2).*((Fz. / Fz0). ^ QSY7.*(p. / pi0). ^ QSY8).*LMY.; % (4.E70)
    //
    // IMPORTANT NOTE : The above equation from the book(4.E70) is not used
    // because is not matching the results of the official TNO solver.
    // This equation gives a positive output of rolling resistance, and in the
    // ISO coordinate system, My should be negative.Furthermore, the equation
    // from the book has an error, multiplying all the equation by Fz instead of
    // Fz0(first term).
    // Because of the previous issues it has been used the equation(A48) from
    // the paper.

    // NOTE: Sign convention used here is identical to Pacejka, it is converted
    // back to ISO externally

    // Check MF version
    if (par.FITTYP == 6 || par.FITTYP == 21) {
        // MF5.2 equations
        // My = -R0 * Fz_unlimited * LMY *
        //     (QSY1 + QSY2 * (Fx / Fz0) + QSY3 * abs(Vcx / V0) +
        //      QSY4 * pow(Vcx / V0, 4));  // From the MF - Tyre equation manual

        My = -R0 * Fz * LMY *
             (QSY1 + QSY2 * (Fx / Fz0) + QSY3 * abs(Vcx / V0) +
              QSY4 * pow(Vcx / V0, 4));  // From the MF - Tyre equation manual
    } else {
        // MF6.1 and MF6.2 equations
        // Paper definition :
        // My = -R0 * Fz0 * LMY *
        //     (QSY1 + QSY2 * (Fx / Fz0) + QSY3 * abs(Vcx / V0) + QSY4 * pow(Vcx / V0, 4) +
        //      (QSY5 + QSY6 * (Fz_unlimited / Fz0)) * pow(gamma, 2)) *
        //     (pow(Fz_unlimited / Fz0, QSY7) * pow(p / pi0, QSY8));  // (A48)

        My = -R0 * Fz0 * LMY *
             (QSY1 + QSY2 * (Fx / Fz0) + QSY3 * abs(Vcx / V0) + QSY4 * pow(Vcx / V0, 4) +
              (QSY5 + QSY6 * (Fz / Fz0)) * pow(gamma, 2)) *
             (pow(Fz / Fz0, QSY7) * pow(p / pi0, QSY8));  // (A48)
    }

    return My * m_fz_scl;
}

double ChMFTire::CalculateMz(double Fz,
                             double kappa,
                             double alpha,
                             double alpha_prime,
                             double gamma,
                             double dfz,
                             double dpi,
                             double phi,
                             double Fx,
                             double Fy,
                             double Vcx) {
    // TODO: This is incorrect and should be calculated as follows:
    // cos_alpha_prime = Vcy / fmax(sqrt(Vcx * Vcx + Vcy * Vcy), 0.1)
    // auto cos_alpha_prime = cos(alpha);
    auto cos_alpha_prime = alpha_prime;

    // Define epsilons
    const double epsilon = 1e-6;  // [Eqn(4.E6a) Page 178 - Book]
    const double epsilonk = epsilon;

    const double pi = atan(1.0) * 4;

    // TODO: Validate this assumption, not sure if this is correct.]
    // From MFEval --> phit = phit * cos(alpha); % Empirically discovered
    auto phit = phi;

    // Retrieve internal variables
    auto By = m_By;
    auto Cy = m_Cy;
    auto Gyk = m_Gyk;
    auto muy = m_muy;
    auto Kxk = m_Kxk;
    auto Kya = m_Kya;
    auto SHy = m_SHy;
    auto SVy = m_SVy;

    auto SVyk = m_SVyk;

    // auto Fy0_sub0 = m_Fy0_sub0;
    // auto Gyk_sub0 = m_Gyk_sub0;

    auto gamma_star = gamma;
    auto alpha_star = alpha;

    // [VERTICAL]
    auto Fz0 = par.FNOMIN;  // Nominal wheel load

    // [DIMENSION]
    auto R0 = par.UNLOADED_RADIUS;  // Free tyre radius

    // [SCALING_COEFFICIENTS]
    auto LKY = par.LKY;    // Scale factor of Fy cornering stiffness
    auto LTR = par.LTR;    // Scale factor of peak of pneumatic trail
    auto LRES = par.LRES;  // Scale factor for offset of residual torque
    auto LKZC = par.LKZC;  // Scale factor of camber torque stiffness
    auto LMP = par.LMP;

    // [ALIGNING_COEFFICIENTS]
    auto QBZ1 = par.QBZ1;    // Trail slope factor for trail Bpt at Fznom
    auto QBZ2 = par.QBZ2;    // Variation of slope Bpt with load
    auto QBZ3 = par.QBZ3;    // Variation of slope Bpt with load squared
    auto QBZ4 = par.QBZ4;    // Variation of slope Bpt with camber
    auto QBZ5 = par.QBZ5;    // Variation of slope Bpt with absolute camber
    auto QBZ9 = par.QBZ9;    // Factor for scaling factors of slope factor Br of Mzr
    auto QBZ10 = par.QBZ10;  // Factor for dimensionless cornering stiffness of Br of Mzr
    auto QBRP1 = par.QBRP1;
    auto QCRP1 = par.QCRP1;
    auto QDRP1 = par.QDRP1;
    auto QCRP2 = par.QCRP2;
    auto QCZ1 = par.QCZ1;    // Shape factor Cpt for pneumatic trail
    auto QDZ1 = par.QDZ1;    // Peak trail Dpt = Dpt.*(Fz. / Fznom.*R0)
    auto QDZ2 = par.QDZ2;    // Variation of peak Dpt" with load
    auto QDZ3 = par.QDZ3;    // Variation of peak Dpt" with camber
    auto QDZ4 = par.QDZ4;    // Variation of peak Dpt" with camber squared
    auto QDZ6 = par.QDZ6;    // Peak residual torque Dmr" = Dmr./(Fz.*R0)
    auto QDZ7 = par.QDZ7;    // Variation of peak factor Dmr" with load
    auto QDZ8 = par.QDZ8;    // Variation of peak factor Dmr" with camber
    auto QDZ9 = par.QDZ9;    // Variation of peak factor Dmr" with camber and load
    auto QDZ10 = par.QDZ10;  // Variation of peak factor Dmr with camber squared
    auto QDZ11 = par.QDZ11;  // Variation of Dmr with camber squaredand load
    auto QEZ1 = par.QEZ1;    // Trail curvature Ept at Fznom
    auto QEZ2 = par.QEZ2;    // Variation of curvature Ept with load
    auto QEZ3 = par.QEZ3;    // Variation of curvature Ept with load squared
    auto QEZ4 = par.QEZ4;    // Variation of curvature Ept with sign of Alpha - t
    auto QEZ5 = par.QEZ5;    // Variation of Ept with camberand sign Alpha - t
    auto QHZ1 = par.QHZ1;    // Trail horizontal shift Sht at Fznom
    auto QHZ2 = par.QHZ2;    // Variation of shift Sht with load
    auto QHZ3 = par.QHZ3;    // Variation of shift Sht with camber
    auto QHZ4 = par.QHZ4;    // Variation of shift Sht with camberand load
    auto PPZ1 = par.PPZ1;    // effect of inflation pressure on length of pneumatic trail
    auto PPZ2 = par.PPZ2;    // Influence of inflation pressure on residual aligning torque

    auto QDTP1 = par.QDTP1;

    // [SCALING_COEFFICIENTS]
    auto LS = par.LS;      // Scale factor of moment arm of Fx
    auto LFZO = par.LFZO;  // Scale factor of nominal(rated) load

    // [ALIGNING_COEFFICIENTS]
    auto SSZ1 = par.SSZ1;  // Nominal value of s. / R0: effect of Fx on Mz
    auto SSZ2 = par.SSZ2;  // Variation of distance s. / R0 with Fy. / Fznom
    auto SSZ3 = par.SSZ3;  // Variation of distance s. / R0 with camber
    auto SSZ4 = par.SSZ4;  // Variation of distance s. / R0 with load and camber

    auto PECP1 = par.PECP1;
    auto PECP2 = par.PECP2;

    // Pre-calculate
    auto Fz0_prime = par.LFZO * par.FNOMIN;  // [Eqn(4.E1) Page 177 - Book]

    auto SHt = QHZ1 + QHZ2 * dfz + (QHZ3 + QHZ4 * dfz) * gamma_star;  // (4.E35)

    // signKya = sign(Kya);
    // signKya(signKya == 0) = 1; // If[Kya = 0] then[sign(0) = 0].This is done to avoid[num / 0 = NaN] in Eqn 4.E38
    auto signKya = signp(Kya);

    auto Kya_prime = Kya + epsilonk * signKya;  // (4.E39)[sign(Kya) term explained on page 177]
    auto SHf = SHy + SVy / Kya_prime;           // (4.E38)

    auto alphar = alpha_star + SHf;  // = alphaf(4.E37)
    auto alphat = alpha_star + SHt;  // (4.E34)

    double zeta0 = 1.0;

    double zeta2 = 1.0;
    double zeta5 = 1.0;
    double zeta6 = 1.0;
    double zeta7 = 1.0;
    double zeta8 = 1.0;

    if (useTurnSlip) {
        zeta5 = cos(atan(QDTP1 * R0 * phi));  // [Eqn(4.91) Page 186 - Book]
    }

    // Dt0 = Fz.*(R0. / Fz0_prime).*(QDZ1 + QDZ2.*dfz).*(1 - PPZ1.*dpi).*LTR.*sign(Vcx);  // (4.E42)
    // Dt = Dt0.*(1 + QDZ3.*abs(gamma_star) + QDZ4.*gamma_star. ^ 2).*zeta5;  // (4.E43)
    //
    // IMPORTANT NOTE : The above original equation(4.E43) was not matching the
    // TNO solver.The coefficient Dt affects the pneumatic trail(t) and the
    // self aligning torque(Mz).
    // It was observed that when negative inclination angles where used as
    // inputs, there was a discrepancy between the TNO solverand mfeval.
    // This difference comes from the term QDZ3, that in the original equation
    // is multiplied by abs(gamma_star).But in the paper the equation is
    // differentand the abs() term is not written.Equation(A60) from the
    // paper resulted into a perfect match with TNO.
    // Keep in mind that the equations from the paper don't include turn slip
    // effects.The term zeta5 has been added although it doesn't appear in the
    // paper.

    // Paper definition :
    auto Dt = (QDZ1 + QDZ2 * dfz) * (1.0 - PPZ1 * dpi) * (1.0 + QDZ3 * gamma + QDZ4 * pow(gamma, 2)) * Fz *
              (R0 / Fz0_prime) * LTR * zeta5;  // (A60)

    // Bt = (QBZ1 + QBZ2.*dfz + QBZ3.*dfz. ^ 2).*(1 + QBZ5.*abs(gamma_star) + QBZ6.*gamma_star. ^ 2).*LKY. / LMUY_star;
    // // (> 0)(4.E40)
    //
    // IMPORTANT NOTE : In the above original equation(4.E40) it is used the
    // parameter QBZ6, which doesn't exist in the standard TIR files. Also note
    // that on page 190 and 615 of the book a full set of parameters is given
    // andQBZ6 doesn't appear.
    // The equation has been replaced with equation(A58) from the paper.

    // Paper definition :

    auto Bt = (QBZ1 + QBZ2 * dfz + QBZ3 * pow(dfz, 2)) * (1.0 + QBZ4 * gamma + QBZ5 * abs(gamma)) * LKY /
              LMUY_star;  // (> 0) (A58)
    auto Ct = QCZ1;       // (> 0) (4.E41)
    auto Et = fmin((QEZ1 + QEZ2 * dfz + QEZ3 * pow(dfz, 2)) *
                       (1.0 + (QEZ4 + QEZ5 * gamma_star) * (2 / pi) * atan(Bt * Ct * alphat)),
                   1.0);  // (<= 1) (4.E44)

    // auto t0 = Dt * cos(Ct * atan(Bt * alphat - Et * (Bt * alphat - atan(Bt * alphat)))) * cos(alpha_prime); //
    // t(aplhat)(4.E33)
    // auto t0 = Dt * cos(Ct * atan(Bt * alphat - Et * (Bt * alphat - atan(Bt * alphat)))) * cos_alpha_prime; //
    // t(aplhat)(4.E33)

    // Evaluate Fy0 with gamma = 0 and phit = 0
    // This is done already in the Fy calculation routine

    // auto Mzo_prime = -t0 * Fy0_sub0; // gamma = phi = 0 (4.E32)

    if (useTurnSlip) {
        zeta0 = 0.0;

        // zeta2 = internalParams.zeta2; // TODO: Get Zeta2 from somewhere!!
        // phi = postProInputs.phi;

        zeta6 = cos(atan(QBRP1 * R0 * phi));  // [Eqn(4.102) Page 188 - Book]

        // [Fy0, muy, ~, ~, ~, ~, ~, ~, ~] = obj.calculateFy0(tirParams, postProInputs, internalParams, modes, starVar,
        // primeVar, incrVar);

        // epsilong = internalParams.epsilong;
        // phit = postProInputs.phit;

        auto epsilong = PECP1 * (1.0 + PECP2 * dfz);  // [Eqn(4.90) Page 186 - Book] Camber reduction factor

        auto Mzp_inf = QCRP1 * abs(muy) * R0 * Fz * sqrt(Fz / Fz0_prime) * LMP;  // [Eqn(4.95) Page 187 - Book]
        // Mzp_inf should be always > 0
        // negative_Mzp_inf = Mzp_inf < 0;
        // Mzp_inf(negative_Mzp_inf) = 1e-6;
        Mzp_inf = Mzp_inf < 0.0 ? 1.0e-6 : Mzp_inf;

        auto CDrp = QDRP1;                           // (> 0)[Eqn(4.96) Page 187 - Book]
        auto DDrp = Mzp_inf / sin(0.5 * pi * CDrp);  // [Eqn(4.94) Page 187 - Book]
        auto Kzgr0 =
            Fz * R0 * (QDZ8 * QDZ9 * dfz + (QDZ10 + QDZ11 * dfz * abs(gamma))) * LKZC;  // [Eqn(4.99) Page 187 - Book]
        auto BDrp = Kzgr0 / (CDrp * DDrp * (1.0 - epsilong));                           // Eqn from the manual
        auto Drp = DDrp * sin(CDrp * atan(BDrp * R0 * (phit)));                         // Eqn from the manual

        // [~, Gyk] = obj.calculateFy(tirParams, postProInputs, internalParams, modes, starVar, incrVar, Fy0, muy);

        auto Mzp90 = Mzp_inf * (2.0 / pi) * atan(QCRP2 * R0 * abs(phit)) * Gyk;  // [Eqn(4.103) Page 188 - Book]

        zeta7 = (2.0 / pi) * acos(Mzp90 / (abs(Drp)));  // Eqn from the manual
        zeta8 = 1.0 + Drp;
    }

    // auto Dr = Fz * R0 * ((QDZ6 + QDZ7 * dfz) * LRES * zeta2 + ((QDZ8 + QDZ9 * dfz) * (1.0 + PPZ2 * dpi)
    //     + (QDZ10 + QDZ11 * dfz) * abs(gamma_star)) * gamma_star * LKZC * zeta0) * LMUY_star * sgn(Vcx) *
    //     cos(alpha_star) +
    //           zeta8 - 1.0;                                         // % (4.E47)

    auto Dr = Fz * R0 *
                  ((QDZ6 + QDZ7 * dfz) * LRES * zeta2 +
                   ((QDZ8 + QDZ9 * dfz) * (1.0 + PPZ2 * dpi) + (QDZ10 + QDZ11 * dfz) * abs(gamma_star)) * gamma_star *
                       LKZC * zeta0) *
                  LMUY_star * signp(Vcx) * cos(alpha_star) +
              zeta8 - 1.0;  // % (4.E47)

    auto Br = (QBZ9 * LKY / LMUY_star + QBZ10 * By * Cy) * zeta6;  //% preferred: qBz9 = 0 (4.E45)
    auto Cr = zeta7;                                               // % (4.E46)
    // auto Mzr0 = Dr * cos(Cr * atan(Br * alphar)) * cos(alpha_prime); // % = Mzr(alphar)(4.E36)
    // auto Mzr0 = Dr * cos(Cr * atan(Br * alphar)) * cos_alpha_prime; // % = Mzr(alphar)(4.E36)
    // auto Mz0 = Mzo_prime + Mzr0; // % (4.E31)

    // alphar_eq = sqrt(alphar. ^ 2 + (Kxk. / Kya_prime). ^ 2. * kappa. ^ 2).*sign(alphar); % (4.E78)
    // alphat_eq = sqrt(alphat. ^ 2 + (Kxk. / Kya_prime). ^ 2. * kappa. ^ 2).*sign(alphat); % (4.E77)
    // s = R0.*(SSZ1 + SSZ2.*(Fy. / Fz0_prime) + (SSZ3 + SSZ4.*dfz).*gamma_star).*LS; % (4.E76)

    // IMPORTANT NOTE : The equations 4.E78 and 4.E77 are not used due to small
    // differences discovered at negative camber angles with the TNO solver.
    // Instead equations A54 and A55 from the paper are used.
    //
    // IMPORTANT NOTE : The coefficient "s" (Equation 4.E76) determines the
    // effect of Fx into Mz.The book uses "Fz0_prime" in the formulation,
    // but the paper uses "Fz0".The equation(A56) from the paper has a better
    // correlation with TNO.

    // auto alphar_eq = atan(sqrt(pow(tan(alphar), 2) + pow(Kxk / Kya_prime, 2) * pow(kappa, 2))) * sgn(alphar);  // %
    // (A54) auto alphat_eq = atan(sqrt(pow(tan(alphat), 2) + pow(Kxk / Kya_prime, 2) * pow(kappa, 2))) * sgn(alphat);
    // // % (A55)
    auto alphar_eq =
        atan(sqrt(pow(tan(alphar), 2) + pow(Kxk / Kya_prime, 2) * pow(kappa, 2))) * signp(alphar);  // % (A54)
    auto alphat_eq =
        atan(sqrt(pow(tan(alphat), 2) + pow(Kxk / Kya_prime, 2) * pow(kappa, 2))) * signp(alphat);  // % (A55)
    auto s = R0 * (SSZ1 + SSZ2 * (Fy / Fz0) + (SSZ3 + SSZ4 * dfz) * gamma) * LS;                    // % (A56)
    auto Mzr = Dr * cos(Cr * atan(Br * alphar_eq));                                                 // % (4.E75)

    // Note: in the above equation starVar is used instead of
    // starVar_sub0 beacuse it was found a better match with TNO
    auto Fy_prime = m_Fy_prime;
    // auto Fy_prime = Gyk_sub0 * Fy0_sub0;  // % (4.E74)
    // auto t = Dt * cos(Ct * atan(Bt * alphat_eq - Et * (Bt * alphat_eq - atan(Bt * alphat_eq)))) * cos(alpha_prime);
    // // % (4.E73)
    auto t = Dt * cos(Ct * atan(Bt * alphat_eq - Et * (Bt * alphat_eq - atan(Bt * alphat_eq)))) *
             cos_alpha_prime;  // % (4.E73)
    t = t * LFZO;

    // IMPORTANT NOTE : the above equation is not written in any source, but "t"
    // is multiplied by LFZO in the TNO dteval function.This has been empirically
    // discovered.

    // Low speed model
    // TODO: Handle low speed
    /*
    auto isLowSpeed = modes.isLowSpeed;
    if (isLowSpeed) {  // % Line for Simulink
        t = t * reductionSmooth;
        Mzr = Mzr * reductionSmooth;
    }  // end% if isLowSpeed
    */

    double Mz;
    // Check MF version
    if (par.FITTYP == 6 || par.FITTYP == 21) {
        // MF5.2 equations
        Mz = -t * (Fy - SVyk) + Mzr + s * Fx;  //%% From the MF - Tyre equation manual
    } else {
        // MF6.1 and 6.2 equatons
        auto Mz_prime = -t * Fy_prime;  // % (4.E72)
        Mz = Mz_prime + Mzr + s * Fx;   // % (4.E71)
    }

    // Gyroscopic couple ==> Is already simulated by wheel/tire inertia (??)
    /*
    auto alpha_dot = 0.0;
    auto Vr = 0.0;
    auto Mgyr = QTZ1 * LGYR * MBELT * Vr * alpha_dot * cos(zeta *  atan(Br + alphar_eq));
    */

    return Mz * m_fz_scl;
}

}  // end namespace vehicle
}  // namespace chrono
