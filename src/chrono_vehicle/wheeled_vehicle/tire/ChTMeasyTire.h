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
//      H. Olsson, K. Astrom e. a., "Friction Modeling and Compensation", Universities of Lund (S), Grenoble (F) and
//      Merida (VE), 1997
//
// Known differences to the commercial version:
//  - No parking slip calculations
//  - No dynamic parking torque
//  - No dynamic tire inflation pressure
//  - No belt dynamics
//  - Simplified stand still handling
//  - Optional tire contact smoothing based on "A New Analytical Tire Model for Vehicle Dynamic Analysis" by
//      J. Shane Sui & John A Hirshey II
//  - Standstill algorithm based on Dahl Friction Model with Damping
//
// This implementation has been validated with:
//  - FED-Alpha vehicle model
//  - Tire data sets gained by conversion of Pac02 TIR parameter files
//  - Steady state cornering test and test results from Keweenah Research Center (KRC)
//  - unvalidateble functionality has been removed
// ===================================================================================

#ifndef CH_TMEASY_TIRE
#define CH_TMEASY_TIRE

#include <vector>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChTMeasyTire : public ChForceElementTire {
  public:
    ChTMeasyTire(const std::string& name);

    virtual ~ChTMeasyTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TMeasyTire"; }

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_width; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const override { return m_width; }

    /// Get the tire slip angle computed internally by the TMsimple model (in radians).
    /// The reported value will be similar to that reported by ChTire::GetSlipAngle.
    double GetSlipAngle_internal() const { return atan(-m_states.sy); }

    /// Get the tire longitudinal slip computed internally by the TMsimple model.
    /// The reported value will be similar to that reported by ChTire::GetLongitudinalSlip.
    double GetLongitudinalSlip_internal() const { return m_states.sx; }

    double GetTireOmega() { return m_states.omega; }

    /// Get maximum tire load from Load Index (LI) in N [0:279].
    static double GetTireMaxLoad(unsigned int li);

    /// Guess Tire Parameters from characteristic truck tire parameter pattern (Ratio = 80%).
    void GuessTruck80Par(unsigned int li,            ///< tire load index
                         double tireWidth,           ///< tire width [m]
                         double ratio,               ///< use 0.75 meaning 75%
                         double rimDia,              ///< rim diameter [m]
                         double pinfl_li = 1.0,      ///< inflation pressure at load index
                         double pinfl_use = 1.0,     ///< inflation pressure in this configuration
                         double damping_ratio = 0.5  ///< scaling factor for normal damping coefficient
    );

    void GuessTruck80Par(double loadForce,           ///< tire nominal load force [N]
                         double tireWidth,           ///< tire width [m]
                         double ratio,               ///< use 0.75 meaning 75%
                         double rimDia,              ///< rim diameter [m]
                         double pinfl_li = 1.0,      ///< inflation pressure at load index
                         double pinfl_use = 1.0,     ///< inflation pressure in this configuration
                         double damping_ratio = 0.5  ///< scaling factor for normal damping coefficient
    );

    /// Guess Tire Parameters from characteristic passenger car tire parameter pattern (Ratio = 70%).
    void GuessPassCar70Par(unsigned int li,            ///< tire load index
                           double tireWidth,           ///< tire width [m]
                           double ratio,               ///< use 0.75 meaning 75%
                           double rimDia,              ///< rim diameter [m]
                           double pinfl_li = 1.0,      ///< inflation pressure at load index
                           double pinfl_use = 1.0,     ///< inflation pressure in this configuration
                           double damping_ratio = 0.5  ///< scaling factor for normal damping coefficient
    );

    void GuessPassCar70Par(double loadForce,           ///< tire nominal load force [N]
                           double tireWidth,           ///< tire width [m]
                           double ratio,               ///< use 0.75 meaning 75%
                           double rimDia,              ///< rim diameter [m]
                           double pinfl_li = 1.0,      ///< inflation pressure at load index
                           double pinfl_use = 1.0,     ///< inflation pressure in this configuration
                           double damping_ratio = 0.5  ///< scaling factor for normal damping coefficient
    );

    /// Set vertical tire stiffness as linear function by coefficient [N/m].
    void SetVerticalStiffness(double Cz) {
        m_d1 = Cz;
        m_d2 = 0;
    }

    /// Set vertical tire stiffness as nonlinear function by coefficients at nominal load 1 [N/m]
    /// and nominal load 2 [N/m].
    /// void SetVerticalStiffness(double Cz1, double Cz2);

    /// Set vertical tire stiffness as nonlinear function by calculation from tire test data (least squares).
    void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

    /// Set the tire reference coefficient of friction.
    void SetFrictionCoefficient(double coeff);

    /// Set rolling resistance coefficients (default: 0.01).
    void SetRollingResistanceCoefficient(double rr_coeff);

    /// Generate basic tire plots.
    /// This function creates a Gnuplot script file with the specified name.
    void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

    /// Get the tire deflection.
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Simple parameter consistency test.
    bool CheckParameters();

  protected:
    /// Set the parameters in the TMsimple model.
    virtual void SetTMeasyParams() = 0;

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const override final;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const override final;

    bool m_use_startup_transition;

    double m_time;
    double m_begin_start_transition;
    double m_end_start_transition;

    double m_vnum;

    double m_gamma_limit;  ///< limit camber angle (degrees!)

    // TMsimple tire model parameters
    double m_unloaded_radius;     ///< reference tire radius
    double m_width;               ///< tire width
    double m_rim_radius;          ///< tire rim radius
    double m_bottom_radius;       ///< radius where tire bottoming begins
    double m_bottom_stiffness;    ///< stiffness of the tire/bottom contact
    double m_rolling_resistance;  ///< actual rolling friction coeff

    double m_d1;  ///< polynomial coefficient for stiffness interpolation, linear
    double m_d2;  ///< polynomial coefficient for stiffness interpolation, quadratic

    double m_vcoulomb;
    double m_frblend_begin;
    double m_frblend_end;

    VehicleSide m_measured_side;

    struct TMeasyCoeff {
        double pn;      ///< Nominal vertical force [N]
        double pn_max;  ///< Maximum vertical force [N]

        double mu_0;  ///< Local friction coefficient of the road for given parameters
        double dz;    ///< Linear damping coefficient z [Ns/m]

        double dfx0_pn, dfx0_p2n;  ///< Initial longitudinal slopes dFx/dsx [kN]
        double sxm_pn, sxm_p2n;    ///< longitudinal slip at Maximum longitudinal force []
        double fxm_pn, fxm_p2n;    ///< Maximum longitudinal force [kN]
        double sxs_pn, sxs_p2n;    ///< Longitudinal slip when pure sliding begins []
        double fxs_pn, fxs_p2n;    ///< Longitudinal load at sliding [kN]

        double dfy0_pn, dfy0_p2n;  ///< Initial lateral slopes dFy/dsy [kN]
        double sym_pn, sym_p2n;    ///< lateral slip at Maximum longitudinal force []
        double fym_pn, fym_p2n;    ///< Maximum lateral force [kN]
        double sys_pn, sys_p2n;    ///< Lateral slip when pure sliding begins []
        double fys_pn, fys_p2n;    ///< Lateral load at sliding [kN]

        double nL0_pn, nL0_p2n;  ///< dimensionless alignment lever
        double sq0_pn, sq0_p2n;  ///< lateral slip, where lever is zero
        double sqe_pn, sqe_p2n;  ///< lever after sliding is reached

        double sigma0{100000.0};  ///< bristle stiffness for Dahl friction model
        double sigma1{5000.0};    ///< bristle damping for Dahl friction model
    };

    TMeasyCoeff m_par;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    void CombinedCoulombForces(double& fx, double& fy, double fz, double muscale);
    void tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs);
    double AlignmentTorque(double fy);

    // linear interpolation
    double InterpL(double w1, double w2) { return w1 + (w2 - w1) * (m_states.q - 1.0); };
    // quadratic interpolation
    double InterpQ(double w1, double w2) {
        return (m_states.q) * (2.0 * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (m_states.q));
    };

    struct TireStates {
        double sx;               // Longitudinal Slip State (sx)
        double sy;               // Side Slip State (sy)
        double q;                // Fz/Fz_nom
        double gamma;            // Inclination Angle
        double muscale;          // Scaling factor for Tire/Road friction
        double vta;              // absolut transport velocity
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis, filtered by running avg,
        double R_eff;            // Effective Rolling Radius
        double P_len;            // Length of contact patch
        double dfx0;             // dfx/dx at Fz (sx = 0)
        double sxm;              // sxm at Fz
        double fxm;              // Fxm at Fz
        double sxs;              // sxs at Fz
        double fxs;              // Fxs at Fz
        double dfy0;             // dfy/dy at Fz (sy = 0)
        double sym;              // sym at Fz
        double fym;              // Fym at Fz
        double sys;              // sys at Fz
        double fys;              // Fys at Fz
        double nL0;              // Dimensionless lever at actual load level
        double sq0;              // Zero crossing at actual load level
        double sqe;              // Zero after complete sliding at actual load level
        double brx{0};           // bristle deformation x
        double bry{0};           // bristle deformation y
        ChVector<> disc_normal;  // (temporary for debug)
    };

    TireStates m_states;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
