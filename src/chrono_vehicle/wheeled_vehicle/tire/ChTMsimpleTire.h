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

#ifndef CH_TMSIMPLETIRE
#define CH_TMSIMPLETIRE

#include <vector>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChTMsimpleTire : public ChForceElementTire {
  public:
    ChTMsimpleTire(const std::string& name);

    virtual ~ChTMsimpleTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TMsimpleTire"; }

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
        m_d2 = 0.0;
    }

    /// Set vertical tire stiffness as nonlinear function by coefficients at nominal load 1 [N/m]
    /// and nominal load 2 [N/m].
    // void SetVerticalStiffness(double Cz1, double Cz2);

    /// Set vertical tire stiffness as nonlinear function by calculation from tire test data (least squares).
    void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

    /// Set polynomial coefficients for interpolation
    void SetHorizontalCoefficients();

    /// Set the tire reference coefficient of friction.
    void SetFrictionCoefficient(double coeff);

    /// Set rolling resistance coefficient (default: 0.01).
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
    virtual void SetTMsimpleParams() = 0;

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const override final;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const override final;

    bool m_use_startup_transition;

    unsigned int m_integration_method;

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

    double m_ax1;
    double m_ax2;
    double m_bx1;
    double m_bx2;
    double m_ay1;
    double m_ay2;
    double m_by1;
    double m_by2;
    double m_cx1;
    double m_cx2;
    double m_cy1;
    double m_cy2;

    double m_d1;  ///< polynomial coefficient for stiffness interpolation
    double m_d2;  ///< polynomial coefficient for stiffness interpolation

    double m_vcoulomb;
    double m_frblend_begin;
    double m_frblend_end;

    VehicleSide m_measured_side;

    struct TMsimpleCoeff  {
        double pn;      ///< Nominal vertical force [N]
        double pn_max;  ///< Maximum vertical force [N]

        double mu_0;  ///< Local friction coefficient of the road for given parameters
        double dz;    ///< Linear damping coefficient z [Ns/m]

        double dfx0_pn, dfx0_p2n;  ///< Initial longitudinal slopes dFx/dsx [kN]
        double fxm_pn, fxm_p2n;    ///< Maximum longitudinal force [kN]
        double fxs_pn, fxs_p2n;    ///< Longitudinal load at sliding [kN]

        double dfy0_pn, dfy0_p2n;  ///< Initial lateral slopes dFy/dsy [kN]
        double fym_pn, fym_p2n;    ///< Maximum lateral force [kN]
        double fys_pn, fys_p2n;    ///< Lateral load at sliding [kN]

        double sigma0{100000.0};  ///< bristle stiffness for Dahl friction model
        double sigma1{5000.0};    ///< bristle damping for Dahl friction model
    };

    TMsimpleCoeff m_par;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    void TMcombinedForces(double& fx, double& fy, double sx, double sy, double fz, double muscale);
    void CombinedCoulombForces(double& fx, double& fy, double fz, double muscale);

    struct TireStates {
        double sx;               // Contact Path - Longitudinal Slip State (Kappa)
        double sy;               // Contact Path - Side Slip State (Alpha)
        double gamma;            // Inclination Angle
        double muscale;          // Scaling factor for Tire/Road friction
        double vta;              // absolut transport velocity
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis, filtered by running avg,
        double R_eff;            // Effective Rolling Radius
        double P_len;            // Length of contact patch
        double brx{0};           // bristle deformation x
        double bry{0};           // bristle deformation y
        ChVector<> disc_normal;  // (temporary for debug)
    };

    TireStates m_states;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
