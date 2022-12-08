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
// Changes:
// 2017-12-21 - There is a simple form of contact smoothing now. It works on flat
//			    terrain as well.
//			  - The parameter estimation routines have changed, know you have the
//				option to use either the load index or the load force as input.
//
// 2018-02-22 - Tire Relaxation is considered now. No user input is needed.
//              The tire step_size should be the same as for the MBS.
//
// 2018-02-24 - Calculation of tire rolling radius with user parameters
//            - Export of tire parameters into a parameter file now possible
// =============================================================================

#ifndef CH_TMEASYTIRE
#define CH_TMEASYTIRE

#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// TMeasy tire model.
/// The Chrono implementation is a basic version of the commercial models available at
/// <a href="http://www.tmeasy.de/">www.tmeasy.de</a>.
class CH_VEHICLE_API ChTMeasyTire : public ChForceElementTire {
  public:
    ChTMeasyTire(const std::string& name);

    virtual ~ChTMeasyTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TMeasyTire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_width; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_width; }

    /// Get the tire slip angle computed internally by the TMeasy model (in radians).
    /// The reported value will be similar to that reported by ChTire::GetSlipAngle.
    double GetSlipAngle_internal() const { return atan(-m_states.sy); }

    /// Get the tire longitudinal slip computed internally by the TMeasy model.
    /// The reported value will be similar to that reported by ChTire::GetLongitudinalSlip.
    double GetLongitudinalSlip_internal() const { return m_states.sx; }

    /// Get the camber angle computed internally by the TMeasy model (in radians).
    /// The reported value will be similar to that reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() { return m_gamma; }

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
    void SetVerticalStiffness(double Cz) { SetVerticalStiffness(Cz, Cz); }

    /// Set vertical tire stiffness as nonlinear function by coefficients at nominal load 1 [N/m]
    /// and nominal load 2 [N/m].
    void SetVerticalStiffness(double Cz1, double Cz2);

    /// Set vertical tire stiffness as nonlinear function by calculation from tire test data (least squares).
    void SetVerticalStiffness(std::vector<double>& defl, std::vector<double>& frc);

    /// Set the tire reference coefficient of friction.
    void SetFrictionCoefficient(double coeff);

    /// Set rolling resistance coefficients.
    void SetRollingResistanceCoefficients(double rr_coeff_1, double rr_coeff_2);

    /// Set dynamic radius coefficients.
    void SetDynamicRadiusCoefficients(double rdyn_coeff_1, double rdyn_coeff_2);

    /// Generate basic tire plots.
    /// This function creates a Gnuplot script file with the specified name.
    void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

    /// Get the tire deflection.
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Using tire relaxation, we have three tire deflections
    ChVector<> GetDeflection3() { return ChVector<>(m_states.xe, m_states.ye, m_data.depth); }

    /// Export a TMeasy Tire Parameter File.
    void ExportParameterFile(std::string fileName);

    /// Export a TMeasy Tire Parameter File in JSON format.
    void ExportJSONFile(std::string jsonFileName);

    /// Simple parameter consistency test.
    bool CheckParameters();

  protected:
    /// Set the parameters in the TMeasy model.
    virtual void SetTMeasyParams() = 0;

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const override final;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const override final;

    bool m_consider_relaxation;

    bool m_use_Reff_fallback_calculation;

    bool m_use_startup_transition;

    unsigned int m_integration_method;

    double m_time;
    double m_begin_start_transition;
    double m_end_start_transition;

    double m_vnum;

    double m_gamma;  ///< actual camber angle

    double m_gamma_limit;  ///< limit camber angle (degrees!)

    // TMeasy tire model parameters
    double m_unloaded_radius;     ///< reference tire radius
    double m_width;               ///< tire width
    double m_rim_radius;          ///< tire rim radius
    double m_roundness;           ///< roundness factor for cross-section profile
    double m_rolling_resistance;  ///< actual rolling friction coeff
    double m_mu;                  ///< local friction coefficient of the road

    double m_a1;  ///< polynomial coefficient a1 in cz = a1 + 2.0*a2 * deflection
    double m_a2;  ///< polynomial coefficient a2 in cz = a1 + 2.0*a2 * deflection

    double m_tau_x;  ///< Longitudinal relaxation delay time
    double m_tau_y;  ///< Lateral relaxation delay time

    double m_relaxation_lenght_x;    ///< Longitudinal relaxation length
    double m_relaxation_lenght_y;    ///< Lateral relaxation length
    double m_relaxation_lenght_phi;  ///< Relaxation length for bore movement

    double m_rdynco;          ///< actual value of dynamic rolling radius weighting coefficient
    double m_rdynco_crit;     ///< max. considered value of m_rdynco (local minimum of dynamic rolling radius)
    double m_fz_rdynco_crit;  ///< Fz value r_dyn = r_dyn(m_fz_rdynco,m_rdynco_crit)

    VehicleSide m_measured_side;

    typedef struct {
        double pn;      ///< Nominal vertical force [N]
        double pn_max;  ///< Maximum vertical force [N]

        double mu_0;  ///< Local friction coefficient of the road for given parameters
        double cx;    ///< Linear stiffness x [N/m]
        double cy;    ///< Linear stiffness y [N/m]
        double cz;    ///< Stiffness, may vary with the vertical force [N/m]
        double dx;    ///< Linear damping coefficient x [Ns/m]
        double dy;    ///< Linear damping coefficient y [Ns/m]
        double dz;    ///< Linear damping coefficient z [Ns/m]

        double dfx0_pn, dfx0_p2n;  ///< Initial longitudinal slopes dFx/dsx [kN]
        double fxm_pn, fxm_p2n;    ///< Maximum longitudinal force [kN]
        double fxs_pn, fxs_p2n;    ///< Longitudinal load at sliding [kN]
        double sxm_pn, sxm_p2n;    ///< Slip sx at maximum longitudinal load Fx
        double sxs_pn, sxs_p2n;    ///< Slip sx where sliding begins

        double dfy0_pn, dfy0_p2n;  ///< Initial lateral slopes dFy/dsy [kN]
        double fym_pn, fym_p2n;    ///< Maximum lateral force [kN]
        double fys_pn, fys_p2n;    ///< Lateral load at sliding [kN]
        double sym_pn, sym_p2n;    ///< Slip sy at maximum lateral load Fy
        double sys_pn, sys_p2n;    ///< Slip sy where sliding begins

        double nto0_pn, nto0_p2n;      ///< Normalized pneumatic trail at sy=0
        double synto0_pn, synto0_p2n;  ///< Slip sy where trail changes sign
        double syntoE_pn, syntoE_p2n;  ///< Slip sy where trail tends to zero

        double rrcoeff_pn, rrcoeff_p2n;  ///< Rolling resistance coefficients
        double rdynco_pn, rdynco_p2n;    ///< Dynamic radius weighting coefficients

    } TMeasyCoeff;

    TMeasyCoeff m_TMeasyCoeff;

    // linear interpolation
    double InterpL(double fz, double w1, double w2) { return w1 + (w2 - w1) * (fz / m_TMeasyCoeff.pn - 1.0); };
    // quadratic interpolation
    double InterpQ(double fz, double w1, double w2) {
        return (fz / m_TMeasyCoeff.pn) * (2.0 * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / m_TMeasyCoeff.pn));
    };

    // private:
    void UpdateVerticalStiffness();

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    std::vector<double> m_tire_test_defl;  // set, when test data are used for vertical
    std::vector<double> m_tire_test_frc;   // stiffness calculation

    void tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs);
    double tmy_tireoff(double sy, double nto0, double synto0, double syntoE);

    struct TireStates {
        double sx;               // Contact Path - Longitudinal Slip State (Kappa)
        double sy;               // Contact Path - Side Slip State (Alpha)
        double vta;              // absolut transport velocity
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis
        double R_eff;            // Effective Rolling Radius
        double Fx_dyn;           // Dynamic longitudinal tire force
        double Fy_dyn;           // Dynamic lateral tire force
        double Mb_dyn;           // Dynamic bore torque
        double xe;               // Longitudinal tire deflection
        double ye;               // Lateral tire deflection
        double xe_dot;           // Longitudinal tire deflection velocity
        double ye_dot;           // Lateral tire deflection velocity
        double Fx_struct;        // Longitudinal tire force from structural deformation
        double Fy_struct;        // Lateral tire force from structural deformation
        double Fx;               // Steady state longitudinal tire force
        double Fy;               // Steady state lateral tire force
        double Mb;               // Steady state bore torque
        ChVector<> disc_normal;  // (temporary for debug)
    };

    TireStates m_states;
    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
