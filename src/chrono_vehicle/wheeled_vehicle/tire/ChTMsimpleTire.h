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
// Template for TMsimple tire model
//
// =============================================================================

#ifndef CH_SIMPLETIRE_H
#define CH_SIMPLETIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// TMsimple based tire model.
class CH_VEHICLE_API ChTMsimpleTire : public ChForceElementTire {
  public:
    ChTMsimpleTire(const std::string& name);

    virtual ~ChTMsimpleTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TMsimpleTire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire width.
    /// For a TMsimple tire, this is the unloaded tire radius.
    virtual double GetRadius() const override { return m_unloaded_radius; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_width; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_width; }

    /// Get the tire slip angle computed internally by the TMsimple model (in radians).
    /// The reported value will be similar to that reported by ChTire::GetSlipAngle.
    double GetSlipAngle_internal() const { return m_states.alpha; }

    /// Get the tire longitudinal slip computed internally by the TMsimple model.
    /// The reported value will be different from that reported by ChTire::GetLongitudinalSlip
    /// because ChTMsimpleTire uses the loaded tire radius for its calculation.
    double GetLongitudinalSlip_internal() const { return m_states.kappa; }

    /// Get the camber angle for the TMsimple tire model (in radians).
    /// ChTMsimpleTire does not calculate its own camber angle. This value is the same as that
    /// reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() { return GetCamberAngle(); }

    /// Get the tire deflection.
    virtual double GetDeflection() const override { return m_data.depth; }

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

    /// Generate basic tire plots.
    /// This function creates a Gnuplot script file with the specified name.
    void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

  protected:
    /// Set the parameters in the TMsimple model.
    virtual void SetTMsimpleParams() = 0;
    
    void GenerateWorkParameters();

    /// Calculate Patch Forces
    void TMsimplePatchForces(double& fx, double& fy, double kappa, double alpha, double fz);

    /// TMsimple tire model parameters

    double m_unloaded_radius;
    double m_width;
    double m_rolling_resistance;

    double m_mu;    ///< Actual friction coefficient of the road
    double m_mu_0;  ///< Local friction coefficient of the road for given parameters

    double m_vcoulomb; // full sliding velocity with coulomb friction

    double m_Fz_nom; // nominal tire load force (N)
    
    // TMsimple input parameters
    // Fx at Fz = Fz_nom
    double m_Fx_max1;   // longitudinal force maximum (N)
    double m_Fx_inf1;   // force at infinite longitudinal slip (N)
    double m_dFx0_1;    // slope of Fx curve at longitunal slip = 0 (N)
    // Fx at Fz = 2*Fz_nom
    double m_Fx_max2;   // longitudinal force maximum (N)
    double m_Fx_inf2;   // force at infinite longitudinal slip (N)
    double m_dFx0_2;    // slope of Fx curve at longitunal slip = 0 (N)
    // Fy at Fz = Fz_nom
    double m_Fy_max1;   // lateral force maximum (N)
    double m_Fy_inf1;   // force at infinite lateral slip (N)
    double m_dFy0_1;    // slope of Fx curve at lateral slip = 0 (N)
    // Fy at Fz = 2*Fz_nom
    double m_Fy_max2;   // lateral force maximum (N)
    double m_Fy_inf2;   // force at infinite lateral slip (N)
    double m_dFy0_2;    // slope of Fx curve at lateral slip = 0 (N)

    // vertical spring constant
    double m_Cz;
    // vertical damping constant
    double m_Dz;

    // TMsimple work parameters
    double m_ax1, m_ax2;
    double m_bx1, m_bx2;
    double m_cx1, m_cx2;
    double m_ay1, m_ay2;
    double m_by1, m_by2;
    double m_cy1, m_cy2;

    double m_time;        // actual system time
    double m_time_trans;  // end of start transient

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    struct TireStates {
        double kappa;   // Contact Path - Stationary Longitudinal Slip State (Kappa)
        double alpha;   // Contact Path - Stationary Side Slip State (Alpha)
        double abs_vx;  // Longitudinal speed
        double abs_vt;  // Longitudinal transport speed
        double vta;     // absolute transport speed
        double vsx;     // Longitudinal slip velocity
        double vsy;     // Lateral slip velocity = Lateral velocity
        double omega;   // Wheel angular velocity about its spin axis (temporary for debug)
        double sx;      // Longitudinal Slip
        double sy;      // Lateral slip
        double Reff;    // effective roll radius
        ChVector<> disc_normal;  // temporary for debug
    };

    TireStates m_states;
    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
