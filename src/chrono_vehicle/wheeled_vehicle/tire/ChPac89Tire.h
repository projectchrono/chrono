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
// Template for a PAC89 tire model
//
// =============================================================================

#ifndef CH_PAC89TIRE_H
#define CH_PAC89TIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"
#include "chrono_vehicle/ChTerrain.h"

// The following undef is required in order to compile in conda-forge
// on OSX for Python 3.6 (somehow termios.h gets included and defines
// B0 to be 0, leading to a syntax error).
// see https://github.com/projectchrono/chrono/pull/222
#undef B0

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Pacjeka 89 tire model.
class CH_VEHICLE_API ChPac89Tire : public ChForceElementTire {
  public:
    ChPac89Tire(const std::string& name);

    virtual ~ChPac89Tire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Pac89Tire"; }

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    virtual double GetWidth() const override { return m_width; }

    /// Get the tire deflection
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const override { return m_width; }

    /// Get the slip angle used in Pac89 (expressed in radians).
    /// The reported value will have opposite sign to that reported by ChTire::GetSlipAngle
    /// because ChPac89 uses internally a different frame convention.
    double GetSlipAngle_internal() const { return m_states.cp_side_slip; }

    /// Get the longitudinal slip used in Pac89.
    /// The reported value will be similar to that reported by ChTire::GetLongitudinalSlip.
    double GetLongitudinalSlip_internal() const { return m_states.cp_long_slip; }

    /// Get the camber angle used in Pac89 (expressed in radians).
    /// The reported value will be similar to that reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() { return m_gamma * CH_C_DEG_TO_RAD; }

  protected:
    /// Set the parameters in the Pac89 model.
    virtual void SetPac89Params() = 0;

    double m_kappa;  ///< longitudinal slip (percentage)
    double m_alpha;  ///< slip angle (degrees)
    double m_gamma;  ///< camber angle (degrees)

    double m_gamma_limit;  ///< limit camber angle (degrees)

    /// Road friction
    double m_mu;
    /// Tire reference friction
    double m_mu0;

    /// Pac89 tire model parameters
    double m_unloaded_radius;
    double m_width;
    double m_rolling_resistance;
    double m_lateral_stiffness;
    VehicleSide m_measured_side;

    struct PacCoeff {
        double A0;
        double A1;
        double A2;
        double A3;
        double A4;
        double A5;
        double A6;
        double A7;
        double A8;
        double A9;
        double A10;
        double A11;
        double A12;
        double A13;

        double B0;
        double B1;
        double B2;
        double B3;
        double B4;
        double B5;
        double B6;
        double B7;
        double B8;
        double B9;
        double B10;

        double C0;
        double C1;
        double C2;
        double C3;
        double C4;
        double C5;
        double C6;
        double C7;
        double C8;
        double C9;
        double C10;
        double C11;
        double C12;
        double C13;
        double C14;
        double C15;
        double C16;
        double C17;
    };

    PacCoeff m_PacCoeff;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    struct TireStates {
        double cp_long_slip;     // Contact Path - Longitudinal Slip State (Kappa)
        double cp_side_slip;     // Contact Path - Side Slip State (Alpha)
        double vx;               // Longitudinal speed
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis
        double R_eff;            // Effective Radius
        ChVector<> disc_normal;  //(temporary for debug)
    };

    TireStates m_states;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
