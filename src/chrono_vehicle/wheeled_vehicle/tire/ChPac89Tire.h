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
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Pacjeka 89 tire model.
class CH_VEHICLE_API ChPac89Tire : public ChTire {
  public:
    ChPac89Tire(const std::string& name  ///< [in] name of this tire system
                );

    virtual ~ChPac89Tire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Pac89Tire"; }

    /// Initialize this tire system.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< [in] associated wheel body
                            VehicleSide side                ///< [in] left/right vehicle side
                            ) override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override { return m_tireforce; }

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_tireforce; }

    /// Update the state of this tire system at the current time.
    /// The tire system is provided the current state of its associated wheel.
    virtual void Synchronize(double time,                    ///< [in] current time
                             const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChTerrain& terrain,       ///< [in] reference to the terrain system
                             CollisionType collision_type = CollisionType::SINGLE_POINT  ///< [in] collision type
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
    void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

    /// Get the width of the tire.
    double GetWidth() const { return m_width; }

    /// Get the tire deflection
    virtual double GetDeflection() const override { return m_data.depth; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_width; }

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
    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const = 0;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const = 0;

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

  private:
    struct ContactData {
        bool in_contact;      // true if disc in contact with terrain
        ChCoordsys<> frame;   // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;       // relative velocity expressed in contact frame
        double normal_force;  // magnitude of normal contact force
        double depth;         // penetration depth
    };

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

    ChFunction_Recorder m_areaDep;  // lookup table for estimation of penetration depth from intersection area

    ContactData m_data;
    TireStates m_states;

    TerrainForce m_tireforce;

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
    std::shared_ptr<ChTexture> m_texture;          ///< visualization texture asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
