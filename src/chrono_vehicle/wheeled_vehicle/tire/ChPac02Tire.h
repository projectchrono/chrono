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
// Template for a PAC02 tire model
//
// =============================================================================

#ifndef CH_PAC02TIRE_H
#define CH_PAC02TIRE_H

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
class CH_VEHICLE_API ChPac02Tire : public ChTire {
  public:
    ChPac02Tire(const std::string& name);

    virtual ~ChPac02Tire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Pac02Tire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_states.R_eff; }

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_tireforce; }

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
    virtual void SetPac02Params() = 0;

    double m_kappa;  ///< longitudinal slip ratio
    double m_alpha;  ///< slip angle
    double m_gamma;  ///< camber angle

    double m_gamma_limit;  ///< limit camber angle

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

    // combined forces calculation
    double m_kappa_c;
    double m_alpha_c;
    double m_mu_x_act;
    double m_mu_x_max;
    double m_mu_y_act;
    double m_mu_y_max;

    struct Pac02ScalingFactors {
        double xsi1;
        double lfz0;
        double lcx1;
        double lex;
        double lkx;
        double lhx;
        double lmux;
        double lvx;

        double xsi2;
        double xsi3;
        double xsi4;
        double lcy;
        double ley;
        double lhy;
        double lky;
        double lmuy;
        double lvy;
    };

    struct Pac02Coeff {
        double FzNomin;
        // Longitudinal Coefficients
        double pcx1;
        double pdx1;
        double pdx2;
        double pex1;
        double pex2;
        double pex3;
        double phx1;
        double phx2;
        double pkx1;
        double pkx2;
        double pvx1;
        double pvx2;
        // Lateral Coefficients
        double pcy1;
        double pdy1;
        double pdy2;
        double pey1;
        double pey2;
        double phy1;
        double phy2;
        double pky1;
        double pky2;
        double pvy1;
        double pvy2;
    };

    Pac02ScalingFactors m_PacScal;
    Pac02Coeff m_PacCoeff;

  private:
    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override { return m_tireforce; }

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

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

    double CalcFx1(double kappa, double Fz);
    double CalcFy1(double alpha, double Fz);
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
