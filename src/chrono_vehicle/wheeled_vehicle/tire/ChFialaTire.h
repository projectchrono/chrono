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
// Template for Fiala tire model
//
// =============================================================================

#ifndef CH_FIALATIRE_H
#define CH_FIALATIRE_H

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

/// Fiala based tire model.
class CH_VEHICLE_API ChFialaTire : public ChTire {
  public:
    ChFialaTire(const std::string& name  ///< [in] name of this tire system
                );

    virtual ~ChFialaTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "FialaTire"; }

    /// Initialize this tire system.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< [in] associated wheel body
                            VehicleSide side                ///< [in] left/right vehicle side
                            ) override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire width.
    /// For a Fiala tire, this is the unloaded tire radius.
    virtual double GetRadius() const override { return m_unloaded_radius; }

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

    /// Get the width of the tire.
    double GetWidth() const { return m_width; }

    /// Get visualization width.
    virtual double GetVisualizationWidth() const { return m_width; }

    /// Get the tire slip angle computed internally by the Fiala model (in radians).
    /// The reported value will be similar to that reported by ChTire::GetSlipAngle.
    double GetSlipAngle_internal() const { return m_states.alpha; }

    /// Get the tire longitudinal slip computed internally by the Fiala model.
    /// The reported value will be different from that reported by ChTire::GetLongitudinalSlip
    /// because ChFialaTire uses the loaded tire radius for its calculation.
    double GetLongitudinalSlip_internal() const { return m_states.kappa; }

    /// Get the camber angle for the Fiala tire model (in radians).
    /// ChFialaTire does not calculate its own camber angle. This value is the same as that
    /// reported by ChTire::GetCamberAngle.
    double GetCamberAngle_internal() { return GetCamberAngle(); }

    /// Generate basic tire plots.
    /// This function creates a Gnuplot script file with the specified name.
    void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

  protected:
    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const = 0;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const = 0;

    /// Set the parameters in the Fiala model.
    virtual void SetFialaParams() = 0;

    /// Calculate Patch Forces
    void FialaPatchForces(double& fx, double& fy, double& mz, double kappa, double alpha, double fz);

    /// Fiala tire model parameters

    double m_unloaded_radius;
    double m_width;
    double m_rolling_resistance;
    double m_c_slip;
    double m_c_alpha;
    double m_u_min;
    double m_u_max;
    double m_relax_length_x;
    double m_relax_length_y;

    // Fiala extensions from ADAMS/Car user source example and TMeasy
    double m_mu;    ///< Actual friction coefficient of the road
    double m_mu_0;  ///< Local friction coefficient of the road for given parameters

    /// Switch for dynamic mode (relaxation)
    bool m_dynamic_mode;
    double m_time;        // actual system time
    double m_time_trans;  // end of start transient

  private:
    struct ContactData {
        bool in_contact;      // true if disc in contact with terrain
        ChCoordsys<> frame;   // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;       // relative velocity expressed in contact frame
        double normal_force;  // magnitude of normal contact force
        double depth;         // penetration depth
    };

    struct TireStates {
        double kappa;   // Contact Path - Stationary Longitudinal Slip State (Kappa)
        double alpha;   // Contact Path - Stationary Side Slip State (Alpha)
        double abs_vx;  // Longitudinal speed
        double abs_vt;  // Longitudinal transport speed
        double vsx;     // Longitudinal slip velocity
        double vsy;     // Lateral slip velocity = Lateral velocity
        double omega;   // Wheel angular velocity about its spin axis (temporary for debug)
        double Fx_l;
        double Fy_l;
        ChVector<> disc_normal;  // temporary for debug
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
