// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Template for Fiala tire model
//
// =============================================================================
// =============================================================================
// STILL UNDERDEVELOPMENT - DO NOT USE
// =============================================================================
// =============================================================================

#ifndef CH_FIALATIRE_H
#define CH_FIALATIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

///
/// Fiala based tire model.
///
class CH_VEHICLE_API ChFialaTire : public ChTire {
  public:
    ChFialaTire(const std::string& name  ///< [in] name of this tire system
                );

    virtual ~ChFialaTire() {}

    /// Initialize this tire system.
    void Initialize();

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TireForce GetTireForce() const override { return m_tireforce; }

    /// Update the state of this tire system at the current time.
    /// The tire system is provided the current state of its associated wheel.
    virtual void Update(double time,                      ///< [in] current time
                        const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                        const ChTerrain& terrain          ///< [in] reference to the terrain system
                        ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    /// Set the value of the integration step size for the underlying dynamics.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Get the current value of the integration step size.
    double GetStepsize() const { return m_stepsize; }

    /// Get the unloaded radius of the tire
    double GetUnloadedRadius() const { return m_unloaded_radius; }

    /// Get the width of the tire
    double GetWidth() const { return m_width; }

    // Temporary debug methods
    double GetKappa() const { return m_states.cp_long_slip; }
    double GetAlpha() const { return m_states.cp_side_slip; }

  protected:
    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double getNormalStiffnessForce(double depth) const = 0;
    /// Return the vertical tire damping contribution to the normal force.
    virtual double getNormalDampingForce(double depth, double velocity) const = 0;

    /// Set the parameters in the Fiala model.
    virtual void SetFialaParams() = 0;

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

  private:
    double m_stepsize;

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
        double abs_vx;           // Longitudinal speed
        double vsx;              // Longitudinal slip velocity
        double vsy;              // Lateral slip velocity = Lateral velocity
        double omega;            // Wheel angular velocity about its spin axis (temporary for debug)
        ChVector<> disc_normal;  //(temporary for debug)
    };

    ContactData m_data;
    TireStates m_states;

    TireForce m_tireforce;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
