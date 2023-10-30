// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Mechanism for a single-wheel testing rig co-simulated with a tire and a
// terrain system.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_RIG_NODE_H
#define CH_VEHCOSIM_RIG_NODE_H

#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Mechanism for a single-wheel testing rig.
/// The mechanism system is co-simulated with a tire and a terrain system.
class CH_VEHICLE_API ChVehicleCosimRigNode : public ChVehicleCosimWheeledMBSNode {
  public:
    ChVehicleCosimRigNode();
    ~ChVehicleCosimRigNode();

    /// Set total rig system mass (default: 100).
    /// This represents the equivalent load on the soil from all rig bodies. 
    /// Note that the total mass must be at least 2 kg; otherwise, it will be overwritten.
    void SetTotalMass(double mass) { m_total_mass = mass; }

    /// Set (constant) toe angle in radians (default: 0).
    void SetToeAngle(double angle) { m_toe_angle = angle; }

    /// Set the initial rig position, relative to the center of the terrain top-surface.
    /// This is the initial location of the spindle.
    void SetInitialLocation(const ChVector<>& init_loc) { m_init_loc = init_loc; }

  private:
    /// Initialize the vehicle MBS and any associated subsystems.
    virtual void InitializeMBS(const ChVector2<>& terrain_size,  ///< terrain length x width
                               double terrain_height             ///< initial terrain height
                               ) override;

    /// Apply tire info (mass, radius, width).
    virtual void ApplyTireInfo(const std::vector<ChVector<>>& tire_info) override;

    // Output rig data.
    virtual void OnOutputData(int frame) override;

    /// Process the provided spindle force (received from the corresponding tire node).
    virtual void ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) override;

    /// Return the number of spindles in the vehicle system.
    virtual int GetNumSpindles() const override { return 1; }

    /// Return the i-th spindle body in the vehicle system.
    virtual std::shared_ptr<ChBody> GetSpindleBody(unsigned int i) const override { return m_spindle; }

    /// Return the vertical mass load on the i-th spindle.
    /// This does not include the mass of the tire.
    virtual double GetSpindleLoad(unsigned int i) const override { return m_total_mass; }

    /// Get the body state of the spindle body to which the i-th wheel/tire is attached.
    virtual BodyState GetSpindleState(unsigned int i) const override;

    /// Get the "chassis" body.
    virtual std::shared_ptr<ChBody> GetChassisBody() const override { return m_chassis; }

    /// Impose spindle angular speed as dictated by an attached DBP rig.
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) override;

  private:
    std::shared_ptr<ChBody> m_chassis;  ///< chassis body
    std::shared_ptr<ChBody> m_spindle;  ///< spindle body

    ChVector<> m_init_loc;  ///< initial rig location (relative to center of terrain top surface)
    double m_total_mass;    ///< total equivalent wheel mass
    double m_toe_angle;     ///< toe angle (controls tire slip angle)

    std::shared_ptr<ChLinkMotorRotationSpeed> m_rev_motor;  ///< motor to enforce spindle angular vel

    std::shared_ptr<ChLoadBodyForce> m_spindle_terrain_force;    ///< terrain force loads on the spindle
    std::shared_ptr<ChLoadBodyTorque> m_spindle_terrain_torque;  ///< terrain torque loads on the spindle
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
