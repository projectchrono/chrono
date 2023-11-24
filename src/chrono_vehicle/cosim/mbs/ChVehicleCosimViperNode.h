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
// Viper rover co-simulated with "tire" nodes and a terrain node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_VIPER_NODE_H
#define CH_VEHCOSIM_VIPER_NODE_H

#include "chrono/assets/ChVisualSystem.h"
#include "chrono_models/robot/viper/Viper.h"
#include "chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Viper rover co-simulation node.
/// The rover is co-simulated with tire nodes and a terrain node.
class CH_VEHICLE_API ChVehicleCosimViperNode : public ChVehicleCosimWheeledMBSNode {
  public:
    /// Construct a Viper rover node.
    ChVehicleCosimViperNode();

    ~ChVehicleCosimViperNode();

    /// Get the underlying rover system.
    std::shared_ptr<viper::Viper> GetRover() const { return m_viper; }

    /// Set the initial rover position, relative to the center of the terrain top-surface.
    void SetInitialLocation(const ChVector<>& init_loc) { m_init_loc = init_loc; }

    /// Set the initial rover yaw angle (in radians).
    void SetInitialYaw(double init_yaw) { m_init_yaw = init_yaw; }

    /// Attach a Viper driver system.
    void SetDriver(std::shared_ptr<viper::ViperDriver> driver) { m_driver = driver; }

  private:
    /// Initialize the rover MBS and any associated subsystems.
    virtual void InitializeMBS(const ChVector2<>& terrain_size,  ///< terrain length x width
                               double terrain_height             ///< initial terrain height
                               ) override;

    /// Apply tire info (mass, radius, width).
    virtual void ApplyTireInfo(const std::vector<ChVector<>>& tire_info) override;

    // Output rover data.
    virtual void OnOutputData(int frame) override;

    /// Perform Viper update before advancing the dynamics.
    virtual void PreAdvance(double step_size) override;

    /// Process the provided spindle force (received from the corresponding tire node).
    virtual void ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) override;

    /// Return the number of spindles in the rover system.
    virtual int GetNumSpindles() const override { return 4; }

    /// Return the i-th spindle body in the rover system.
    virtual std::shared_ptr<ChBody> GetSpindleBody(unsigned int i) const override;

    /// Return the vertical mass load on the i-th spindle.
    /// This does not include the mass of the wheel.
    virtual double GetSpindleLoad(unsigned int i) const override;

    /// Get the body state of the spindle body to which the i-th wheel/tire is attached.
    virtual BodyState GetSpindleState(unsigned int i) const override;

    /// Get the "chassis" body.
    virtual std::shared_ptr<ChBody> GetChassisBody() const override;

    /// Impose spindle angular speed as dictated by an attached DBP rig.
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) override;

    void WriteBodyInformation(utils::CSV_writer& csv);

    virtual void OnRender() override;

  private:
    std::shared_ptr<viper::Viper> m_viper;         ///< Viper rover;
    std::shared_ptr<viper::ViperDriver> m_driver;  ///< Viper driver
    std::shared_ptr<ChVisualSystem> m_vsys;        ///< run-time visualization system

    ChVector<> m_init_loc;  ///< initial rover location (relative to center of terrain top surface)
    double m_init_yaw;      ///< initial rover yaw

    std::vector<double> m_spindle_vertical_loads;                              ///< vertical loads on each spindle
    std::vector<std::shared_ptr<ChLoadBodyForce>> m_spindle_terrain_forces;    ///< terrain force loads on each spindle
    std::vector<std::shared_ptr<ChLoadBodyTorque>> m_spindle_terrain_torques;  ///< terrain torque loads on each spindle
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
