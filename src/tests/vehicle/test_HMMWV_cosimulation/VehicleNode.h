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
// Authors: Radu Serban
// =============================================================================
//
// Definition of the VEHICLE NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef HMMWV_COSIM_VEHICLENODE_H
#define HMMWV_COSIM_VEHICLENODE_H

#include <vector>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Powertrain.h"

#include "BaseNode.h"

// =============================================================================

class VehicleNode : public BaseNode {
  public:
    VehicleNode();
    ~VehicleNode();

    /// Set delay in generating driver inputs.
    void SetDriverDelay(double delay) { m_delay = delay; }

    /// Specify whether or not chassis is fixed to ground (default: false).
    void SetChassisFixed(bool fixed) { m_chassis_fixed = fixed; }

    /// Specify initial chassis forward velocity (default: 0).
    void SetInitFwdVel(double fwd_vel) { m_init_fwd_vel = fwd_vel; }

    /// Specify initial wheel angular velocity (default: 0).
    void SetInitWheelAngVel(double ang_vel) { m_init_ang_vel = ang_vel; }

    /// Set driver inputs from data.
    /// The data points must be provided with increasing time values.
    void SetDataDriver(const std::vector<chrono::vehicle::ChDataDriver::Entry>& data);

    /// Set path-follower driver.
    void SetPathDriver(double run, double radius, double offset, int nturns, double target_speed);

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override;

  private:
    enum DriverType {
        DEFAULT_DRIVER,  ///< default driver inputs (all zero)
        DATA_DRIVER,     ///< driver inputs from data (linear interpolation)
        PATH_DRIVER      ///< PID path-follower driver
    };

    chrono::ChSystemDEM* m_system;  ///< containing system
    double m_delay;                 ///< delay in generating driver inputs

    bool m_chassis_fixed;   ///< flag indicating whther or not chassis is fixed to ground
    double m_init_fwd_vel;  ///< initial chassis forward velocity
    double m_init_ang_vel;  ///< initial wheel angular velocity

    DriverType m_driver_type;             ///< driver type
    chrono::vehicle::ChDriver* m_driver;  ///< driver system

    std::vector<chrono::vehicle::ChDataDriver::Entry> m_driver_data;  ///< data points for DATA driver type

    chrono::ChBezierCurve* m_driver_path;  ///< path (Bezier curve) for PATH_FOLLOWER driver type
    double m_driver_target_speed;          ///< target speed for PATH_FOLLOWER driver type

    chrono::vehicle::hmmwv::HMMWV_VehicleFull* m_vehicle;    ///< vehicle system
    chrono::vehicle::hmmwv::HMMWV_Powertrain* m_powertrain;  ///< powertrain system

    int m_num_wheels;                           ///< number of vehicle wheels
    chrono::vehicle::TireForces m_tire_forces;  ///< forces received from tire nodes

    // Private methods

    // Write node state information
    void WriteStateInformation(chrono::utils::CSV_writer& csv);
};

#endif