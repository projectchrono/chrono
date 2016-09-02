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

#include "chrono_vehicle/ChDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
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
    chrono::ChSystemDEM* m_system;  ///< containing system
    double m_delay;                 ///< delay in generating driver inputs

    bool m_chassis_fixed;  ///< flag indicating whther or not chassis is fixed to ground

    chrono::vehicle::hmmwv::HMMWV_Vehicle* m_vehicle;        ///< vehicle system
    chrono::vehicle::hmmwv::HMMWV_Powertrain* m_powertrain;  ///< powertrain system
    chrono::vehicle::ChDriver* m_driver;                     ///< driver system

    int m_num_wheels;                           ///< number of vehicle wheels
    chrono::vehicle::TireForces m_tire_forces;  ///< forces received from tire nodes

    // Private methods

    // Write node state information
    void WriteStateInformation(chrono::utils::CSV_writer& csv);
};

#endif