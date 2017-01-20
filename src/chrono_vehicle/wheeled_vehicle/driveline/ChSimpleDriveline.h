// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Simple driveline model. This template can be used to model a 4WD driveline.
// It uses a constant front/rear torque split (a value between 0 and 1) and a
// simple model for Torsen limited-slip differentials.
//
// =============================================================================

#ifndef CH_SIMPLE_DRIVELINE_H
#define CH_SIMPLE_DRIVELINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Simple driveline model. This template can be used to model a 4WD driveline.
/// It uses a constant front/rear torque split (a value between 0 and 1) and a
/// simple model for Torsen limited-slip differentials.
class CH_VEHICLE_API ChSimpleDriveline : public ChDriveline {
  public:
    ChSimpleDriveline(const std::string& name);
    virtual ~ChSimpleDriveline() {}

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const final override { return 2; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the axles of the
    /// specified suspension subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,      ///< handle to the chassis body
                            const ChSuspensionList& suspensions,  ///< list of all vehicle suspension subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to
    /// the powertrain system.
    virtual double GetDriveshaftSpeed() const override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain
    /// system.
    virtual void Synchronize(double torque) override;

    /// Get the motor torque to be applied to the specified wheel.
    virtual double GetWheelTorque(const WheelID& wheel_id) const override;

  protected:
    /// Return the front torque fraction [0,1].
    virtual double GetFrontTorqueFraction() const = 0;

    /// Return the torque bias ratio for the front differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetFrontDifferentialMaxBias() const = 0;

    /// Return the torque bias ratio for the rear differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetRearDifferentialMaxBias() const = 0;

  private:
    std::shared_ptr<ChShaft> m_front_left;
    std::shared_ptr<ChShaft> m_front_right;
    std::shared_ptr<ChShaft> m_rear_left;
    std::shared_ptr<ChShaft> m_rear_right;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
