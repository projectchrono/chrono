// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a suspension test rig driver. A driver system must be able to
// report the current values of the inputs (steering, left post, right post).
//
// =============================================================================

#ifndef CH_STR_DRIVER_H
#define CH_STR_DRIVER_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Base class for a suspension test rig driver system.
/// A driver system must be able to report the current values of the inputs (left posts, right posts, steering).
/// A concrete driver class must set the member variables m_displacementLeft, m_displacementRight, m_steering,
/// and - if possible - the rates of change for the displacements (otherwise left at their default value of 0).
class CH_VEHICLE_API ChSuspensionTestRigDriver {
  public:
    virtual ~ChSuspensionTestRigDriver() {}

    /// Get the driver steering input (in the range [-1,+1]).
    double GetSteering() const { return m_steering; }

    /// Get the left post vertical displacement inputs (each in the range [-1,+1]).
    const std::vector<double>& GetDisplacementLeft() const { return m_displLeft; }

    /// Get the right post vertical displacement inputs (each in the range [-1,+1]).
    const std::vector<double>& GetDisplacementRight() const { return m_displRight; }

    /// Get the left post displacement rates of change.
    const std::vector<double>& GetDisplacementSpeedLeft() const { return m_displSpeedLeft; }

    /// Get the right post displacement rates of change.
    const std::vector<double>& GetDisplacementSpeedRight() const { return m_displSpeedRight; }

    /// Return false while driver inputs are ignored (while the rig is reaching the ride height configuration) and true
    /// otherwise. In general, outputs from the test rig should only be collected while Started returns true.
    bool Started() const;

    /// Return true when driver stopped producing inputs.
    virtual bool Ended() const { return false; }

    /// Initialize output file for recording driver inputs.
    bool LogInit(const std::string& filename);

    /// Record the current driver inputs to the log file.
    bool Log(double time);

  protected:
    ChSuspensionTestRigDriver();

    /// Initialize this driver system.
    virtual void Initialize(int naxles);

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time);

    /// Get string message.
    virtual std::string GetInfoMessage() const { return ""; }

    /// Set the value for the driver left post displacement input.
    void SetDisplacementLeft(int axle, double val, double min_val = -1, double max_val = 1);

    /// Set the value for the driver right post displacement input.
    void SetDisplacementRight(int axle, double val, double min_val = -1, double max_val = 1);

    /// Set the value for the driver steering input.
    void SetSteering(double val, double min_val = -1, double max_val = 1);

    int m_naxles;                           ///< number of actuated axles
    std::vector<double> m_displLeft;        ///< current value of left post displacements
    std::vector<double> m_displRight;       ///< current value of right post displacements
    std::vector<double> m_displSpeedLeft;   ///< current value of left post displacement rates of change
    std::vector<double> m_displSpeedRight;  ///< current value of right post displacement rates of change
    double m_steering;                      ///< current value of steering input

    double m_delay;  ///< time delay before generating inputs

  private:
    double m_time;               ///< time of last synchronization
    std::string m_log_filename;  ///< name of output file for recording driver inputs

    friend class ChSuspensionTestRig;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
