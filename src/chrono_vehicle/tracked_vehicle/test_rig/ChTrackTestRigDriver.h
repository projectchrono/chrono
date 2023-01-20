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
// Base class for a track test rig driver. A driver system must be able to
// report the current values of the inputs (throttle and post displacements).
//
// =============================================================================

#ifndef CH_TTR_DRIVER_H
#define CH_TTR_DRIVER_H

#include <string>
#include <vector>
//
#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Base class for a track test rig driver system.
/// A driver system must be able to report the current values of the inputs (throttle and post displacements).
class CH_VEHICLE_API ChTrackTestRigDriver {
  public:
    ChTrackTestRigDriver();

    virtual ~ChTrackTestRigDriver() {}

    /// Get the specified post vertical displacement (in the range [-1,+1])
    double GetDisplacement(int index) const { return m_displ[index]; }

    /// Get the specified post vertical displacement rate of change.
    double GetDisplacementSpeed(int index) const { return m_displSpeed[index]; }

    /// Get the driver throttle input (in the range [-1,+1])
    double GetThrottle() const { return m_throttle; }

    /// Return false while driver inputs are ignored (while the rig is reaching the ride height configuration) and true
    /// otherwise. In general, outputs from the test rig should only be collected while Started returns true.
    bool Started() const;

    /// Return true when driver stopped producing inputs.
    virtual bool Ended() const { return false; }

  protected:
    /// Overwrite the value for the specified post displacement input.
    void SetDisplacement(int index, double val, double min_val = -1, double max_val = 1);

    /// Overwrite the value for the throttle input.
    void SetThrottle(double val, double min_val = -1, double max_val = 1);

    /// Initialize this driver system.
    virtual void Initialize(size_t num_posts, const std::vector<double>& locations);

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time);

    /// Initialize output file for recording driver inputs.
    bool LogInit(const std::string& filename);

    /// Record the current driver inputs to the log file.
    bool Log(double time);

    /// Set value of the time delay.
    /// During this initial time period, no driver inputs are generated.
    void SetTimeDelay(double delay) { m_delay = delay; }

    /// Get string message.
    virtual std::string GetInfoMessage() const { return ""; }

    std::vector<double> m_displ;       ///< current values of post displacements
    std::vector<double> m_displSpeed;  ///< current value of post displacement rates of change
    double m_throttle;                 ///< current value of throttle input
    double m_delay;                    ///< time delay before generating inputs

    std::vector<double> m_locations;  ///< post locations in X direction

  private:
    double m_time;               ///< time of last synchronization
    std::string m_log_filename;  ///< name of output file for recording driver inputs

    friend class ChTrackTestRig;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif