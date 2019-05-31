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

#ifndef CH_DRIVER_TTR_H
#define CH_DRIVER_TTR_H

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
class CH_VEHICLE_API ChDriverTTR {
  public:
    ChDriverTTR();

    virtual ~ChDriverTTR() {}

    /// Get the specified post vertical displacement (in the range [-1,+1])
    double GetDisplacement(int index) const { return m_displ[index]; }

    /// Get the driver throttle input (in the range [-1,+1])
    double GetThrottle() const { return m_throttle; }

    /// Initialize this driver system.
    virtual void Initialize(size_t num_posts);

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time) {}

    /// Initialize output file for recording driver inputs.
    bool LogInit(const std::string& filename);

    /// Record the current driver inputs to the log file.
    bool Log(double time);

    /// Set value of the time delay.
    /// During this initial time period, no driver inputs are generated.
    void SetTimeDelay(double delay) { m_delay = delay; }

    /// Overwrite the value for the specified post displacement input.
    void SetDisplacement(int index, double val, double min_val = -1, double max_val = 1);

    /// Overwrite the value for the throttle input.
    void SetThrottle(double val, double min_val = -1, double max_val = 1);

    /// Get string message.
    virtual std::string GetInfoMessage() const { return ""; }

  protected:
    std::vector<double> m_displ;  ///< current values of post displacements
    double m_throttle;            ///< current value of throttle input

    double m_delay;  ///< time delay before generating inputs

  private:
    std::string m_log_filename;  ///< name of output file for recording driver inputs
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif