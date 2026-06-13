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
// Utility classes implementing PID speed controllers.
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the throttle/braking outputs.
//
// =============================================================================

#ifndef CH_SPEED_CONTROLLER_H
#define CH_SPEED_CONTROLLER_H

#include <string>

#include "chrono/core/ChFrameMoving.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

/// Base class for a speed controller.
/// Data collection from a speed controller can be started (restarted) and suspended (stopped) as many times as desired. Data collected so far can be written to a file.  The
/// tab-separated output ASCII file contains on each line the time, current desired speed, and current actual speed.
class CH_VEHICLE_API ChSpeedController {
  public:
    /// Destructor
    virtual ~ChSpeedController();

    /// Return the current speed.
    double GetCurrentSpeed() const { return m_speed; }

    /// Reset the speed controller.
    void Reset(const ChFrameMoving<>& ref_frame);

    /// Advance the state of the speed controller.
    double Advance(const ChFrameMoving<>& ref_frame, double target_speed, double time, double step);

    /// Start/restart data collection.
    void StartDataCollection();

    /// Suspend/stop data collection.
    void StopDataCollection();

    /// Return true if data is being collected.
    bool IsDataCollectionEnabled() const { return m_collect; }

    /// Return true if data is available for output.
    bool IsDataAvailable() const { return m_csv != NULL; }

    /// Output data collected so far to the specified file.
    void WriteOutputFile(const std::string& filename);

  protected:
    ChSpeedController();

    /// Reset the PID controller.
    virtual void OnReset(const ChFrameMoving<>& ref_frame) = 0;

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral
    /// component of the PID controller and returns the calculated controller value.
    virtual double OnAdvance(const ChFrameMoving<>& ref_frame, double target_speed, double time, double step) = 0;

    double m_speed;  ///< current vehicle speed
    double m_err;    ///< current error (signed distance to target point)
    double m_errd;   ///< error derivative
    double m_erri;   ///< integral of error

    ChWriterCSV* m_csv;  ///< ChWriterCSV object for data collection
    bool m_collect;      ///< flag indicating whether or not data is being collected
};

// --------------------------------------------------------------------------------------------------------------------

/// Speed controller using a PID controller.
class CH_VEHICLE_API ChSpeedControllerPID : public ChSpeedController {
  public:
    /// Construct a speed controller with default parameters.
    /// Default values are all gains set to zero (no controller).
    /// The user is responsible for calling SetGains.
    ChSpeedControllerPID();

    /// Construct a steering controller with parameters read from a JSON file.
    ChSpeedControllerPID(const std::string& filename);

    /// Destructor.
    ~ChSpeedControllerPID();

    /// Set the gains for the PID controller.
    void SetGains(double Kp, double Ki, double Kd) {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
    }

  private:
    /// Reset the PID controller.
    virtual void OnReset(const ChFrameMoving<>& ref_frame) override;

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral
    /// component of the PID controller and returns the calculated controller value.
    virtual double OnAdvance(const ChFrameMoving<>& ref_frame, double target_speed, double time, double step) override;

    double m_Kp;  ///<
    double m_Ki;  ///< PID controller gains
    double m_Kd;  ///<
};

/// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
