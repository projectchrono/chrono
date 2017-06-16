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
// Utility classes implementing PID steering controllers. The base class
// implements the basic functionality to control the error between the location
// of a sentinel point (a point at a look-ahead distance in front of the vehicle)
// and the current target point.
//
// Derived classes differ in how they specify the target point.  This can be the
// closest point to the sentinel point on a pre-defined curve path (currently
// using a ChBezierCurve) or from some other external sources (e.g. interfacing
// with a camera sensor).
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the steering output.
//
// =============================================================================

#ifndef CH_STEERING_CONTROLLER_H
#define CH_STEERING_CONTROLLER_H

#include <string>

#include "chrono/core/ChBezierCurve.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

/// Base class for all steering path-following PID controllers.
/// The base class implements the basic functionality to control the error
/// between the location of a sentinel point (a point at a look-ahead distance
/// in front of the vehicle) and the current target point.
///
/// The parameters of a steering controller can also be specified through a JSON
/// file; a sample JSON input file is as follows:
///
///      {
///          "Gains":
///          {
///              "Kp":   0.5,
///                  "Ki" : 0.0,
///                  "Kd" : 0.0
///          },
///
///          "Lookahead Distance": 20.0
///      }
///
/// Data collection from the steering controller can be started (restarted) and
/// suspended (stopped) as many times as desired.  Data collected so far can be
/// written to a file.  The tab-separated output ASCII file contains on each line
/// the time, location of the target point, and location of the sentinel point.
class CH_VEHICLE_API ChSteeringController {
  public:
    /// Construct a steering controller with default parameters.
    /// Default values are all gains set to zero (no controller).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChSteeringController();

    /// Construct a steering controller with parameters read from a JSON file.
    ChSteeringController(const std::string& filename);

    /// Destructor.
    virtual ~ChSteeringController();

    /// Specify the look-ahead distance.
    /// This defines the location of the "sentinel" point (in front
    /// of the vehicle, at the given distance from the chassis reference
    /// frame).
    void SetLookAheadDistance(double dist) { m_dist = dist; }

    /// Set the gains for the PID controller.
    void SetGains(double Kp, double Ki, double Kd) {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
    }

    /// Return the current location of the sentinel point.
    /// The return vector is expressed in the global reference frame.
    const ChVector<>& GetSentinelLocation() const { return m_sentinel; }

    /// Return the current value of the target point.
    /// The return vector is expressed in the global reference frame.
    const ChVector<>& GetTargetLocation() const { return m_target; }

    /// Reset the PID controller.
    /// This function must be called at a configuration where a valid location
    /// for the sentinel point can be calculated.  The default implementation
    /// in the base class simply calculates the new sentinel point location and
    /// sets all errors to 0.
    virtual void Reset(const ChVehicle& vehicle);

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral
    /// component of the PID controller and returns the calculated steering value.
    double Advance(const ChVehicle& vehicle, double step);

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
    /// Calculate the current target point location.
    /// All derived classes must implement this function to calculate the current
    /// location of the target point, expressed in the global frame. The location
    /// of the sentinel point at the current time is calculated and available in
    /// m_sentinel.
    virtual void CalcTargetLocation() = 0;

    double m_dist;          ///< look-ahead distance
    ChVector<> m_sentinel;  ///< position of sentinel point in global frame
    ChVector<> m_target;    ///< position of target point in global frame

    double m_Kp;  ///<
    double m_Ki;  ///< PID controller gains
    double m_Kd;  ///<

    double m_err;   ///< current error (signed distance to target point)
    double m_errd;  ///< error derivative
    double m_erri;  ///< integral of error

    utils::CSV_writer* m_csv;  ///< CSV_writer object for data collection
    bool m_collect;            ///< flag indicating whether or not data is being collected
};

/// Concrete path-following steering PID controller.
/// The path to be followed is specified as a ChBezierCurve object and the
/// target point is defined to be the point on that path that is closest to the
/// current location of the sentinel point.
class CH_VEHICLE_API ChPathSteeringController : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringController(std::shared_ptr<ChBezierCurve> path, bool isClosedPath = false);

    /// Construct a steering controller to track the specified path.
    /// This version reads controller gains and lookahead distance from the
    /// specified JSON file.
    ChPathSteeringController(const std::string& filename,
                             std::shared_ptr<ChBezierCurve> path,
                             bool isClosedPath = false);

    /// Destructor for ChPathSteeringController.
    ~ChPathSteeringController() {}

    /// Return a pointer to the Bezier curve
    std::shared_ptr<ChBezierCurve> GetPath() const { return m_path; }

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location
    /// of the sentinel point.
    virtual void Reset(const ChVehicle& vehicle) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to
    /// the current location of the sentinel point.
    virtual void CalcTargetLocation() override;

  private:
    std::shared_ptr<ChBezierCurve> m_path;            ///< tracked path (piecewise cubic Bezier curve)
    std::unique_ptr<ChBezierCurveTracker> m_tracker;  ///< path tracker
};

/// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
