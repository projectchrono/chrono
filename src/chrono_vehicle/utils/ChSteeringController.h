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
// Utility classes implementing PID steering controllers. The base class
// implements the basic functionality to control the error between the location
// of a sentinel point (a point at a look-ahead distance in front of the vehicle)
// and the current target point.
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
#include <vector>

#include "chrono/core/ChBezierCurve.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"

namespace chrono {

// -----------------------------------------------------------------------------
/// Base class for all steering path-following PID controllers.
// -----------------------------------------------------------------------------
class CH_VEHICLE_API ChSteeringController {
  public:
    ChSteeringController();
    virtual ~ChSteeringController() {}

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
    /// for the sentinel point can be calculated.  the default implementation
    /// in the base class simply calculates the new sentinel point location.
    virtual void Reset(const ChVehicle& vehicle);

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral
    /// component of the PID controller and returns the calculated steering value.
    double Advance(const ChVehicle& vehicle, double step);

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
};

// -----------------------------------------------------------------------------
/// Concrete path-following steering PID controller.
/// The path to be followed is specified as a ChBezierCurve object and the
/// target point is defined to be the point on that path that is closest to the
/// current location of the sentinel point.
// -----------------------------------------------------------------------------
class CH_VEHICLE_API ChPathSteeringController : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    ChPathSteeringController(ChBezierCurve* path);

    /// Destructor for ChPathSteeringController.
    ~ChPathSteeringController();

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location
    /// of the sentinel point.
    virtual void Reset(const ChVehicle& vehicle) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to
    /// the current location of the sentinel point.
    virtual void CalcTargetLocation() override;

  private:
    ChBezierCurve* m_path;            ///< tracked path (piecewise cubic Bezier curve)
    ChBezierCurveTracker* m_tracker;  ///< path tracker
};

}  // end namespace chrono

#endif