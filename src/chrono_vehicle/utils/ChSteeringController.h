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
#include "chrono/core/ChFrameMoving.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

// --------------------------------------------------------------------------------------------------------------------

/// Base class for all steering path-following PID controllers.
/// The base class implements the basic functionality to control the error between the location of a sentinel point (a
/// point at a look-ahead distance in front of the vehicle) and the current target point.
///
/// Data collection from the steering controller can be started (restarted) and suspended (stopped) as many times as
/// desired.  Data collected so far can be written to a file.  The tab-separated output ASCII file contains on each line
/// the time, location of the target point, and location of the sentinel point.
class CH_VEHICLE_API ChSteeringController {
  public:
    /// Destructor.
    virtual ~ChSteeringController();

    /// Specify the look-ahead distance.
    /// This defines the location of the "sentinel" point (in front of the vehicle, at the given distance from the
    /// chassis reference frame).
    void SetLookAheadDistance(double dist) { m_dist = dist; }

    /// Return the current location of the sentinel point.
    /// The return vector is expressed in the global reference frame.
    const ChVector<>& GetSentinelLocation() const { return m_sentinel; }

    /// Return the current value of the target point.
    /// The return vector is expressed in the global reference frame.
    const ChVector<>& GetTargetLocation() const { return m_target; }

    /// Return a pointer to the Bezier curve
    std::shared_ptr<ChBezierCurve> GetPath() const { return m_path; }

    /// Reset the PID controller.
    /// This function must be called at a configuration where a valid location for the sentinel point can be calculated.
    /// The default implementation in the base class simply calculates the new sentinel point location and sets all
    /// errors to 0.
    virtual void Reset(const ChFrameMoving<>& ref_frame);

    /// Advance the state of the PID controller.
    virtual double Advance(const ChFrameMoving<>& ref_frame, double time, double step) = 0;

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
    /// Construct a steering controller with default parameters.
    ChSteeringController(std::shared_ptr<ChBezierCurve> path);

    /// Calculate the current target point location.
    /// All derived classes must implement this function to calculate the current location of the target point,
    /// expressed in the global frame. The location of the sentinel point at the current time is calculated and
    /// available in m_sentinel.
    virtual void CalcTargetLocation() = 0;

    std::shared_ptr<ChBezierCurve> m_path;  ///< tracked path (piecewise cubic Bezier curve)

    double m_dist;          ///< look-ahead distance
    ChVector<> m_sentinel;  ///< position of sentinel point in global frame
    ChVector<> m_target;    ///< position of target point in global frame

    double m_err;   ///< current error (signed distance to target point)
    double m_errd;  ///< error derivative
    double m_erri;  ///< integral of error

    utils::CSV_writer* m_csv;  ///< CSV_writer object for data collection
    bool m_collect;            ///< flag indicating whether or not data is being collected
};

// --------------------------------------------------------------------------------------------------------------------

/// Path-following steering PID controller.
/// The path to be followed is specified as a ChBezierCurve object and the target point is defined to be the point on
/// that path that is closest to the current location of the sentinel point.
///
/// The parameters of a steering controller can also be specified through a JSON file; sample JSON input file:
/// <pre>
///      {
///          "Gains":
///          {
///              "Kp":   0.5,
///              "Ki" : 0.0,
///              "Kd" : 0.0
///          },
///
///          "Lookahead Distance": 20.0
///      }
/// </pre>
class CH_VEHICLE_API ChPathSteeringController : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringController(std::shared_ptr<ChBezierCurve> path);

    /// Construct a steering controller to track the specified path. This version reads controller gains and lookahead
    /// distance from the specified JSON file.
    ChPathSteeringController(const std::string& filename, std::shared_ptr<ChBezierCurve> path);

    ~ChPathSteeringController() {}

    /// Set the gains for the PID controller.
    void SetGains(double Kp, double Ki, double Kd);

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location
    /// of the sentinel point.
    virtual void Reset(const ChFrameMoving<>& ref_frame) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to
    /// the current location of the sentinel point.
    virtual void CalcTargetLocation() override;

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral component of the PID controller and returns the
    /// calculated steering value.
    virtual double Advance(const ChFrameMoving<>& ref_frame, double time, double step) override;

  private:
    std::unique_ptr<ChBezierCurveTracker> m_tracker;  ///< path tracker

    double m_Kp;  ///< Proportional gain
    double m_Ki;  ///< Integral gain
    double m_Kd;  ///< Differential gain
};

// --------------------------------------------------------------------------------------------------------------------

/// Path-following steering 3(2) channel PDT1/PT1 controller.
/// The path to be followed is specified as a ChBezierCurve object and the target point is defined to be the point on
/// that path that is closest to the current location of the sentinel point. The sentinel point should never leave the
/// end or beginning of the path.
/// The controller is sensitive to tire relaxiation, when steering oscillations occure and do not calm down after a
/// short driving distance, Kp should be reduced carefully.
class CH_VEHICLE_API ChPathSteeringControllerXT : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringControllerXT(std::shared_ptr<ChBezierCurve> path, double max_wheel_turn_angle = 0.0);

    /// Construct a steering controller to track the specified path.
    /// This version reads controller gains and lookahead distance from the specified JSON file.
    ChPathSteeringControllerXT(const std::string& filename,
                               std::shared_ptr<ChBezierCurve> path,
                               double max_wheel_turn_angle = 0.0);

    ~ChPathSteeringControllerXT() {}

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location of the sentinel point.
    virtual void Reset(const ChFrameMoving<>& ref_frame) override;

    /// Set the gains (weighting factors) for the eXTended controller.
    void SetGains(double Kp = 0.4, double W_y_err = 1.0, double W_heading_err = 1.0, double W_ackermann = 1.0);

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to the current location of the sentinel
    /// point.
    virtual void CalcTargetLocation() override;

    /// Advance the state of the XT controller.
    virtual double Advance(const ChFrameMoving<>& ref_frame, double time, double step) override;

  private:
    double CalcHeadingError(ChVector<>& a, ChVector<>& b);
    int CalcCurvatureCode(ChVector<>& a, ChVector<>& b);
    double CalcAckermannAngle();

    std::unique_ptr<ChBezierCurveTracker> m_tracker;  ///< path tracker

    double m_R_threshold;           ///< allowed minimal curve radius to be treated as straight line
    double m_max_wheel_turn_angle;  ///< max. possible turn angle of the front wheel (bicycle model)

    ChVector<> m_pnormal;    ///< local path normal
    ChVector<> m_ptangent;   ///< local path tangent
    ChVector<> m_pbinormal;  ///< local path binormal
    double m_pcurvature;     ///< local curvature

    bool m_filters_initialized;

    double m_T1_delay;  ///< human driver reaction time = 30 ms, neuromuscular system behavior

    double m_Kp;  // P factor of the PDT1 controller
    double m_Wy;  // weighting factor for y_err (lateral deviation)
    double m_Wh;  // weighting factor for h_err (Heading error)
    double m_Wa;  // weighting factor for a_err (Ackermann angle)

    utils::ChFilterPT1 m_HeadErrDelay;         ///< H(s) = 1 / ( T1 * s + 1 )
    utils::ChFilterPT1 m_AckermannAngleDelay;  ///< H(s) = 1 / ( T1 * s + 1 )
    utils::ChFilterPDT1 m_PathErrCtl;          ///< H(s) = Kp*(0.3*s + 1) / (0.15*s + 1), from the book

    double m_res;      ///< last steering signal
    ChVector<> m_vel;  ///< vehicle velocity vector
};
// --------------------------------------------------------------------------------------------------------------------

/// Path-following steering P-like controller with variable path prediction.
/// The algorithm is from:
///    M.C. Best, "A simple realistic driver model,"
///    AVEC `12: The 11th International Symposium on Advanced Vehicle Control, 9th-12th September 2012, Seoul, Korea.
/// The path to be followed is specified as a ChBezierCurve object and the the original definition points are extracted
/// automatically. Open and closed course definitions can be handled. The ChBezier is still used for visualization.
class CH_VEHICLE_API ChPathSteeringControllerSR : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringControllerSR(std::shared_ptr<ChBezierCurve> path,
                               bool isClosedPath = false,
                               double max_wheel_turn_angle = 0.0,
                               double axle_space = 2.5);

    /// Construct a steering controller to track the specified path.
    /// This version reads controller gains and lookahead distance from the specified JSON file.
    ChPathSteeringControllerSR(const std::string& filename,
                               std::shared_ptr<ChBezierCurve> path,
                               bool isClosedPath = false,
                               double max_wheel_turn_angle = 0.0,
                               double axle_space = 2.5);

    ~ChPathSteeringControllerSR() {}

    void SetGains(double Klat = 0.1, double Kug = 0.0);

    void SetPreviewTime(double Tp = 0.5);

    /// Advance the state of the SR controller.
    virtual double Advance(const ChFrameMoving<>& ref_frame, double time, double step) override;

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location of the sentinel point.
    virtual void Reset(const ChFrameMoving<>& ref_frame) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to the current location of the sentinel
    /// point.
    virtual void CalcTargetLocation() override{};

  private:
    void CalcPathPoints();  ///< extracts path points and direction vectors, make adjustments as needed

    bool m_isClosedPath;  ///< needed for point extraction

    double m_Klat;       ///< lateral controller gain factor
    double m_Kug;        ///< understeering gradient in °/g, can be set to 0 if unknown
    double m_Tp;         ///< prediction time
    double m_L;          ///< effective axlespace (bycicle model)
    double m_delta;      ///< average turn angle of the steered wheels (bycicle model of the vehicle)
    double m_delta_max;  ///< max. allowed average turn angle of the steered wheels (bycicle model of the vehicle)
    double m_umin;       ///< threshold where the controller gets active

    size_t m_idx_curr;              ///< current interval index
    std::vector<ChVector<> > S_l;   ///< course definition points
    std::vector<ChVector<> > R_l;   ///< direction vector: S_l[i+1] = S_l[i] + R_l[i]
    std::vector<ChVector<> > R_lu;  ///< R_l with unit length, precalculated to avoid redundant calculations
};

// --------------------------------------------------------------------------------------------------------------------

/// "Stanley" path-following ontroller named after an autonomous vehicle called Stanley.
/// It minimizes lateral error and heading error. Time delay of the driver's reaction is considered.
/// This driver can be parametrized by a PID json file. It can consider a dead zone left and right to the
/// path, where no path information is recognized. This can be useful when the path information contains
/// lateral disturbances, that could cause bad disturbances of the controller.
/// dead_zone = 0.05 means:
///     0 <= lat_err <= 0.05        no driver reaction
///     0.05 < lat_err <= 2*0.05    smooth transition interval to complete controller engagement
/// The Stanley driver should find 'always' back to the path, despite of great heading or lateral error.
/// If an integral term is used, its state is getting reset every 30 secs to avoid controller wind-up.
///
/// The algorithm comes from from :
///   Gabriel M. Hoffmann, Claire J. Tomlin, Michael Montemerlo, Sebastian Thrun:
///   "Autonomous Automobile Trajectory Tracking for Off-Road Driving", 2005, Stanford University, USA
class CH_VEHICLE_API ChPathSteeringControllerStanley : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringControllerStanley(std::shared_ptr<ChBezierCurve> path, double max_wheel_turn_angle = 0.0);

    /// Construct a steering controller to track the specified path.
    /// This version reads controller gains and lookahead distance from the specified JSON file.
    ChPathSteeringControllerStanley(const std::string& filename,
                                    std::shared_ptr<ChBezierCurve> path,
                                    double max_wheel_turn_angle = 0.0);

    ~ChPathSteeringControllerStanley() {}

    /// Set the gains for the PID controller.
    void SetGains(double Kp, double Ki, double Kd);

    void SetDeadZone(double dead_zone = 0.0) { m_deadZone = std::abs(dead_zone); }

    /// Advance the state of the Stanley controller.
    virtual double Advance(const ChFrameMoving<>& ref_frame, double time, double step) override;

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location of the sentinel point.
    virtual void Reset(const ChFrameMoving<>& ref_frame) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to the current location of the sentinel
    /// point.
    virtual void CalcTargetLocation() override;

    double CalcHeadingError(ChVector<>& a, ChVector<>& b);

  private:
    std::shared_ptr<utils::ChFilterPT1> m_delayFilter;
    std::unique_ptr<ChBezierCurveTracker> m_tracker;  ///< path tracker

    double m_pcurvature;    ///< local curvature
    ChVector<> m_ptangent;  ///< local path tangent

    double m_Kp;  ///< Proportional gain
    double m_Ki;  ///< Integral gain
    double m_Kd;  ///< Differential gain

    double m_delta;      ///< average turn angle of the steered wheels (bycicle model of the vehicle)
    double m_delta_max;  ///< max. allowed average turn angle of the steered wheels (bycicle model of the vehicle)
    double m_umin;       ///< threshold where the controller gets active
    double m_Treset;     ///< the integral error gets reset after this time automatically, no wind-up should happen
    double m_deadZone;  ///< lateral zone where no lateral error is recognized, reduces sensitivity to path disturbances
    double m_Tdelay;    ///< delay time to consider driver reaction (around 0.4 sec)
};

/// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
