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
#include "chrono/utils/ChFilters.h"

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

/// Concrete path-following steering 3(2) channel PDT1/PT1 controller.
/// The path to be followed is specified as a ChBezierCurve object and the
/// target point is defined to be the point on that path that is closest to the
/// current location of the sentinel point. The sentinel point should never
/// leave the end or beginning of the path.

/// The controller is sensitive to tire relaxiation, when steering oscillations
/// occure and do not calm down after a short driving distance, Kp should be
/// reduced carefully.

class CH_VEHICLE_API ChPathSteeringControllerXT : public ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChPathSteeringControllerXT(std::shared_ptr<ChBezierCurve> path,
                               bool isClosedPath = false,
                               double max_wheel_turn_angle = 0.0);

    /// Construct a steering controller to track the specified path.
    /// This version reads controller gains and lookahead distance from the
    /// specified JSON file.
    ChPathSteeringControllerXT(const std::string& filename,
                               std::shared_ptr<ChBezierCurve> path,
                               bool isClosedPath = false,
                               double max_wheel_turn_angle = 0.0);

    /// Destructor for ChPathSteeringController.
    ~ChPathSteeringControllerXT() {}

    /// Return a pointer to the Bezier curve
    std::shared_ptr<ChBezierCurve> GetPath() const { return m_path; }

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location
    /// of the sentinel point.
    virtual void Reset(const ChVehicle& vehicle) override;

    /// Set the gains (weighting factors) for the eXTended controller.
    void SetGains(double Kp = 0.4, double W_y_err = 1.0, double W_heading_err = 1.0, double W_ackermann = 1.0) {
        m_Kp = Kp;
        m_Wy = W_y_err;
        m_Wh = W_heading_err;
        m_Wa = W_ackermann;
    }

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to
    /// the current location of the sentinel point.
    virtual void CalcTargetLocation() override;

    double Advance(const ChVehicle& vehicle, double step);

  private:
    double CalcHeadingError(ChVector<>& a, ChVector<>& b);
    int CalcCurvatureCode(ChVector<>& a, ChVector<>& b);
    double CalcAckermannAngle();

    std::shared_ptr<ChBezierCurve> m_path;            ///< tracked path (piecewise cubic Bezier curve)
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

/// Concrete path-following steering P-like controller with variable path prediction.
/// The algorithm is from :
/// BEST, M.C., 2012. A simple realistic driver model. Presented at:
/// AVEC `12: The 11th International Symposium on Advanced Vehicle Control, 9th-12th September 2012, Seoul, Korea.
/// The path to be followed is specified as a ChBezierCurve object and the the original
/// definition points are extracted automatically. Open and closed course definitions
/// can be handled. The ChBezier is still used for visualization.

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
    /// This version reads controller gains and lookahead distance from the
    /// specified JSON file.
    ChPathSteeringControllerSR(const std::string& filename,
                               std::shared_ptr<ChBezierCurve> path,
                               bool isClosedPath = false,
                               double max_wheel_turn_angle = 0.0,
                               double axle_space = 2.5);

    /// Destructor for ChPathSteeringController.
    ~ChPathSteeringControllerSR() {}

    /// Return a pointer to the Bezier curve
    std::shared_ptr<ChBezierCurve> GetPath() const { return m_path; }

    void SetGains(double Klat = 0.1, double Kug = 0.0);

    void SetPreviewTime(double Tp = 0.5);

    /// Advance the state of the P controller.
    double Advance(const ChVehicle& vehicle, double step);

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

/// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
