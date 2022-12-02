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
// Authors: Mike Taylor, Radu Serban
// =============================================================================
//
// A driver model that uses a path steering controller and a speed controller.
// The steering controller adjusts the steering input to follow the prescribed
// path.  The output from the speed controller is used to adjust throttle and
// braking inputs in order to maintain the prescribed constant vehicle speed.
//
// =============================================================================

#ifndef CH_PATHFOLLOWER_ACC_DRIVER_H
#define CH_PATHFOLLOWER_ACC_DRIVER_H

#include <string>

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Closed-loop path-follower driver model.
/// A driver model that uses a path steering controller and a speed controller.
/// The steering controller adjusts the steering input to follow the prescribed
/// path.  The output from the speed controller is used to adjust throttle and
/// braking inputs in order to maintain the prescribed constant vehicle speed.
///
/// @sa ChPathSteeringController
/// @sa ChAdaptiveSpeedController
class CH_VEHICLE_API ChPathFollowerACCDriver : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerACCDriver(ChVehicle& vehicle,                   ///< associated vehicle
                            std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
                            const std::string& path_name,         ///< name of the path curve
                            double target_speed,                  ///< constant target speed
                            double target_following_time,         ///< seconds of following time
                            double target_min_distance,           ///< min following distance
                            double current_distance               ///< current distance to the vehicle in front
    );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerACCDriver(ChVehicle& vehicle,                    ///< associated vehicle
                            const std::string& steering_filename,  ///< JSON file with steering controller specification
                            const std::string& speed_filename,     ///< JSON file with speed controller specification
                            std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
                            const std::string& path_name,          ///< name of the path curve
                            double target_speed,                   ///< constant target speed
                            double target_following_time,          ///< seconds of following time
                            double target_min_distance,            ///< min following distance
                            double current_distance                ///< current distance to the vehicle in front
    );

    ~ChPathFollowerACCDriver() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Set the desired number of seconds of following distance
    void SetDesiredFollowingTime(double val) { m_target_following_time = val; }

    /// Set the desired min distance to the vehicle in front
    /// This comes into play especially at low to zero speeds
    void SetDesiredFollowingMinDistance(double val) { m_target_min_distance = val; }

    /// Set the distance to the vehicle in front that the control will track
    /// Typically this will be called at each time step
    void SetCurrentDistance(double val) { m_current_distance = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThresholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChPathSteeringController& GetSteeringController() { return m_steeringPID; }

    /// Get the underlying speed controller object.
    ChAdaptiveSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChPathSteeringController m_steeringPID;  ///< steering controller
    ChAdaptiveSpeedController m_speedPID;    ///< speed controller
    double m_target_speed;                   ///< desired vehicle speed
    double m_target_following_time;          ///< desired min following time gap
    double m_target_min_distance;            ///< desired min distance to the vehicle in front
    double m_current_distance;               ///< current distance to the vehicle in front
    std::string m_pathName;                  ///< for path visualization
    double m_throttle_threshold;             ///< throttle value below which brakes are applied
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
