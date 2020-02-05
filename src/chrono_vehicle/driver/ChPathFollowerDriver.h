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
// A driver model that uses a path steering controller and a speed controller.
// The steering controller adjusts the steering input to follow the prescribed
// path.  The output from the speed controller is used to adjust throttle and
// braking inputs in order to maintain the prescribed constant vehicle speed.
//
// =============================================================================

#ifndef CH_PATHFOLLOWER_DRIVER_H
#define CH_PATHFOLLOWER_DRIVER_H

#include <string>

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
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
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriver : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriver(ChVehicle& vehicle,                   ///< associated vehicle
                         std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
                         const std::string& path_name,         ///< name of the path curve
                         double target_speed,                  ///< constant target speed
                         bool isClosedPath = false             ///< Treat the path as a closed loop
                         );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerDriver(ChVehicle& vehicle,                    ///< associated vehicle
                         const std::string& steering_filename,  ///< JSON file with steering controller specification
                         const std::string& speed_filename,     ///< JSON file with speed controller specification
                         std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
                         const std::string& path_name,          ///< name of the path curve
                         double target_speed,                   ///< constant target speed
                         bool isClosedPath = false              ///< Treat the path as a closed loop
                         );

    ~ChPathFollowerDriver() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChPathSteeringController& GetSteeringController() { return m_steeringPID; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChPathSteeringController m_steeringPID;  ///< steering controller
    ChSpeedController m_speedPID;            ///< speed controller
    double m_target_speed;                   ///< desired vehicle speed
    std::string m_pathName;                  ///< for path visualization
    double m_throttle_threshold;             ///< throttle value below which brakes are applied
};

/// Alternative closed-loop path-follower driver model.
/// A driver model that uses a path steering controller and a speed controller.
/// The steering controller adjusts the steering input to follow the prescribed
/// path.  The output from the speed controller is used to adjust throttle and
/// braking inputs in order to maintain the prescribed constant vehicle speed.
///
/// Implemented from an algorithm found in:
/// "Automotive Control Systems - For Engine, Driveline and Vehicle",
/// Kiencke, U.; Nielson, L:
/// ISBN 3-540-23139-0 Springer Berlin Heidelberg New York
///
/// @sa ChPathSteeringControllerXT
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriverXT : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriverXT(
        ChVehicle& vehicle,                   ///< associated vehicle
        std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
        const std::string& path_name,         ///< name of the path curve
        double target_speed,                  ///< constant target speed
        bool isClosedPath = false,            ///< Treat the path as a closed loop
        double maxWheelTurnAngle = 0.0        ///< needed for wheeled vehicles, use default for tracked vehicles
        );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerDriverXT(
        ChVehicle& vehicle,                    ///< associated vehicle
        const std::string& steering_filename,  ///< JSON file with steering controller specification
        const std::string& speed_filename,     ///< JSON file with speed controller specification
        std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
        const std::string& path_name,          ///< name of the path curve
        double target_speed,                   ///< constant target speed
        bool isClosedPath = false,             ///< Treat the path as a closed loop
        double maxWheelTurnAngle = 0.0         ///< needed for wheeled vehicles, use default for tracked vehicles
        );

    ~ChPathFollowerDriverXT() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChPathSteeringControllerXT& GetSteeringController() { return m_steeringXT; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChPathSteeringControllerXT m_steeringXT;  ///< steering controller
    ChSpeedController m_speedPID;             ///< speed controller
    double m_target_speed;                    ///< desired vehicle speed
    std::string m_pathName;                   ///< for path visualization
    double m_throttle_threshold;              ///< throttle value below which brakes are applied
};

///
/// @sa ChPathSteeringControllerSR
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriverSR : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriverSR(
        ChVehicle& vehicle,                   ///< associated vehicle
        std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
        const std::string& path_name,         ///< name of the path curve
        double target_speed,                  ///< constant target speed
        bool isClosedPath = false,            ///< Treat the path as a closed loop
        double maxWheelTurnAngle = 0.0,       ///< needed for wheeled vehicles, use default for tracked vehicles
        double axle_space = 2.5               ///< needed for course prediction
        );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerDriverSR(
        ChVehicle& vehicle,                    ///< associated vehicle
        const std::string& steering_filename,  ///< JSON file with steering controller specification
        const std::string& speed_filename,     ///< JSON file with speed controller specification
        std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
        const std::string& path_name,          ///< name of the path curve
        double target_speed,                   ///< constant target speed
        bool isClosedPath = false,             ///< Treat the path as a closed loop
        double maxWheelTurnAngle = 0.0,        ///< needed for wheeled vehicles, use default for tracked vehicles
        double axle_space = 2.5                ///< needed for course prediction
        );

    ~ChPathFollowerDriverSR() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChPathSteeringControllerSR& GetSteeringController() { return m_steeringSR; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChPathSteeringControllerSR m_steeringSR;  ///< steering controller
    ChSpeedController m_speedPID;             ///< speed controller
    double m_target_speed;                    ///< desired vehicle speed
    std::string m_pathName;                   ///< for path visualization
    double m_throttle_threshold;              ///< throttle value below which brakes are applied
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
