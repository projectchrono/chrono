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

#include "chrono/assets/ChColor.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

// --------------------------------------------------------------------------------------------------------------------

/// Base class for closed-loop path-follower driver modesl.
/// This driver model uses a path steering controller and a speed controller.
/// The steering controller adjusts the steering input to follow the prescribed path.  The output from the speed
/// controller is used to adjust throttle and braking inputs in order to maintain the prescribed constant vehicle speed.
///
/// Derived classes differ in the type of laterla steering controller.
///
/// @sa ChSteeringController
/// @sa ChSpeedController
class CH_VEHICLE_API ChClosedLoopDriver : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChClosedLoopDriver(ChVehicle& vehicle,            ///< associated vehicle
                       const std::string& path_name,  ///< name of the path curve
                       double target_speed            ///< constant target speed
    );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChClosedLoopDriver(ChVehicle& vehicle,                 ///< associated vehicle
                       const std::string& speed_filename,  ///< JSON file with speed controller specification
                       const std::string& path_name,       ///< name of the path curve
                       double target_speed                 ///< constant target speed
    );

    virtual ~ChClosedLoopDriver() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThresholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() const { return *m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Initialize this driver system.
    virtual void Initialize() override;

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Set color of path visualization.
    void SetColor(const ChColor& color) { m_color = color; }

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

  protected:
    std::unique_ptr<ChSteeringController> m_steeringPID;  ///< steering controller
    std::unique_ptr<ChSpeedController> m_speedPID;        ///< speed controller
    double m_target_speed;                                ///< desired vehicle speed
    std::string m_pathName;                               ///< for path visualization
    ChColor m_color;                                      ///< for path visualization
    double m_throttle_threshold;                          ///< throttle value below which brakes are applied
};

// --------------------------------------------------------------------------------------------------------------------

/// Path-following driver system using a default PID lateral steering controller.
///
/// @sa ChPathSteeringController
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriver : public ChClosedLoopDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriver(ChVehicle& vehicle,                   ///< associated vehicle
                         std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
                         const std::string& path_name,         ///< name of the path curve
                         double target_speed                   ///< constant target speed
    );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerDriver(ChVehicle& vehicle,                    ///< associated vehicle
                         const std::string& steering_filename,  ///< JSON file with steering controller specification
                         const std::string& speed_filename,     ///< JSON file with speed controller specification
                         std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
                         const std::string& path_name,          ///< name of the path curve
                         double target_speed                    ///< constant target speed
    );

    ~ChPathFollowerDriver() {}

    /// Access the underlying steering controller object.
    ChPathSteeringController& GetSteeringController() const;
};

// --------------------------------------------------------------------------------------------------------------------

/// Path-following driver system using an extended PID lateral steering controller.
///
/// Implemented from an algorithm found in:
/// "Automotive Control Systems - For Engine, Driveline and Vehicle",
/// Kiencke, U.; Nielson, L:
/// ISBN 3-540-23139-0 Springer Berlin Heidelberg New York
///
/// @sa ChPathSteeringControllerXT
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriverXT : public ChClosedLoopDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriverXT(
        ChVehicle& vehicle,                   ///< associated vehicle
        std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
        const std::string& path_name,         ///< name of the path curve
        double target_speed,                  ///< constant target speed
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
        double maxWheelTurnAngle = 0.0         ///< needed for wheeled vehicles, use default for tracked vehicles
    );

    ~ChPathFollowerDriverXT() {}

    /// Get the underlying steering controller object.
    ChPathSteeringControllerXT& GetSteeringController() const;
};

// --------------------------------------------------------------------------------------------------------------------

/// Path-following driver system using a P-like lateral steering controller with variable path prediction.
///
/// @sa ChPathSteeringControllerSR
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriverSR : public ChClosedLoopDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriverSR(
        ChVehicle& vehicle,                   ///< associated vehicle
        std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
        const std::string& path_name,         ///< name of the path curve
        double target_speed,                  ///< constant target speed
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
        double maxWheelTurnAngle = 0.0,        ///< needed for wheeled vehicles, use default for tracked vehicles
        double axle_space = 2.5                ///< needed for course prediction
    );

    ~ChPathFollowerDriverSR() {}

    /// Get the underlying steering controller object.
    ChPathSteeringControllerSR& GetSteeringController() const;
};

/// Path-following driver system using a lateral steering controller as used on the Stanley AV.
///
/// @sa ChPathSteeringControllerStanley
/// @sa ChSpeedController
class CH_VEHICLE_API ChPathFollowerDriverStanley : public ChClosedLoopDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChPathFollowerDriverStanley(
        ChVehicle& vehicle,                   ///< associated vehicle
        std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
        const std::string& path_name,         ///< name of the path curve
        double target_speed,                  ///< constant target speed
        double maxWheelTurnAngle = 0.0        ///< needed for wheeled vehicles, use default for tracked vehicles
    );

    /// Construct using JSON specification files.
    /// The two files must contain specification for the path-follower steering controller
    /// and the constant-speed controller, respectively.
    ChPathFollowerDriverStanley(
        ChVehicle& vehicle,                    ///< associated vehicle
        const std::string& steering_filename,  ///< JSON file with steering controller specification
        const std::string& speed_filename,     ///< JSON file with speed controller specification
        std::shared_ptr<ChBezierCurve> path,   ///< Bezier curve with target path
        const std::string& path_name,          ///< name of the path curve
        double target_speed,                   ///< constant target speed
        double maxWheelTurnAngle = 0.0         ///< needed for wheeled vehicles, use default for tracked vehicles
    );

    ~ChPathFollowerDriverStanley() {}

    /// Get the underlying steering controller object.
    ChPathSteeringControllerStanley& GetSteeringController() const;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
