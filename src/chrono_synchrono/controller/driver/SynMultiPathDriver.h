#ifndef SYN_MULTI_PATH_DRIVER_H
#define SYN_MULTI_PATH_DRIVER_H

#include <string>
#include <tuple>

#include "chrono_synchrono/SynApi.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"

namespace chrono {
namespace synchrono {

/// Concrete path-following steering PID controller with multiple path support
class SYN_API ChMultiplePathSteeringController : public vehicle::ChSteeringController {
  public:
    /// Construct a steering controller to track the specified path.
    /// This version uses default controller parameters (zero gains).
    /// The user is responsible for calling SetGains and SetLookAheadDistance.
    ChMultiplePathSteeringController(std::vector<std::shared_ptr<ChBezierCurve>> paths,
                                     int lane = 0);

    /// Destructor for ChPathSteeringController.
    ~ChMultiplePathSteeringController() {}

    /// Set the gains for the PID controller.
    void SetGains(double Kp, double Ki, double Kd);

    /// Return a pointer to the Bezier curve.
    std::vector<std::shared_ptr<ChBezierCurve>> GetPath() const { return m_path; }

    /// Change the path to the index of the desire path.
    void changePath(int target_lane) { m_lane = target_lane; }

    /// Add another path to the steering Controller, will return the path index in the array.
    unsigned addPath(std::shared_ptr<ChBezierCurve> path);

    /// Reset the PID controller.
    /// This function resets the underlying path tracker using the current location of the sentinel point.
    virtual void Reset(const vehicle::ChVehicle& vehicle) override;

    /// Calculate the current target point location.
    /// The target point is the point on the associated path that is closest to the current location of the sentinel point.
    virtual void CalcTargetLocation() override;

    /// Advance the state of the PID controller.
    /// This function performs the required integration for the integral component of the PID controller and returns the calculated steering value.
    virtual double Advance(const vehicle::ChVehicle& vehicle, double step) override;

  private:
    int m_lane;

    double m_Kp;  ///< Proportional gain
    double m_Ki;  ///< Integral gain
    double m_Kd;  ///< Differential gain

    std::vector<std::shared_ptr<ChBezierCurve>> m_path;            ///< tracked path (piecewise cubic Bezier curve)
    std::vector<std::shared_ptr<ChBezierCurveTracker>> m_tracker;  ///< path tracker
};

/// A driver model that is very similar with ChPathFollowerACCDriver but it uses multiple path steering controllers.
/// This allows the driver to switch to other paths when needed.
///
/// @sa ChMultiplePathSteeringController
/// @sa ChAdaptiveSpeedController
class SYN_API ChMultiPathFollowerACCDriver : public vehicle::ChDriver {
  public:
    /// Construct using the specified Bezier curves.
    ChMultiPathFollowerACCDriver(
        vehicle::ChVehicle& vehicle,                        ///< associated vehicle
        std::vector<std::shared_ptr<ChBezierCurve>> paths,  ///< an array of Bezier curve with target path
        const std::string& path_name,                       ///< name of the path curve
        double target_speed,                                ///< constant target speed
        double target_following_time,                       ///< seconds of following time
        double target_min_distance,                         ///< min following distance
        double current_distance                             ///< current distance to the vehicle in front
    );

    ~ChMultiPathFollowerACCDriver() {}

    /// Set the desired vehicle speed.
    void SetDesiredSpeed(double val) { m_target_speed = val; }

    /// Set the desired number of seconds of following distance.
    void SetDesiredFollowingTime(double val) { m_target_following_time = val; }

    /// Set the desired min distance to the vehicle in front.
    /// This comes into play especially at low to zero speeds.
    void SetDesiredFollowingMinDistance(double val) { m_target_min_distance = val; }

    /// Set the distance to the vehicle in front that the control will track.
    /// Typically this will be called at each time step.
    void SetCurrentDistance(double val) { m_current_distance = val; }

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying steering controller object.
    ChMultiplePathSteeringController& GetSteeringController() { return m_steeringPID; }

    /// Get the underlying speed controller object.
    vehicle::ChAdaptiveSpeedController& GetSpeedController() { return m_speedPID; }

    /// Reset the underlying controllers.
    void Reset();

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

    /// Change to another path in the array.
    void changePath(int path) { m_steeringPID.changePath(path); }

    /// Add another path into the array.
    unsigned addPath(std::shared_ptr<ChBezierCurve> path) {
        return m_steeringPID.addPath(path);
    }

  private:
    void Create();

    ChMultiplePathSteeringController m_steeringPID;  ///< steering controller
    vehicle::ChAdaptiveSpeedController m_speedPID;   ///< speed controller
    double m_target_speed;                           ///< desired vehicle speed
    double m_target_following_time;                  ///< desired min following time gap
    double m_target_min_distance;                    ///< desired min distance to the vehicle in front
    double m_current_distance;                       ///< current distance to the vehicle in front
    std::string m_pathName;                          ///< for path visualization
    double m_throttle_threshold;                     ///< throttle value below which brakes are applied
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_MULTIPATHDRIVER_H