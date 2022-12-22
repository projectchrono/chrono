#include <algorithm>

#include "chrono/core/ChMathematics.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChWorldFrame.h"

#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

ChMultiPathFollowerACCDriver::ChMultiPathFollowerACCDriver(
    ChVehicle& vehicle,
    std::vector<std::shared_ptr<ChBezierCurve>> paths,
    const std::string& path_name,
    double target_speed,
    double target_following_time,
    double target_min_distance,
    double current_distance)
    : ChDriver(vehicle),
      m_steeringPID(paths),
      m_target_speed(target_speed),
      m_target_following_time(target_following_time),
      m_target_min_distance(target_min_distance),
      m_current_distance(current_distance),
      m_pathName(path_name),
      m_throttle_threshold(0.2) {
    Create();
}

void ChMultiPathFollowerACCDriver::Create() {
    // Reset the steering and speed controllers
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);

    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringPID.GetPath();
    for (int i = 0; i < bezier_curve.size(); i++) {
        auto num_points = static_cast<unsigned int>(bezier_curve[i]->getNumPoints());
        auto path_asset = chrono_types::make_shared<ChLineShape>();
        path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(bezier_curve[i]));
        path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
        path_asset->SetName(m_pathName);
        path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
        road->AddVisualShape(path_asset);
    }
}

void ChMultiPathFollowerACCDriver::Reset() {
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void ChMultiPathFollowerACCDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, m_target_following_time, m_target_min_distance,
                                          m_current_distance, step);
    ChClampValue(out_speed, -1.0, 1.0);

    if (out_speed > 0) {
        // Vehicle moving too slow
        m_braking = 0;
        m_throttle = out_speed;
    } else if (m_throttle > m_throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        m_braking = 0;
        m_throttle = 1 + out_speed;
    } else {
        // Vehicle moving too fast: apply brakes
        m_braking = -out_speed;
        m_throttle = 0;
    }
    // Set the steering value based on the output from the steering controller.
    double out_steering = m_steeringPID.Advance(m_vehicle, step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void ChMultiPathFollowerACCDriver::ExportPathPovray(const std::string& out_dir) {
    for (int i = 0; i < m_steeringPID.GetPath().size(); i++) {
        utils::WriteCurvePovray(*(m_steeringPID.GetPath()[i]), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
    }
}

// -----------------------------------------------------------------------------
// Implementation of the derived class ChMultiplePathSteeringController.
// -----------------------------------------------------------------------------
ChMultiplePathSteeringController::ChMultiplePathSteeringController(std::vector<std::shared_ptr<ChBezierCurve>> paths,
                                                                   int lane)
    : ChSteeringController(nullptr), m_lane(lane), m_Kp(0), m_Ki(0), m_Kd(0), m_path(paths) {
    for (const auto& path : m_path) {
        auto tracker_element = chrono_types::make_shared<ChBezierCurveTracker>(path);
        m_tracker.push_back(tracker_element);
    }
}

void ChMultiplePathSteeringController::SetGains(double Kp, double Ki, double Kd) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

void ChMultiplePathSteeringController::CalcTargetLocation() {
    // Let the underlying tracker do its magic.
    m_tracker[m_lane]->calcClosestPoint(m_sentinel, m_target);
}

void ChMultiplePathSteeringController::Reset(const ChVehicle& vehicle) {
    // Let the base class calculate the current location of the sentinel point.
    ChSteeringController::Reset(vehicle);

    // Reset the path tracker with the new sentinel location
    for (int i = 0; i < m_tracker.size(); i++) {
        m_tracker[i]->reset(m_sentinel);
    }
}

double ChMultiplePathSteeringController::Advance(const ChVehicle& vehicle, double step) {
    // Calculate current "sentinel" location.  This is a point at the look-ahead
    // distance in front of the vehicle.
    m_sentinel =
        vehicle.GetChassisBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(m_dist * ChWorldFrame::Forward());

    // Calculate current "target" location.
    CalcTargetLocation();

    // If data collection is enabled, append current target and sentinel locations.
    if (m_collect) {
        *m_csv << vehicle.GetChTime() << m_target << m_sentinel << std::endl;
    }

    // The "error" vector is the projection onto the horizontal plane of the vector between sentinel and target.
    ChVector<> err_vec = m_target - m_sentinel;
    ChWorldFrame::Project(err_vec);

    // Calculate the sign of the angle between the projections of the sentinel
    // vector and the target vector (with origin at vehicle location).
    ChVector<> sentinel_vec = m_sentinel - vehicle.GetPos();
    ChWorldFrame::Project(sentinel_vec);
    ChVector<> target_vec = m_target - vehicle.GetPos();
    ChWorldFrame::Project(target_vec);

    double temp = Vdot(Vcross(sentinel_vec, target_vec), ChWorldFrame::Vertical());

    // Calculate current error (magnitude).
    double err = ChSignum(temp) * err_vec.Length();

    // Estimate error derivative (backward FD approximation).
    m_errd = (err - m_err) / step;

    // Calculate current error integral (trapezoidal rule).
    m_erri += (err + m_err) * step / 2;

    // Cache new error
    m_err = err;

    // Return PID output (steering value)
    return m_Kp * m_err + m_Ki * m_erri + m_Kd * m_errd;
}

unsigned ChMultiplePathSteeringController::addPath(std::shared_ptr<ChBezierCurve> path) {
    m_path.push_back(path);
    auto m_tracker_element = chrono_types::make_shared<ChBezierCurveTracker>(path);
    m_tracker.push_back(m_tracker_element);
    return (unsigned)(m_tracker.size() - 1);
}

}  // namespace synchrono
}  // namespace chrono
