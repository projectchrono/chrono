// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// A driver model that combines a path steering controller and a speed controller.
// The controller adjusts the steering input to follow the prescribed
// path.  The output also adjusts throttle and braking inputs in order to maintain a varying speed that depends
// on the curvature of the road. This implementation is based on the following paper:
//
// BEST, M.C., 2012. A simple realistic driver model. Presented at:
// AVEC `12: The 11th International Symposium on Advanced Vehicle Control, 9th-12th September 2012, Seoul, Korea.
// The path to be followed is specified as a ChBezierCurve object and the the original
// definition points are extracted automatically. Open and closed course definitions
// can be handled. The ChBezier is still used for visualization.
//
// =============================================================================

#ifndef CH_HUMAN_DRIVER_H
#define CH_HUMAN_DRIVER_H

#include <string>

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChFilters.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Closed-loop path-follower and speed maintaining driver model.
/// A driver model that combines a path steering controller and a speed controller.
/// The controller adjusts the steering input to follow the prescribed
/// path.  The output also adjusts throttle and braking inputs in order to maintain a varying speed that depends
/// on the curvature of the road.
///
class CH_VEHICLE_API ChHumanDriver : public ChDriver {
  public:
    /// Construct using the specified Bezier curve.
    ChHumanDriver(ChVehicle& vehicle,                   ///< associated vehicle
                  std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
                  const std::string& path_name,         ///< name of the path curve
                  double road_width = 5.0,              ///< road width
                  double max_wheel_turn_angle = 0,      ///< maximum wheel turning angle
                  double axle_space = 2.5               ///< wheel track
    );
    /// Construct using a JSON parameter file
    ChHumanDriver(const std::string& filename,          ///< path of the JSON file
                  ChVehicle& vehicle,                   ///< associated vehicle
                  std::shared_ptr<ChBezierCurve> path,  ///< Bezier curve with target path
                  const std::string& path_name,         ///< name of the path curve
                  double road_width = 5.0,              ///< road width
                  double max_wheel_turn_angle = 0,      ///< maximum wheel turning angle
                  double axle_space = 2.5               ///< wheel track
    );
    ~ChHumanDriver() {}

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

    /// Initialize the driver system
    virtual void Initialize() override;

    /// Export the Bezier curve for POV-Ray postprocessing.
    void ExportPathPovray(const std::string& out_dir);

    void SetPreviewTime(double Tp = 0.5) { m_Tp = ChClamp(Tp, 0.2, 0.7); }

    void SetLateralGains(double Klat = 0.1, double Kug = 0) {
        m_Klat = ChClamp(Klat, 0.02, 0.2);
        m_Kug = ChClamp(Kug, 0.0, 5.0);
    }

    void SetLongitudinalGains(double Klong = 0.1, double Kplus = 0.1, double Kminus = 0.1) {
        m_Klong = ChClamp(Klong, 0.05, 0.2);
        m_Kplus = ChClamp(Kplus, 0.05, 1.0);
        m_Kminus = ChClamp(Kminus, 0.05, 1.0);
    }

    void SetSpeedRange(double u0 = 10.0, double umax = 30.0);

    ChVector<> GetTargetLocation() { return m_target; }

    ChVector<> GetSentinelLocation() { return m_sentinel; }

    double GetTraveledDistance() { return m_distance; }

    double GetAverageSpeed() { return m_travel_time > 0.0 ? m_distance / m_travel_time : 0.0; }
    double GetMaxSpeed() { return m_speed_max; }
    double GetMinSpeed() { return m_speed_min; }
    double GetMaxLatAcc() { return m_left_acc; }
    double GetMinLatAcc() { return m_right_acc; }

  private:
    void Create();
    size_t GetNextI();
    size_t GetNextJ();
    std::shared_ptr<ChBezierCurve> m_path;
    std::string m_pathName;  ///< for path visualization
    double m_road_width;
    double m_Tp;  ///< preview time
    double m_Klat;
    double m_Kug;
    double m_Klong;
    double m_Kplus;
    double m_Kminus;
    double m_u0;
    double m_umax;
    double m_uthres;
    ChVector<> m_target;
    ChVector<> m_sentinel;
    size_t m_idx_curr;
    size_t m_i_curr;
    size_t m_j_curr;
    std::vector<ChVector<> > m_S_l;   ///< course definition points
    std::vector<ChVector<> > m_R_l;   ///< direction vector: S_l[i+1] = S_l[i] + R_l[i]
    std::vector<ChVector<> > m_R_lu;  ///< R_l with unit length, precalculated to avoid redundant calculations
    std::vector<ChVector<> > m_Li;    ///< left road border
    std::vector<ChVector<> > m_Rj;    ///< right road border
    double m_delta;
    double m_delta_max;
    double m_L;
    bool m_run_once;
    double m_distance;
    double m_travel_time;
    utils::ChFilterI m_UIntegrator;
    utils::ChButterworth_Lowpass m_acc_filter;
    double m_speed_max;
    double m_speed_min;
    double m_left_acc;
    double m_right_acc;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // namespace chrono

#endif
