// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Class to interpolate a list of waypoints with assigned geometric and
// time motion functions.
//
// =============================================================================

#ifndef TRAJECTORY_INTERPOLATOR_H
#define TRAJECTORY_INTERPOLATOR_H

#include "chrono_models/ChApiModels.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRotation.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API TrajectoryInterpolator {
  public:
    /// Type of geometric position function used to define trajectory.
    enum class PosfunType { LINE, BSPLINE2, BEZIER };

    /// Type of geometric rotation function used to define trajectory.
    enum class RotfunType { BSPLINE1, BSPLINE2, SQUAD };

    /// Type of space function used to evaluate geometric functions.
    /// Two categories available:
    /// - single shot: interpolate whole trajectory with a single function
    /// - piecewise: interpolate individual trajectory parts with dedicated functions
    enum class SpacefunType { LINEAR, POLY345, CONSTACC, CYCLOIDAL, PW_LINEAR, PW_POLY345, PW_CONSTACC, PW_CYCLOIDAL };

    /// Default constructor.
    TrajectoryInterpolator(){};

    /// Create a trajectory interpolator from given waypoints and total motion time.
    /// Trajectory is defined by given geometric position and rotation functions, which are evaluated in time through
    /// provided space functions.
    /// Individual times to travel each part of the trajectory
    /// - can be manually defined by the user: in this case, their sum must equal total motion time parameter
    /// - can be skipped: in this case, durations are automatically computed as weighted average of total path length
    TrajectoryInterpolator(
        const std::vector<ChCoordsysd>& waypoints,  ///< input trajectory waypoints
        double motion_time_tot,                     ///< total time to complete trajectory
        PosfunType posfun_type,                     ///< type of geometric position function
        SpacefunType pos_spacefun_type,             ///< type of space function used to evaluate position
        RotfunType rotfun_type,                     ///< type of geometric rotation function
        SpacefunType rot_spacefun_type,             ///< type of space function used to evaluate rotation
        std::vector<double>* durations = nullptr  ///< times to travel individual trajectory parts; autocompute if empty
    );

    /// Setup interpolator internal data.
    /// NB: must to be manually called after settings are changed.
    void Setup(
        const std::vector<ChCoordsysd>& waypoints,  ///< input trajectory waypoints
        double motion_time_tot,                     ///< total time to complete trajectory
        PosfunType posfun_type,                     ///< type of geometric position function
        SpacefunType pos_spacefun_type,             ///< type of space function used to evaluate position
        RotfunType rotfun_type,                     ///< type of geometric rotation function
        SpacefunType rot_spacefun_type,             ///< type of space function used to evaluate rotation
        std::vector<double>* durations = nullptr  ///< times to travel individual trajectory parts; autocompute if empty
    );

    /// Get interpolation pose of trajectory, at given time.
    ChCoordsysd GetInterpolation(double time) const;

    /// Get geometric position function.
    std::shared_ptr<ChFunctionPositionLine> GetPositionFunction() const { return m_posfun; }

    /// Get geometric rotation function.
    std::shared_ptr<ChFunctionRotation> GetRotationFunction() const { return m_rotfun; }

    /// Get input waypoints positions.
    std::vector<ChVector3d> GetPositions() const;

    /// Get input waypoints rotations.
    std::vector<ChQuaterniond> GetRotations() const;

    /// Set individual times to travel each part of the trajectory.
    /// NB: needs manual call to Setup() to have effect.
    void SetDurations(const std::vector<double>& durations);

    /// Get individual times to travel each part of the trajectory.
    std::vector<double> GetDurations() const { return m_durations; }

    /// Set total time to complete trajectory.
    /// NB: needs manual call to Setup() to have effect.
    void SetTotalMotionTime(double motion_time_tot) { m_motion_time_tot = motion_time_tot; }

    /// Get total time to complete trajectory.
    double GetTotalMotionTime() const { return m_motion_time_tot; }

    /// Get cumulative times of trajectory travelling.
    std::vector<double> GetMotionTimesCumulative() const { return m_cum_times; }

  private:
    /// Auto-compute durations based on weighted average of trajectory length.
    void AutoComputeTrajectoryDurations();

    void SetupPositionFunction();

    void SetupRotationFunction();

    std::shared_ptr<ChFunction> SetupSpaceFunction(SpacefunType spacefun_type);

    std::vector<ChCoordsysd> m_waypoints = {};                   ///< input trajectory waypoints to interpolate
    std::vector<double> m_durations = {};                        ///< individual trajectory parts times
    std::vector<double> m_cum_times = {};                        ///< cumulative trajectory parts times
    double m_motion_time_tot = 0;                                ///< total time to travel trajectory
    double m_traj_length_tot = 0;                                ///< total length of trajectory
    std::shared_ptr<ChFunctionPositionLine> m_posfun = nullptr;  ///< geometric position function
    std::shared_ptr<ChFunction> m_pos_spacefun = nullptr;        ///< time function to evaluate position
    std::shared_ptr<ChFunctionRotation> m_rotfun = nullptr;      ///< geometric rotation function
    std::shared_ptr<ChFunction> m_rot_spacefun = nullptr;        ///< time function to evaluate rotation
    PosfunType m_posfun_type = PosfunType::LINE;                 ///< type of geometric position function
    RotfunType m_rotfun_type = RotfunType::BSPLINE1;             ///< type of geometric rotation function
    SpacefunType m_pos_spacefun_type = SpacefunType::LINEAR;     ///< type of time evaluation function for position
    SpacefunType m_rot_spacefun_type = SpacefunType::LINEAR;     ///< type of time evaluation function for rotation
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end TRAJECTORY_INTERPOLATOR_H