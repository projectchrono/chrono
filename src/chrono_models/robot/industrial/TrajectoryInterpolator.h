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
// Classes to interpolate a list of waypoints in joint or operation space.
//
// =============================================================================

#ifndef TRAJECTORY_INTERPOLATOR_H
#define TRAJECTORY_INTERPOLATOR_H

#include "chrono_models/ChApiModels.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionRotation.h"
#include "chrono/functions/ChFunctionSequence.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

// =============================================================================
// TrajectoryInterpolator base class
// =============================================================================
class CH_MODELS_API TrajectoryInterpolator {
  public:
    /// Default constructor.
    TrajectoryInterpolator() {}

    /// Default destructor.
    virtual ~TrajectoryInterpolator() {}

    /// Set total time to complete trajectory.
    /// NB: needs manual call to Setup() to have effect.
    virtual void SetTotalMotionTime(double motion_time_tot) { m_motion_time_tot = motion_time_tot; }

    /// Get total time to complete trajectory.
    virtual double GetTotalMotionTime() const { return m_motion_time_tot; }

    /// Set individual times to travel each part of the trajectory.
    /// NB: needs manual call to Setup() to have effect.
    virtual void SetDurations(const std::vector<double>& durations) = 0;

    /// Get individual times to travel each part of the trajectory.
    virtual std::vector<double> GetDurations() const { return m_durations; }

    /// Get cumulative times of trajectory travelling.
    virtual std::vector<double> GetMotionTimesCumulative() const { return m_cumul_times; }

  protected:
    /// Auto-compute durations based on weighted average of trajectory length.
    virtual void AutoComputeTrajectoryDurations() = 0;

    double m_motion_time_tot = 0;            ///< total time to travel trajectory
    std::vector<double> m_durations = {};    ///< individual trajectory parts times
    std::vector<double> m_cumul_times = {};  ///< cumulative trajectory parts times
    double m_traj_length_tot = 0;            ///< total length of trajectory
};

// =============================================================================
// TrajectoryInterpolatorOperationSpace
// =============================================================================
/// Interpolate a list of 3D waypoints with assigned geometric and time motion functions.
class CH_MODELS_API TrajectoryInterpolatorOperationSpace : public TrajectoryInterpolator {
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
    TrajectoryInterpolatorOperationSpace() {}

    /// Default destructor.
    virtual ~TrajectoryInterpolatorOperationSpace() {}

    /// Create a trajectory interpolator from given operation-space waypoints and total motion time.
    /// Trajectory is defined by given geometric position and rotation functions, which are evaluated in time through
    /// provided space functions.
    /// Individual times to travel each part of the trajectory
    /// - can be manually defined by the user: in this case, their sum must equal total motion time parameter
    /// - can be skipped: in this case, durations are automatically computed as weighted average of total path length
    TrajectoryInterpolatorOperationSpace(
        double motion_time_tot,                     ///< total time to complete trajectory
        const std::vector<ChCoordsysd>& waypoints,  ///< input trajectory waypoints
        PosfunType posfun_type,                     ///< type of geometric position function
        SpacefunType pos_spacefun_type,             ///< type of space function used to evaluate position
        RotfunType rotfun_type,                     ///< type of geometric rotation function
        SpacefunType rot_spacefun_type,             ///< type of space function used to evaluate rotation
        std::vector<double>* durations = nullptr  ///< times to travel individual trajectory parts; autocompute if empty
    );

    /// Setup interpolator internal data.
    /// NB: must to be manually called after settings are changed.
    void Setup(
        double motion_time_tot,                     ///< total time to complete trajectory
        const std::vector<ChCoordsysd>& waypoints,  ///< input trajectory waypoints
        PosfunType posfun_type,                     ///< type of geometric position function
        SpacefunType pos_spacefun_type,             ///< type of space function used to evaluate position
        RotfunType rotfun_type,                     ///< type of geometric rotation function
        SpacefunType rot_spacefun_type,             ///< type of space function used to evaluate rotation
        std::vector<double>* durations = nullptr  ///< times to travel individual trajectory parts; autocompute if empty
    );

    /// Get interpolation pose of trajectory, at given time.
    ChCoordsysd GetInterpolation(double time) const;

    /// Set individual times to travel each part of the trajectory.
    /// NB: needs manual call to Setup() to have effect.
    virtual void SetDurations(const std::vector<double>& durations) override;

    /// Get geometric position function.
    std::shared_ptr<ChFunctionPositionLine> GetPositionFunction() const { return m_posfun; }

    /// Get geometric rotation function.
    std::shared_ptr<ChFunctionRotation> GetRotationFunction() const { return m_rotfun; }

    /// Get position evaluation function.
    std::shared_ptr<ChFunctionSequence> GetPositionSpaceFunction() const { return m_pos_spacefun; }

    /// Get rotation evaluation function.
    std::shared_ptr<ChFunctionSequence> GetRotationSpaceFunction() const { return m_rot_spacefun; }

    /// Get input waypoints positions.
    std::vector<ChVector3d> GetPositions() const;

    /// Get input waypoints rotations.
    std::vector<ChQuaterniond> GetRotations() const;

  private:
    /// Auto-compute durations based on weighted average of trajectory length.
    virtual void AutoComputeTrajectoryDurations() override;

    void SetupPositionFunction();

    void SetupRotationFunction();

    std::shared_ptr<ChFunctionSequence> SetupSpaceFunction(SpacefunType spacefun_type);

    std::vector<ChCoordsysd> m_waypoints = {};                     ///< input trajectory waypoints to interpolate
    std::shared_ptr<ChFunctionPositionLine> m_posfun = nullptr;    ///< geometric position function
    std::shared_ptr<ChFunctionSequence> m_pos_spacefun = nullptr;  ///< time function to evaluate position
    std::shared_ptr<ChFunctionRotation> m_rotfun = nullptr;        ///< geometric rotation function
    std::shared_ptr<ChFunctionSequence> m_rot_spacefun = nullptr;  ///< time function to evaluate rotation
    PosfunType m_posfun_type = PosfunType::LINE;                   ///< type of geometric position function
    RotfunType m_rotfun_type = RotfunType::BSPLINE1;               ///< type of geometric rotation function
    SpacefunType m_pos_spacefun_type = SpacefunType::LINEAR;       ///< type of time evaluation function for position
    SpacefunType m_rot_spacefun_type = SpacefunType::LINEAR;       ///< type of time evaluation function for rotation
};

// =============================================================================
// TrajectoryInterpolatorJointSpace
// =============================================================================
/// Interpolate a list of joint waypoints with time motion functions.
class CH_MODELS_API TrajectoryInterpolatorJointSpace : public TrajectoryInterpolator {
  public:
    /// Type of space function used to evaluate trajectory segments.
    enum class SpacefunType { LINEAR, POLY345, CONSTACC, CYCLOIDAL };

    /// Default constructor.
    TrajectoryInterpolatorJointSpace() {}

    /// Default destructor.
    virtual ~TrajectoryInterpolatorJointSpace() {}

    /// Create a joint-space trajectory interpolator from given joint-space waypoints and total motion time.
    /// Individual times to travel each part of the trajectory
    /// - can be manually defined by the user: in this case, their sum must equal total motion time parameter
    /// - can be skipped: in this case, durations are automatically computed as weighted average of total path length
    TrajectoryInterpolatorJointSpace(double motion_time_tot,
                                     const std::vector<ChVectorDynamic<>>& waypoints,
                                     SpacefunType spacefun,
                                     std::vector<double>* durations = nullptr);

    /// Setup interpolator internal data.
    /// NB: must to be manually called after settings are changed.
    void Setup(double motion_time_tot,
               const std::vector<ChVectorDynamic<>>& waypoints,
               SpacefunType spacefun,
               std::vector<double>* durations = nullptr);

    /// Get interpolated joints configuration, at given time.
    ChVectorDynamic<> GetInterpolation(double time) const;

    /// Set individual times to travel each part of the trajectory.
    /// NB: needs manual call to Setup() to have effect.
    virtual void SetDurations(const std::vector<double>& durations) override;

    /// Get motion functions.
    std::vector<std::shared_ptr<ChFunctionSequence>> GetMotionFunctions() const { return m_motfuns; }

  private:
    /// Auto-compute durations based on weighted average of trajectory length.
    virtual void AutoComputeTrajectoryDurations() override;

    unsigned int m_num_joints = 0;
    std::vector<ChVectorDynamic<>> m_waypoints = {};
    SpacefunType m_spacefun_type = SpacefunType::LINEAR;
    std::vector<std::shared_ptr<ChFunctionSequence>> m_motfuns = {};
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end TRAJECTORY_INTERPOLATOR_H