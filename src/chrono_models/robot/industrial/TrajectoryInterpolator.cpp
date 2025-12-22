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

#include "TrajectoryInterpolator.h"

#include "chrono/functions/ChFunction.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationSQUAD.h"

namespace chrono {
namespace industrial {

// =============================================================================
// TrajectoryInterpolatorOperationSpace
// =============================================================================
TrajectoryInterpolatorOperationSpace::TrajectoryInterpolatorOperationSpace(double motion_time_tot,
                                                                           const std::vector<ChCoordsysd>& waypoints,
                                                                           PosfunType posfun_type,
                                                                           SpacefunType pos_spacefun_type,
                                                                           RotfunType rotfun_type,
                                                                           SpacefunType rot_spacefun_type,
                                                                           std::vector<double>* durations) {
    Setup(motion_time_tot, waypoints, posfun_type, pos_spacefun_type, rotfun_type, rot_spacefun_type, durations);
}

void TrajectoryInterpolatorOperationSpace::Setup(double motion_time_tot,
                                                 const std::vector<ChCoordsysd>& waypoints,

                                                 PosfunType posfun_type,
                                                 SpacefunType pos_spacefun_type,
                                                 RotfunType rotfun_type,
                                                 SpacefunType rot_spacefun_type,
                                                 std::vector<double>* durations) {
    // Set data
    m_waypoints = waypoints;
    m_motion_time_tot = motion_time_tot;
    m_posfun_type = posfun_type;
    m_pos_spacefun_type = pos_spacefun_type;
    m_rotfun_type = rotfun_type;
    m_rot_spacefun_type = rot_spacefun_type;

    // If individual motion durations are not provided, auto-compute them based on trajectory lengths
    if (durations) {
        m_durations = *durations;
        double check_motion_time_tot = 0;
        for (const auto& d : m_durations)
            check_motion_time_tot += d;

        if (check_motion_time_tot != m_motion_time_tot) {
            std::cerr << "Sum of durations is different from total motion time" << std::endl;
            throw std::invalid_argument("Sum of durations is different from total motion time");
        }
    } else {
        AutoComputeTrajectoryDurations();
    }

    // Position interpolation function
    SetupPositionFunction();

    // Rotation interpolation function
    SetupRotationFunction();
}

void TrajectoryInterpolatorOperationSpace::SetupPositionFunction() {
    // Collect waypoint positions
    std::vector<ChVector3d> pos = GetPositions();

    // Initialize common position function
    m_posfun = chrono_types::make_shared<ChFunctionPositionLine>();

    switch (m_posfun_type) {
        // LINE
        case TrajectoryInterpolatorOperationSpace::PosfunType::LINE: {
            m_posfun->SetLine(chrono_types::make_shared<ChLineBSpline>(1, pos));
            break;
        }

        // BSPLINE2
        case TrajectoryInterpolatorOperationSpace::PosfunType::BSPLINE2:
            m_posfun->SetLine(chrono_types::make_shared<ChLineBSpline>(2, pos));
            break;

        // BEZIER
        case TrajectoryInterpolatorOperationSpace::PosfunType::BEZIER:
            m_posfun->SetLine(chrono_types::make_shared<ChLineBezier>(chrono_types::make_shared<ChBezierCurve>(pos)));
            break;

        // DEFAULT
        default:
            break;
    }

    // Set space function
    m_pos_spacefun = SetupSpaceFunction(m_pos_spacefun_type);
    m_posfun->SetSpaceFunction(m_pos_spacefun);
}

void TrajectoryInterpolatorOperationSpace::SetupRotationFunction() {
    // Collect waypoint rotations
    std::vector<ChQuaterniond> rots = GetRotations();

    switch (m_rotfun_type) {
        // BSPLINE1
        case chrono::industrial::TrajectoryInterpolatorOperationSpace::RotfunType::BSPLINE1:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationBSpline>(1, rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationBSpline>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // BSPLINE2
        case chrono::industrial::TrajectoryInterpolatorOperationSpace::RotfunType::BSPLINE2:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationBSpline>(2, rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationBSpline>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // SQUAD
        case chrono::industrial::TrajectoryInterpolatorOperationSpace::RotfunType::SQUAD:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationSQUAD>(rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationSQUAD>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // DEFAULT
        default:
            break;
    }
}

std::shared_ptr<ChFunctionSequence> TrajectoryInterpolatorOperationSpace::SetupSpaceFunction(
    SpacefunType spacefun_type) {
    auto spacefun = chrono_types::make_shared<ChFunctionSequence>();
    auto num_motions = m_waypoints.size() - 1;

    switch (m_rot_spacefun_type) {
        // LINEAR
        case SpacefunType::LINEAR:
            spacefun->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 1. / m_motion_time_tot),
                                  m_motion_time_tot);
            break;

        // POLY345
        case SpacefunType::POLY345:
            spacefun->InsertFunct(chrono_types::make_shared<ChFunctionPoly345>(1, m_motion_time_tot),
                                  m_motion_time_tot);
            break;

        // CONSTACC
        case SpacefunType::CONSTACC:
            // assume 1/3 - 1/3 - 1/3 law
            spacefun->InsertFunct(chrono_types::make_shared<ChFunctionConstAcc>(1, 1. / 2, CH_2_3, m_motion_time_tot),
                                  m_motion_time_tot);
            break;

        // CYCLOIDAL
        case SpacefunType::CYCLOIDAL:
            spacefun->InsertFunct(chrono_types::make_shared<ChFunctionCycloidal>(1, m_motion_time_tot),
                                  m_motion_time_tot);
            break;

        // PIECEWISE LINEAR
        case SpacefunType::PW_LINEAR:
            for (auto i = 0; i < num_motions; ++i) {
                spacefun->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, (1. / m_durations[i]) / num_motions),
                                      m_durations[i], 1, true);
            }
            break;

        // PIECEWISE POLY345
        case SpacefunType::PW_POLY345:
            for (auto i = 0; i < num_motions; ++i) {
                spacefun->InsertFunct(chrono_types::make_shared<ChFunctionPoly345>(1. / num_motions, m_durations[i]),
                                      m_durations[i], 1, true);
            }
            break;

        // PIECEWISE CONSTACC
        case SpacefunType::PW_CONSTACC:
            // assume 1/3 - 1/3 - 1/3 law
            for (auto i = 0; i < num_motions; ++i) {
                spacefun->InsertFunct(
                    chrono_types::make_shared<ChFunctionConstAcc>(1. / num_motions, 1. / 2, CH_2_3, m_durations[i]),
                    m_durations[i], 1, true);
            }
            break;

        // PIECEWISE CYCLOIDAL
        case SpacefunType::PW_CYCLOIDAL:
            for (auto i = 0; i < num_motions; ++i) {
                spacefun->InsertFunct(chrono_types::make_shared<ChFunctionCycloidal>(1. / num_motions, m_durations[i]),
                                      m_durations[i], 1, true);
            }
            break;

        // DEFAULT
        default:
            break;
    }

    return spacefun;
}

std::vector<ChVector3d> TrajectoryInterpolatorOperationSpace::GetPositions() const {
    std::vector<ChVector3d> positions(m_waypoints.size());
    for (auto i = 0; i < positions.size(); ++i) {
        positions[i] = m_waypoints[i].pos;
    }
    return positions;
}

std::vector<ChQuaterniond> TrajectoryInterpolatorOperationSpace::GetRotations() const {
    std::vector<ChQuaterniond> rots(m_waypoints.size());
    for (auto i = 0; i < rots.size(); ++i) {
        rots[i] = m_waypoints[i].rot;
    }
    return rots;
}

ChCoordsysd TrajectoryInterpolatorOperationSpace::GetInterpolation(double time) const {
    ChCoordsysd csys(m_posfun->GetPos(time), m_rotfun->GetQuat(time));
    return csys;
}

void TrajectoryInterpolatorOperationSpace::SetDurations(const std::vector<double>& durations) {
    if (durations.size() != m_waypoints.size() - 1) {
        std::cerr << "Invalid durations size" << std::endl;
        throw std::invalid_argument("Invalid durations size");
    }
    m_durations = durations;
}

void TrajectoryInterpolatorOperationSpace::AutoComputeTrajectoryDurations() {
    std::vector<double> path_lengths(m_waypoints.size() - 1);
    double path_length_tot = 0;
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        // measure traj lengths in cartesian space
        path_lengths[i] = (m_waypoints[i + 1].pos - m_waypoints[i].pos).Length();
        path_length_tot += path_lengths[i];
    }

    // Measure durations and cumulative times
    m_durations.clear();  // clear, for safety
    m_cumul_times = {0};
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        double duration = path_lengths[i] / path_length_tot * m_motion_time_tot;
        m_durations.push_back(duration);
        m_cumul_times.push_back(m_cumul_times.back() + duration);  // cumulative time
    }
}

// =============================================================================
// TrajectoryInterpolatorJointSpace
// =============================================================================
TrajectoryInterpolatorJointSpace::TrajectoryInterpolatorJointSpace(double motion_time_tot,
                                                                   const std::vector<ChVectorDynamic<>>& waypoints,
                                                                   SpacefunType spacefun,
                                                                   std::vector<double>* durations) {
    Setup(motion_time_tot, waypoints, spacefun, durations);
}

void TrajectoryInterpolatorJointSpace::Setup(double motion_time_tot,
                                             const std::vector<ChVectorDynamic<>>& waypoints,
                                             SpacefunType spacefun,
                                             std::vector<double>* durations) {
    // Set data
    m_motion_time_tot = motion_time_tot;
    m_waypoints = waypoints;
    m_spacefun_type = spacefun;
    m_num_joints = waypoints[0].size();

    // If individual motion durations are not provided, auto-compute them based on trajectory lengths
    if (durations) {
        m_durations = *durations;
        double check_motion_time_tot = 0;
        for (const auto& d : m_durations)
            check_motion_time_tot += d;

        if (check_motion_time_tot != m_motion_time_tot) {
            std::cerr << "Sum of durations is different from total motion time" << std::endl;
            throw std::invalid_argument("Sum of durations is different from total motion time");
        }
    } else {
        AutoComputeTrajectoryDurations();
    }

    // Initialize motion functions
    for (unsigned int i = 0; i < m_num_joints; ++i) {
        m_motfuns.push_back(chrono_types::make_shared<ChFunctionSequence>());
    }

    // Populate motion functions
    for (auto w = 0; w < m_waypoints.size() - 1; ++w) {
        ChVectorDynamic<> delta_waypoint = m_waypoints[w + 1] - m_waypoints[w];

        for (unsigned int j = 0; j < m_num_joints; ++j) {
            double path_dist = delta_waypoint[j];
            std::shared_ptr<ChFunction> func;
            switch (m_spacefun_type) {
                case SpacefunType::LINEAR:
                    func = chrono_types::make_shared<ChFunctionRamp>(0, path_dist / m_durations[w]);
                    break;
                case SpacefunType::POLY345:
                    func = chrono_types::make_shared<ChFunctionPoly345>(path_dist, m_durations[w]);
                    break;
                case SpacefunType::CONSTACC:
                    // NB: assume law 1/3 - 1/3 - 1/3
                    func = chrono_types::make_shared<ChFunctionConstAcc>(path_dist, CH_1_3, CH_2_3, m_durations[w]);
                    break;
                case SpacefunType::CYCLOIDAL:
                    func = chrono_types::make_shared<ChFunctionCycloidal>(path_dist, m_durations[w]);
                    break;
                default:
                    break;
            }

            m_motfuns[j]->InsertFunct(func, m_durations[w], 1, true);
        }
    }
}

ChVectorDynamic<> TrajectoryInterpolatorJointSpace::GetInterpolation(double time) const {
    ChVectorDynamic<> interp(m_num_joints);
    for (unsigned int i = 0; i < m_num_joints; ++i) {
        interp[i] = m_motfuns[i]->GetVal(time);
    }
    return interp;
}

void TrajectoryInterpolatorJointSpace::SetDurations(const std::vector<double>& durations) {
    if (durations.size() != m_waypoints.size() - 1) {
        std::cerr << "Invalid durations size" << std::endl;
        throw std::invalid_argument("Invalid durations size");
    }
    m_durations = durations;
}

void TrajectoryInterpolatorJointSpace::AutoComputeTrajectoryDurations() {
    std::vector<double> path_lengths(m_waypoints.size() - 1);
    double path_length_tot = 0;
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        // measure traj lengths in joint space
        path_lengths[i] = (m_waypoints[i + 1] - m_waypoints[i]).norm();
        path_length_tot += path_lengths[i];
    }

    // Measure durations and cumulative times
    m_durations.clear();  // clear, for safety
    m_cumul_times = {0};
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        double duration = path_lengths[i] / path_length_tot * m_motion_time_tot;
        m_durations.push_back(duration);
        m_cumul_times.push_back(m_cumul_times.back() + duration);  // cumulative time
    }
}

}  // end namespace industrial
}  // end namespace chrono
