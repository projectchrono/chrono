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

TrajectoryInterpolator::TrajectoryInterpolator(const std::vector<ChCoordsysd>& waypoints,
                                               double motion_time_tot,
                                               PosfunType posfun_type,
                                               SpacefunType pos_spacefun_type,
                                               RotfunType rotfun_type,
                                               SpacefunType rot_spacefun_type,
                                               std::vector<double>* durations) {
    Setup(waypoints, motion_time_tot, posfun_type, pos_spacefun_type, rotfun_type, rot_spacefun_type, durations);
}

void TrajectoryInterpolator::Setup(const std::vector<ChCoordsysd>& waypoints,
                                   double motion_time_tot,
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
            std::cerr << "Error: sum of durations is different from total motion time" << std::endl;
            throw std::invalid_argument("Error: sum of durations is different from total motion time");
        }
    } else {
        AutoComputeTrajectoryDurations();
    }

    // Position interpolation function
    SetupPositionFunction();

    // Rotation interpolation function
    SetupRotationFunction();
}

void TrajectoryInterpolator::SetupPositionFunction() {
    // Collect waypoint positions
    std::vector<ChVector3d> pos = GetPositions();

    // Initialize common position function
    m_posfun = chrono_types::make_shared<ChFunctionPositionLine>();

    switch (m_posfun_type) {
        // LINE
        case TrajectoryInterpolator::PosfunType::LINE: {
            m_posfun->SetLine(chrono_types::make_shared<ChLineBSpline>(1, pos));
            break;
        }

        // BSPLINE2
        case TrajectoryInterpolator::PosfunType::BSPLINE2:
            m_posfun->SetLine(chrono_types::make_shared<ChLineBSpline>(2, pos));
            break;

        // BEZIER
        case TrajectoryInterpolator::PosfunType::BEZIER:
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

void TrajectoryInterpolator::SetupRotationFunction() {
    // Collect waypoint rotations
    std::vector<ChQuaterniond> rots = GetRotations();

    switch (m_rotfun_type) {
        // BSPLINE1
        case chrono::industrial::TrajectoryInterpolator::RotfunType::BSPLINE1:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationBSpline>(1, rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationBSpline>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // BSPLINE2
        case chrono::industrial::TrajectoryInterpolator::RotfunType::BSPLINE2:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationBSpline>(2, rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationBSpline>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // SQUAD
        case chrono::industrial::TrajectoryInterpolator::RotfunType::SQUAD:
            m_rotfun = chrono_types::make_shared<ChFunctionRotationSQUAD>(rots);
            m_rot_spacefun = SetupSpaceFunction(m_rot_spacefun_type);
            std::dynamic_pointer_cast<ChFunctionRotationSQUAD>(m_rotfun)->SetSpaceFunction(m_rot_spacefun);
            break;

        // DEFAULT
        default:
            break;
    }
}

std::shared_ptr<ChFunction> TrajectoryInterpolator::SetupSpaceFunction(SpacefunType spacefun_type) {
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
            spacefun->InsertFunct(chrono_types::make_shared<ChFunctionConstAcc>(1, 1. / 2, 2. / 3, m_motion_time_tot),
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
                    chrono_types::make_shared<ChFunctionConstAcc>(1. / num_motions, 1. / 2, 2. / 3, m_durations[i]),
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

std::vector<ChVector3d> TrajectoryInterpolator::GetPositions() const {
    std::vector<ChVector3d> positions(m_waypoints.size());
    for (auto i = 0; i < positions.size(); ++i) {
        positions[i] = m_waypoints[i].pos;
    }
    return positions;
}

std::vector<ChQuaterniond> TrajectoryInterpolator::GetRotations() const {
    std::vector<ChQuaterniond> rots(m_waypoints.size());
    for (auto i = 0; i < rots.size(); ++i) {
        rots[i] = m_waypoints[i].rot;
    }
    return rots;
}

ChCoordsysd TrajectoryInterpolator::GetInterpolation(double time) const {
    ChCoordsysd frame(m_posfun->GetPos(time), m_rotfun->GetQuat(time));
    return frame;
}

void TrajectoryInterpolator::SetDurations(const std::vector<double>& durations) {
    if (durations.size() != m_waypoints.size() - 1) {
        std::cerr << "Invalid durations size" << std::endl;
        throw std::invalid_argument("Invalid durations size");
    }
    m_durations = durations;
}

void TrajectoryInterpolator::AutoComputeTrajectoryDurations() {
    std::vector<double> path_lengths(m_waypoints.size() - 1);
    double path_length_tot = 0;
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        // measure traj lengths in cartesian space
        path_lengths[i] = (m_waypoints[i + 1].pos - m_waypoints[i].pos).Length();
        path_length_tot += path_lengths[i];
    }

    // Measure durations and cumulative times
    m_durations.clear();  // clear, for safety
    m_cum_times = {0};
    for (auto i = 0; i < m_waypoints.size() - 1; ++i) {
        double duration = path_lengths[i] / path_length_tot * m_motion_time_tot;
        m_durations.push_back(duration);
        m_cum_times.push_back(m_cum_times.back() + duration);  // cumulative time
    }
}

}  // end namespace industrial
}  // end namespace chrono
