// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Utility class which provides actuations for a set of motors, based on data
// files.
//
// =============================================================================

#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "chrono_parsers/ChRobotActuation.h"

namespace chrono {
namespace parsers {

const std::string ChRobotActuation::m_phase_names[] = {"POSE", "HOLD", "START", "CYCLE", "STOP"};

ChRobotActuation::ChRobotActuation(int num_motors,
                                   const std::string& filename_start,
                                   const std::string& filename_cycle,
                                   const std::string& filename_stop,
                                   bool repeat)
    : m_num_motors(num_motors),
      m_repeat(repeat),
      m_verbose(false),
      m_time_pose(1),
      m_time_hold(1),
      m_offset(0),
      m_phase(POSE),
      m_callback(nullptr) {
    // Open the cycle file (required)
    assert(!filename_cycle.empty());
    m_ifs_cycle.open(filename_cycle.c_str());

    // If a start data file is provided, open it and set as current stream.
    // Otherwise, set the cycle file as current stream
    if (!filename_start.empty()) {
        m_ifs_start.open(filename_start.c_str());
        m_ifs = &m_ifs_start;
    } else {
        m_ifs = &m_ifs_cycle;
    }

    // If a stop file is provided, open it
    if (!filename_stop.empty()) {
        m_ifs_stop.open(filename_stop.c_str());
    }

    // Resize actuation buffers
    m_actuations_1.resize(m_num_motors);
    m_actuations_2.resize(m_num_motors);
    m_actuations.resize(m_num_motors);

    // Read first two lines in current stream
    LoadDataLine(m_time_1, m_actuations_1);
    LoadDataLine(m_time_2, m_actuations_2);
}

ChRobotActuation ::~ChRobotActuation() {}

void ChRobotActuation::SetTimeOffsets(double time_pose, double time_hold) {
    m_time_pose = time_pose;
    m_time_hold = time_hold;
    m_offset = time_pose + time_hold;
}

void ChRobotActuation::LoadDataLine(double& time, Actuation& buffer) {
    assert(buffer.size() == m_num_motors);

    *m_ifs >> time;
    for (int i = 0; i < m_num_motors; i++)
        *m_ifs >> buffer[i];
}

// -----------------------------------------------------------------------------

class ax {
  public:
    ax(double a) : m_a(a) {}
    double operator()(const double& x) { return m_a * x; }

  private:
    double m_a;
};

class axpby {
  public:
    axpby(double a, double b) : m_a(a), m_b(b) {}
    double operator()(const double& x, const double& y) { return m_a * x + m_b * y; }

  private:
    double m_a;
    double m_b;
};

void ChRobotActuation::Update(double time) {
    // In the POSE phase, use a logistic function to reach first data entry
    if (m_phase == POSE) {
        double tau = 20 * (time / m_time_pose) - 10;
        ax op(std::exp(tau) / (1 + std::exp(tau)));
        std::transform(m_actuations_1.begin(), m_actuations_1.end(), m_actuations.begin(), op);
        if (time >= m_time_pose) {
            m_phase = HOLD;
            if (m_verbose)
                std::cout << "time = " << time << " | Switch to phase: " << GetCurrentPhase() << std::endl;
            if (m_callback)
                m_callback->OnPhaseChange(POSE, m_phase);
        }
        return;
    }

    // In the HOLD phase, always use the first data entry
    if (m_phase == HOLD) {
        m_actuations = m_actuations_1;
        if (time >= m_offset) {
            m_phase = (m_ifs_start.is_open()) ? START : CYCLE;
            if (m_verbose)
                std::cout << "time = " << time << " | Switch to phase: " << GetCurrentPhase() << std::endl;
            if (m_callback)
                m_callback->OnPhaseChange(HOLD, m_phase);
        }
        return;
    }

    // Offset time
    double t = time - m_offset;

    switch (m_phase) {
        case START:
            while (t > m_time_2) {
                m_time_1 = m_time_2;
                m_actuations_1 = m_actuations_2;
                if (!m_ifs->eof()) {
                    LoadDataLine(m_time_2, m_actuations_2);
                } else {
                    m_phase = CYCLE;
                    m_ifs = &m_ifs_cycle;
                    LoadDataLine(m_time_1, m_actuations_1);
                    LoadDataLine(m_time_2, m_actuations_2);
                    m_offset = time;
                    if (m_verbose)
                        std::cout << "time = " << time << " | Switch to phase: " << GetCurrentPhase() << std::endl;
                    if (m_callback)
                        m_callback->OnPhaseChange(START, CYCLE);
                    return;
                }
            }

            break;

        case CYCLE:
            while (t > m_time_2) {
                m_time_1 = m_time_2;
                m_actuations_1 = m_actuations_2;
                if (m_ifs->eof()) {
                    if (m_repeat) {
                        m_ifs->clear();
                        m_ifs->seekg(0);
                        LoadDataLine(m_time_1, m_actuations_1);
                        LoadDataLine(m_time_2, m_actuations_2);
                        m_offset = time;
                        if (m_verbose)
                            std::cout << "time = " << time << " | New cycle" << std::endl;
                        if (m_callback)
                            m_callback->OnPhaseChange(CYCLE, CYCLE);
                    }
                    return;
                }
                LoadDataLine(m_time_2, m_actuations_2);
            }

            break;

        case STOP:
            //// TODO
            break;

        default:
            break;
    }

    // Interpolate  v = alpha_1 * v_1 + alpha_2 * v_2
    axpby op((t - m_time_2) / (m_time_1 - m_time_2),   // alpha_1
             (t - m_time_1) / (m_time_2 - m_time_1));  // alpha_2
    std::transform(m_actuations_1.begin(), m_actuations_1.end(), m_actuations_2.begin(), m_actuations.begin(), op);
}

}  // namespace parsers
}  // namespace chrono
