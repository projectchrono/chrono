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
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include <chrono>

namespace chrono {
namespace sensor {

ChNoiseNormal::ChNoiseNormal(ChVector<double> mean, ChVector<double> stdev)
    : m_mean(mean), m_stdev(stdev), ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

void ChNoiseNormal::AddNoise(ChVector<double>& data) {
    std::normal_distribution<double> dist_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_z(m_mean.z(), m_stdev.z());
    data += ChVector<double>(dist_x(m_generator), dist_y(m_generator), dist_z(m_generator));
}

ChNoiseNormalDrift::ChNoiseNormalDrift(double updateRate,
                                       ChVector<double> mean,
                                       ChVector<double> stdev,
                                       double drift_bias,
                                       double tau_drift)
    : m_updateRate(updateRate),
      m_mean(mean),
      m_stdev(stdev),
      m_drift_bias(drift_bias),
      m_tau_drift(tau_drift),
      ChNoiseModel() {
    m_bias_prev = {0, 0, 0};
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

void ChNoiseNormalDrift::AddNoise(ChVector<double>& data) {
    std::normal_distribution<double> dist_a_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_a_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_a_z(m_mean.z(), m_stdev.z());
    ChVector<double> eta_a = {dist_a_x(m_generator), dist_a_y(m_generator), dist_a_z(m_generator)};

    ChVector<double> eta_b = {0, 0, 0};
    if (m_tau_drift > std::numeric_limits<double>::epsilon() && m_drift_bias > std::numeric_limits<double>::epsilon()) {
        std::normal_distribution<double> dist_b(0.0, m_drift_bias * sqrt(1 / (m_updateRate * m_tau_drift)));
        eta_b = {dist_b(m_generator), dist_b(m_generator), dist_b(m_generator)};
    }
    m_bias_prev += eta_b;
    data += eta_a + m_bias_prev;
}

}  // namespace sensor
}  // namespace chrono
