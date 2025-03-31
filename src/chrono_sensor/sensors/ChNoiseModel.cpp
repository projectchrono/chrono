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

ChNoiseNormal::ChNoiseNormal(ChVector3d mean, ChVector3d stdev) : m_mean(mean), m_stdev(stdev), ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

void ChNoiseNormal::AddNoise(ChVector3d& data) {
    std::normal_distribution<double> dist_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_z(m_mean.z(), m_stdev.z());
    data += ChVector3d(dist_x(m_generator), dist_y(m_generator), dist_z(m_generator));
}

void ChNoiseNormal::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

ChNoiseNormalDrift::ChNoiseNormalDrift(double updateRate,
                                       ChVector3d mean,
                                       ChVector3d stdev,
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

void ChNoiseNormalDrift::AddNoise(ChVector3d& data) {
    std::normal_distribution<double> dist_a_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> dist_a_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> dist_a_z(m_mean.z(), m_stdev.z());
    ChVector3d eta_a = {dist_a_x(m_generator), dist_a_y(m_generator), dist_a_z(m_generator)};

    ChVector3d eta_b = {0, 0, 0};
    if (m_tau_drift > std::numeric_limits<double>::epsilon() && m_drift_bias > std::numeric_limits<double>::epsilon()) {
        std::normal_distribution<double> dist_b(0.0, m_drift_bias * sqrt(1 / (m_updateRate * m_tau_drift)));
        eta_b = {dist_b(m_generator), dist_b(m_generator), dist_b(m_generator)};
    }
    m_bias_prev += eta_b;
    data += eta_a + m_bias_prev;
}

void ChNoiseNormalDrift::AddNoise(ChVector3d& data, float last_ch_time, float ch_time) {
    AddNoise(data);
}

// RandomWalks class constructor with default parameters
ChNoiseRandomWalks::ChNoiseRandomWalks(float mean,
                                       float sigma,
                                       float noise_model_update_rate,
                                       ChVector3d gps_reference)
    : m_mean(mean),
      m_sigma(sigma),
      m_step_size((float)1 /
                  noise_model_update_rate),  // Note: noise_model_update_rate likely differs from GPS update rate.
      m_max_velocity(0.03),
      m_max_acceleration(0.005),
      m_gps_reference(gps_reference),
      m_last_updated_ch_time(0),
      ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
}

// RandomWalks class constructor with custom parameters
ChNoiseRandomWalks::ChNoiseRandomWalks(float mean,
                                       float sigma,
                                       float noise_model_update_rate,
                                       double max_velocity,
                                       double max_acceleration,
                                       ChVector3d gps_reference)
    : m_mean(mean),
      m_sigma(sigma),
      m_step_size((float)1 /
                  noise_model_update_rate),  // Note: noise_model_update_rate likely differs from GPS update rate.
      m_max_velocity(max_velocity),
      m_max_acceleration(max_acceleration),
      m_gps_reference(gps_reference),
      m_last_updated_ch_time(0),
      ChNoiseModel() {
    m_generator =
        std::minstd_rand((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    m_prev_error_v = ChVector3d(0, 0, 0);
    m_prev_error_p = ChVector3d(0, 0, 0);
}

void ChNoiseRandomWalks::AddNoise(ChVector3d& data, float last_ch_time, float next_ch_time) {

    double curr_time = m_last_updated_ch_time;

    // Main integration loop
    while (curr_time < next_ch_time) {
        // Determine timestep for this iteration
        double time_step = std::min(m_step_size, next_ch_time - curr_time);

        double maxD = 1.;
        // Sample either 1 or -1
        std::uniform_int_distribution<int> dist_sample(-1, 1);
        int sample = dist_sample(m_generator);

        sample = -1;
        double c_x = sample * m_prev_error_p.x() / (maxD);
        double c_y = sample * m_prev_error_p.y() / (maxD);
        double c_z = sample * m_prev_error_p.z() / (maxD);

        // Apply another weight
        c_x = m_sigma * 0.2 * c_x;
        c_y = m_sigma * 0.2 * c_y;
        c_z = m_sigma * 0.2 * c_z;

        // Generate white noise
        std::normal_distribution<double> dist_x(c_x, m_sigma);
        std::normal_distribution<double> dist_y(c_y, m_sigma);
        std::normal_distribution<double> dist_z(c_z, m_sigma);

        ChVector3d white_noise = {dist_x(m_generator), dist_y(m_generator), dist_z(m_generator)};

        // Limit all axis of white noise to m_max_acceleration
        white_noise.x() = (white_noise.x() > m_max_acceleration) ? m_max_acceleration : white_noise.x();
        white_noise.x() = (white_noise.x() < -m_max_acceleration) ? -m_max_acceleration : white_noise.x();
        white_noise.y() = (white_noise.y() > m_max_acceleration) ? m_max_acceleration : white_noise.y();
        white_noise.y() = (white_noise.y() < -m_max_acceleration) ? -m_max_acceleration : white_noise.y();
        white_noise.z() = (white_noise.z() > m_max_acceleration) ? m_max_acceleration : white_noise.z();
        white_noise.z() = (white_noise.z() < -m_max_acceleration) ? -m_max_acceleration : white_noise.z();

        // Integrate to get random walk contributions
        m_prev_error_v += white_noise * time_step;
        // If the norm of velocity is greater than max_velocity, set to max velocity
        if (m_prev_error_v.Length() > m_max_velocity) {
            m_prev_error_v = m_prev_error_v / m_prev_error_v.Length() * m_max_velocity;
        }

        m_prev_error_p += m_prev_error_v * time_step;

        // Update current time
        curr_time += time_step;
    }
    m_last_updated_ch_time = next_ch_time;
    // Apply accumulated position error to data
    data += m_prev_error_p;
}

}  // namespace sensor
}  // namespace chrono