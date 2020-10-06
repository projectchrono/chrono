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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/ChIMUSensor.h"

#include <chrono>

namespace chrono {
namespace sensor {

ChFilterIMUUpdate::ChFilterIMUUpdate(std::shared_ptr<ChIMUNoiseModel> noise_model)
    : m_noise_model(noise_model), ChFilter("IMU Updater") {}

CH_SENSOR_API void ChFilterIMUUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pIMU = std::dynamic_pointer_cast<ChIMUSensor>(pSensor);
    if (!pIMU) {
        throw std::runtime_error("IMU Update filter can only be used on an IMU sensor\n");
    }

    // TODO: This sensor is not currently threadsafe, so we need to have the sensormanager do the computation directly
    // for now. Possible solution is to have the sensormanager save relavant scene information in threadsafe struct so
    // each worker thread can access as it wishes

    // TODO: have the IMU use data from ALL chrono timesteps and average to get the "ground truth" data

    // TODO: change to account for offset pose

    // default sensor values
    ChVector<float> ang_vel = {0, 0, 0};
    ChVector<float> tran_acc = {0, 0, 0};

    if (pIMU->imu_key_frames.size() > 0) {
        for (auto c : pIMU->imu_key_frames) {
            ang_vel += std::get<0>(c);
            tran_acc += std::get<1>(c);
        }
        ang_vel = ang_vel / pIMU->imu_key_frames.size();
        tran_acc = tran_acc / pIMU->imu_key_frames.size();
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(ang_vel, tran_acc);
    }

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostIMUBuffer>();
        m_buffer->Buffer = std::make_unique<IMUData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    // load IMU data
    m_buffer->Buffer[0].Accel[0] = tran_acc.x();
    m_buffer->Buffer[0].Accel[1] = tran_acc.y();
    m_buffer->Buffer[0].Accel[2] = tran_acc.z();
    m_buffer->Buffer[0].Roll = ang_vel.x();
    m_buffer->Buffer[0].Pitch = ang_vel.y();
    m_buffer->Buffer[0].Yaw = ang_vel.z();

    m_buffer->LaunchedCount = pSensor->GetNumLaunches();
    m_buffer->TimeStamp = pSensor->GetParent()->GetSystem()->GetChTime();

    bufferInOut = m_buffer;
}

CH_SENSOR_API ChIMUNoiseNormalDrift::ChIMUNoiseNormalDrift(float updateRate,
                                                           float g_mean,
                                                           float g_stdev,
                                                           float g_bias_drift,
                                                           float g_tau_drift,
                                                           float a_mean,
                                                           float a_stdev,
                                                           float a_bias_drift,
                                                           float a_tau_drift)
    : m_updateRate(updateRate),
      m_g_mean(g_mean),
      m_g_stdev(g_stdev),
      m_g_bias_drift(g_bias_drift),
      m_g_tau_drift(g_tau_drift),
      m_a_mean(a_mean),
      m_a_stdev(a_stdev),
      m_a_bias_drift(a_bias_drift),
      m_a_tau_drift(a_tau_drift),
      ChIMUNoiseModel() {
    m_generator = std::minstd_rand(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

void ChIMUNoiseNormalDrift::AddNoise(chrono::ChVector<float>& gyro, chrono::ChVector<float>& acc) {
    // add noise to gyroscope
    std::normal_distribution<float> gyro_dist_a(m_g_mean, m_g_stdev);
    ChVector<float> gyro_eta_a = {gyro_dist_a(m_generator), gyro_dist_a(m_generator), gyro_dist_a(m_generator)};

    ChVector<float> gyro_eta_b = {0, 0, 0};
    if (m_g_tau_drift > 1e-6 && m_g_bias_drift > 1e-6) {
        std::normal_distribution<float> gyro_dist_b(0.0, m_g_bias_drift * sqrt(1 / (m_updateRate * m_g_tau_drift)));
        gyro_eta_b = {gyro_dist_b(m_generator), gyro_dist_b(m_generator), gyro_dist_b(m_generator)};
    }
    m_gyro_bias_prev = m_gyro_bias_prev + gyro_eta_b;
    gyro = gyro + gyro_eta_a + m_gyro_bias_prev;

    // add noise to accelerometer
    std::normal_distribution<float> acc_dist_a(m_a_mean, m_a_stdev);
    ChVector<float> acc_eta_a = {acc_dist_a(m_generator), acc_dist_a(m_generator), acc_dist_a(m_generator)};

    ChVector<float> acc_eta_b = {0, 0, 0};
    if (m_a_tau_drift > 1e-6 && m_a_bias_drift > 1e-6) {
        std::normal_distribution<float> acc_dist_b(0.0, m_a_bias_drift * sqrt(1 / (m_updateRate * m_a_tau_drift)));
        acc_eta_b = {acc_dist_b(m_generator), acc_dist_b(m_generator), acc_dist_b(m_generator)};
    }
    m_acc_bias_prev = m_acc_bias_prev + acc_eta_b;
    acc = acc + acc_eta_a + m_acc_bias_prev;
}

}  // namespace sensor
}  // namespace chrono
