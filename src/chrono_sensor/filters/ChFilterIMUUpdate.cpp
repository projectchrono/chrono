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
#include "chrono_sensor/ChNoiseModel.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

#include <chrono>

namespace chrono {
namespace sensor {

// ChFilterIMUUpdate::ChFilterIMUUpdate(std::shared_ptr<ChNoiseModel> noise_model)
//     : m_noise_model(noise_model), ChFilter("IMU Updater") {}

ChFilterAccelerometerUpdate::ChFilterAccelerometerUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : m_noise_model(noise_model), ChFilter("Accelerometer Updater") {}

ChFilterGyroscopeUpdate::ChFilterGyroscopeUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : m_noise_model(noise_model), ChFilter("Gyroscope Updater") {}

ChFilterMagnetometerUpdate::ChFilterMagnetometerUpdate(std::shared_ptr<ChNoiseModel> noise_model,
                                                       ChVector<double> gps_reference)
    : m_noise_model(noise_model), ChFilter("Magnetometer Updater"), m_gps_reference(gps_reference) {}

CH_SENSOR_API void ChFilterAccelerometerUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                                      std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pAcc = std::dynamic_pointer_cast<ChAccelerometerSensor>(pSensor);
    if (!pAcc) {
        throw std::runtime_error("Accelerometer Update filter can only be used on an accelerometer\n");
    }

    // default sensor values
    ChVector<double> acc = {0, 0, 0};

    if (pAcc->m_keyframes.size() > 0) {
        for (auto c : pAcc->m_keyframes) {
            acc += c;
        }
        acc /= (float)(pAcc->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(acc);
    }

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostAccelBuffer>();
        m_buffer->Buffer = std::make_unique<AccelData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    // load IMU data
    m_buffer->Buffer[0].X = acc.x();
    m_buffer->Buffer[0].Y = acc.y();
    m_buffer->Buffer[0].Z = acc.z();

    m_buffer->LaunchedCount = pSensor->GetNumLaunches();
    m_buffer->TimeStamp = (float)pSensor->GetParent()->GetSystem()->GetChTime();

    bufferInOut = m_buffer;
}

CH_SENSOR_API void ChFilterGyroscopeUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                                  std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pGyr = std::dynamic_pointer_cast<ChGyroscopeSensor>(pSensor);
    if (!pGyr) {
        throw std::runtime_error("Accelerometer Update filter can only be used on a gyroscope\n");
    }

    // default sensor values
    ChVector<double> ang_vel = {0, 0, 0};

    if (pGyr->m_keyframes.size() > 0) {
        for (auto c : pGyr->m_keyframes) {
            ang_vel += c;
        }
        ang_vel = ang_vel / (double)(pGyr->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(ang_vel);
    }

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostGyroBuffer>();
        m_buffer->Buffer = std::make_unique<GyroData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    // load IMU data
    m_buffer->Buffer[0].Roll = ang_vel.x();
    m_buffer->Buffer[0].Pitch = ang_vel.y();
    m_buffer->Buffer[0].Yaw = ang_vel.z();

    m_buffer->LaunchedCount = pSensor->GetNumLaunches();
    m_buffer->TimeStamp = (float)pSensor->GetParent()->GetSystem()->GetChTime();

    bufferInOut = m_buffer;
}

CH_SENSOR_API void ChFilterMagnetometerUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pMag = std::dynamic_pointer_cast<ChMagnetometerSensor>(pSensor);
    if (!pMag) {
        throw std::runtime_error("Accelerometer Update filter can only be used on a magnetometer\n");
    }

    // default sensor values
    ChVector<double> pos = {0, 0, 0};
    if (pMag->m_keyframes.size() > 0) {
        for (auto c : pMag->m_keyframes) {
            pos += c;
        }
        pos = pos / (float)(pMag->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(pos);
    }

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostMagnetBuffer>();
        m_buffer->Buffer = std::make_unique<MagnetData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    Cartesian2GPS(pos, m_gps_reference);
    double phi = pos.x() * CH_C_DEG_TO_RAD;    // longitude
    double theta = pos.y() * CH_C_DEG_TO_RAD;  // latitude
    double h = pos.z();                        // altitude

    double cos_theta_m = cos(theta) * cos(theta_0) + sin(theta) * sin(theta_0) * cos(phi - phi_0);
    double sin_theta_m = sin(acos(cos_theta_m));

    double q = EARTH_RADIUS / (EARTH_RADIUS + h);
    double B_abs = abs(B_0 * (q * q * q) * sqrt(1 + 3 * cos_theta_m * cos_theta_m));

    double alpha = atan2(2 * cos_theta_m, sin_theta_m);
    double beta = sin(theta_0);
    if (cos_theta_m > beta) {
        beta = asin(sin(phi - phi_0) * (cos(theta_0) / sin_theta_m));
    } else {
        beta = asin(cos(phi - phi_0) * (cos(theta_0) / sin_theta_m));
    }
    // pack magnetometer data
    m_buffer->Buffer[0].H = B_abs * cos(alpha);                 // mag[0];
    m_buffer->Buffer[0].X = B_abs * sin(alpha);                 // mag[1];
    m_buffer->Buffer[0].Y = m_buffer->Buffer[0].H * cos(beta);  // mag[2];
    m_buffer->Buffer[0].Z = m_buffer->Buffer[0].H * sin(beta);  // mag[0 * 2];

    m_buffer->LaunchedCount = pSensor->GetNumLaunches();
    m_buffer->TimeStamp = (float)pSensor->GetParent()->GetSystem()->GetChTime();

    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
