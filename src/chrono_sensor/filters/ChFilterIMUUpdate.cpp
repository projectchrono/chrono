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
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/utils/ChGPSUtils.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

#include <chrono>

namespace chrono {
namespace sensor {

ChFilterAccelerometerUpdate::ChFilterAccelerometerUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : m_noise_model(noise_model), ChFilter("Accelerometer Updater") {}

CH_SENSOR_API void ChFilterAccelerometerUpdate::Apply() {
    // default sensor values
    ChVector<double> acc = {0, 0, 0};

    if (m_accSensor->m_keyframes.size() > 0) {
        for (auto c : m_accSensor->m_keyframes) {
            acc += c;
        }
        acc /= (float)(m_accSensor->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(acc);
    }

    // load IMU data
    m_bufferOut->Buffer[0].X = acc.x();
    m_bufferOut->Buffer[0].Y = acc.y();
    m_bufferOut->Buffer[0].Z = acc.z();

    m_bufferOut->LaunchedCount = m_accSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = (float)m_accSensor->GetParent()->GetSystem()->GetChTime();
}

CH_SENSOR_API void ChFilterAccelerometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                           std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Accelerometer update filter must be applied first in filter graph");
    }
    m_accSensor = std::dynamic_pointer_cast<ChAccelerometerSensor>(pSensor);
    if (!m_accSensor) {
        throw std::runtime_error("Accelerometer Update filter can only be used on an accelerometer\n");
    }

    m_bufferOut = chrono_types::make_shared<SensorHostAccelBuffer>();
    m_bufferOut->Buffer = std::make_unique<AccelData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_accSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

ChFilterGyroscopeUpdate::ChFilterGyroscopeUpdate(std::shared_ptr<ChNoiseModel> noise_model)
    : m_noise_model(noise_model), ChFilter("Gyroscope Updater") {}

CH_SENSOR_API void ChFilterGyroscopeUpdate::Apply() {
    // default sensor values
    ChVector<double> ang_vel = {0, 0, 0};

    if (m_gyroSensor->m_keyframes.size() > 0) {
        for (auto c : m_gyroSensor->m_keyframes) {
            ang_vel += c;
        }
        ang_vel = ang_vel / (double)(m_gyroSensor->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(ang_vel);
    }

    // load IMU data
    m_bufferOut->Buffer[0].Roll = ang_vel.x();
    m_bufferOut->Buffer[0].Pitch = ang_vel.y();
    m_bufferOut->Buffer[0].Yaw = ang_vel.z();
    m_bufferOut->LaunchedCount = m_gyroSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = (float)m_gyroSensor->GetParent()->GetSystem()->GetChTime();
}

CH_SENSOR_API void ChFilterGyroscopeUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                       std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Gyroscope update filter must be applied first in filter graph");
    }

    m_gyroSensor = std::dynamic_pointer_cast<ChGyroscopeSensor>(pSensor);
    if (!m_gyroSensor) {
        throw std::runtime_error("Gyroscope update filter can only be used on a gyroscope\n");
    }

    m_bufferOut = chrono_types::make_shared<SensorHostGyroBuffer>();
    m_bufferOut->Buffer = std::make_unique<GyroData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_gyroSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;
    bufferInOut = m_bufferOut;
}

ChFilterMagnetometerUpdate::ChFilterMagnetometerUpdate(std::shared_ptr<ChNoiseModel> noise_model,
                                                       ChVector<double> gps_reference)
    : m_noise_model(noise_model), ChFilter("Magnetometer Updater"), m_gps_reference(gps_reference) {}

CH_SENSOR_API void ChFilterMagnetometerUpdate::Apply() {
    // default sensor values
    ChVector<double> pos = {0, 0, 0};
    if (m_magSensor->m_keyframes.size() > 0) {
        for (auto c : m_magSensor->m_keyframes) {
            pos += c.GetPos();
        }
        pos = pos / (float)(m_magSensor->m_keyframes.size());
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

    // get magnetic field in sensor frame
    double H = B_abs * cos(alpha);
    ChVector<double> mag_field = {H * cos(beta), H * sin(beta), B_abs * sin(alpha)};

    double ang;
    ChVector<double> axis;
    m_magSensor->m_keyframes[0].GetRot().Q_to_AngAxis(ang, axis);
    ChVector<double> mag_field_sensor = m_magSensor->m_keyframes[0].GetRot().Rotate(mag_field);

    if (m_noise_model) {
        m_noise_model->AddNoise(mag_field_sensor);
    }

    // pack magnetometer data
    m_bufferOut->Buffer[0].X = mag_field_sensor.x();  // units of Gauss
    m_bufferOut->Buffer[0].Y = mag_field_sensor.y();  // units of Gauss
    m_bufferOut->Buffer[0].Z = mag_field_sensor.z();  // units of Gauss
    m_bufferOut->LaunchedCount = m_magSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = (float)m_magSensor->GetParent()->GetSystem()->GetChTime();
}

CH_SENSOR_API void ChFilterMagnetometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                          std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Magenetometer update filter must be applied first in filter graph");
    }

    m_magSensor = std::dynamic_pointer_cast<ChMagnetometerSensor>(pSensor);
    if (!m_magSensor) {
        throw std::runtime_error("Magnetometer update filter can only be used on a magnetometer\n");
    }

    m_bufferOut = chrono_types::make_shared<SensorHostMagnetBuffer>();
    m_bufferOut->Buffer = std::make_unique<MagnetData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_magSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono
