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

#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

#include <chrono>

namespace chrono {
namespace sensor {

ChFilterGPSUpdate::ChFilterGPSUpdate(ChVector3d gps_reference, std::shared_ptr<ChNoiseModel> noise_model)
    : m_ref(gps_reference), m_noise_model(noise_model), ChFilter("GPS Updater") {}

CH_SENSOR_API void ChFilterGPSUpdate::Apply() {
    auto curr_index = m_GPSSensor->m_keyframes.size() - 1;
    float ch_time = std::get<0>(m_GPSSensor->m_keyframes[curr_index]);
    float last_ch_time = (curr_index == 0 ? 0.0 : std::get<0>(m_GPSSensor->m_keyframes[curr_index - 1]));
    ChVector3d coords = std::get<1>(m_GPSSensor->m_keyframes[curr_index]);
    // std::cout << "GPS coords: " << coords.x() << " " << coords.y() << " " << coords.z() << std::endl;
    if (m_noise_model) {
        m_noise_model->AddNoise(coords, last_ch_time, ch_time);  // 3 is length of ChVector
    }

    Cartesian2GPS(coords, m_ref);

    // load GPS data
    m_bufferOut->Buffer[0].Latitude = coords.y();
    m_bufferOut->Buffer[0].Longitude = coords.x();
    m_bufferOut->Buffer[0].Altitude = coords.z();
    m_bufferOut->Buffer[0].Time = ch_time;
    m_bufferOut->LaunchedCount = m_GPSSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = last_ch_time;
}

CH_SENSOR_API void ChFilterGPSUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("GPS update filter must be first in filter graph");
    }

    m_GPSSensor = std::dynamic_pointer_cast<ChGPSSensor>(pSensor);
    if (!m_GPSSensor) {
        throw std::runtime_error("GPS Update filter can only be used on a GPS sensor\n");
    }

    m_bufferOut = chrono_types::make_shared<SensorHostGPSBuffer>();
    m_bufferOut->Buffer = std::make_unique<GPSData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->LaunchedCount = m_GPSSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = 0;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono