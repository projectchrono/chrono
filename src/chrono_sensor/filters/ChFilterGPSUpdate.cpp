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
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChNoiseModel.h"
#include <chrono>

namespace chrono {
namespace sensor {

ChFilterGPSUpdate::ChFilterGPSUpdate(ChVector<double> gps_reference, std::shared_ptr<ChNoiseModel> noise_model)
    : m_ref(gps_reference), m_noise_model(noise_model), ChFilter("GPS Updater") {}

CH_SENSOR_API void ChFilterGPSUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pGPS = std::dynamic_pointer_cast<ChGPSSensor>(pSensor);

    if (!pGPS) {
        throw std::runtime_error("GPS Update filter can only be used on a GPS sensor\n");
    }

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostGPSBuffer>();
        m_buffer->Buffer = std::make_unique<GPSData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    ChVector<double> coords = {0, 0, 0};
    float ch_time = 0;
    float last_ch_time = 0;
    if (pGPS->m_keyframes.size() > 0) {
        for (auto c : pGPS->m_keyframes) {
            ch_time += std::get<0>(c);
            coords += std::get<1>(c);
            last_ch_time = std::get<0>(c);
        }
        coords = coords / (double)(pGPS->m_keyframes.size());
        ch_time = ch_time / (float)(pGPS->m_keyframes.size());
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(coords);  // 3 is length of ChVector
    }

    Cartesian2GPS(coords, m_ref);

    // load GPS data
    m_buffer->Buffer[0].Latitude = coords.y();
    m_buffer->Buffer[0].Longitude = coords.x();
    m_buffer->Buffer[0].Altitude = coords.z();
    m_buffer->Buffer[0].Time = ch_time;

    m_buffer->LaunchedCount = pSensor->GetNumLaunches();
    m_buffer->TimeStamp = last_ch_time;

    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
