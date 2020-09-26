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
#include <chrono>

namespace chrono {
namespace sensor {

ChFilterGPSUpdate::ChFilterGPSUpdate(ChVector<double> gps_reference, std::shared_ptr<ChGPSNoiseModel> noise_model)
    : m_ref(gps_reference), m_noise_model(noise_model), ChFilter("GPS Updater") {}

CH_SENSOR_API void ChFilterGPSUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    auto pGPS = std::dynamic_pointer_cast<ChGPSSensor>(pSensor);

    if (!pGPS) {
        throw std::runtime_error("GPS Update filter can only be used on a GPS sensor\n");
    }

    // TODO: This sensor is not currently threadsafe, so we need to have the sensormanager do the computation directly
    // for now. Possible solution is to have the sensormanager save relavant scene information in threadsafe struct so
    // each worker thread can access as it wishes

    // TODO: have the GPS use data from ALL chrono timesteps and average to get the "ground truth" data

    // TODO: change to account for offset pose

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorHostGPSBuffer>();
        m_buffer->Buffer = std::make_unique<GPSData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    ChVector<double> coords = {0, 0, 0};
    float ch_time = 0;
    float last_ch_time = 0;
    if (pGPS->gps_key_frames.size() > 0) {
        for (auto c : pGPS->gps_key_frames) {
            ch_time += std::get<0>(c);
            coords += std::get<1>(c);
            last_ch_time = std::get<0>(c);
        }
        coords = coords / pGPS->gps_key_frames.size();
        ch_time = ch_time / pGPS->gps_key_frames.size();
    }

    if (m_noise_model) {
        m_noise_model->AddNoise(coords);
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

CH_SENSOR_API ChGPSNoiseNormal::ChGPSNoiseNormal(ChVector<float> mean, ChVector<float> stdev)
    : m_mean(mean), m_stdev(stdev), ChGPSNoiseModel() {
    m_generator = std::minstd_rand(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}
CH_SENSOR_API ChGPSNoiseNormal::~ChGPSNoiseNormal() {}
CH_SENSOR_API void ChGPSNoiseNormal::AddNoise(ChVector<double>& coords) {
    std::normal_distribution<double> gps_dist_x(m_mean.x(), m_stdev.x());
    std::normal_distribution<double> gps_dist_y(m_mean.y(), m_stdev.y());
    std::normal_distribution<double> gps_dist_z(m_mean.z(), m_stdev.z());

    coords = coords + ChVector<double>(gps_dist_x(m_generator), gps_dist_y(m_generator), gps_dist_z(m_generator));
}

}  // namespace sensor
}  // namespace chrono
