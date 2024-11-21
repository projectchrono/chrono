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
// Class for managing the Optix rendering system
//
// =============================================================================

#include "chrono_sensor/ChDynamicsManager.h"

#include <iomanip>
#include <iostream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChDynamicsManager::ChDynamicsManager(ChSystem* chrono_system) {
    // save the chrono system handle
    m_system = chrono_system;
}

CH_SENSOR_API ChDynamicsManager::~ChDynamicsManager() {}

CH_SENSOR_API void ChDynamicsManager::UpdateSensors() {
    for (int i = 0; i < m_sensor_list.size(); i++) {
        auto pSen = m_sensor_list[i];

        if (m_system->GetChTime() > pSen->GetNumLaunches() / pSen->GetUpdateRate() - 1e-7) {
            pSen->PushKeyFrame();
            if (m_system->GetChTime() >
                pSen->GetNumLaunches() / pSen->GetUpdateRate() + pSen->GetCollectionWindow() - 1e-7) {
                // pGPS->gps_key_frames = m_gps_collection_data[i];
                pSen->IncrementNumLaunches();
                // step through the filter list, applying each filter
                for (auto filter : pSen->GetFilterList()) {
                    filter->Apply();
                }
                // pSen->ClearKeyFrames();
            }
        }
    }
}

CH_SENSOR_API void ChDynamicsManager::AssignSensor(std::shared_ptr<ChSensor> sensor) {
    if (auto sen = std::dynamic_pointer_cast<ChDynamicSensor>(sensor)) {
        // check if sensor is already in sensor list
        if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sen) != m_sensor_list.end()) {
            std::cerr << "WARNING: This sensor already exists in manager. Ignoring this addition\n";
            return;
        }
        // add a GPS sensor
        m_sensor_list.push_back(sen);
        std::shared_ptr<SensorBuffer> buffer;
        for (auto f : sen->GetFilterList()) {
            f->Initialize(sen, buffer);
        }
        sen->LockFilterList();

    } else {
        std::cerr << "WARNING: unsupported sensor type found in the dynamic sensor manager. Ignoring...\n";
        std::cerr << "Sensor was: " << sensor->GetName() << std::endl;
    }
}

}  // namespace sensor
}  // namespace chrono
