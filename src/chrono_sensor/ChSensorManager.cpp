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

#include "chrono_sensor/ChSensorManager.h"

#include "chrono_sensor/ChOptixSensor.h"
#include <iomanip>
#include <iostream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChSensorManager::ChSensorManager(ChSystem* chrono_system)
    : m_verbose(false), m_optix_reflections(7), m_num_keyframes(2) {
    // save the chrono system handle
    m_system = chrono_system;
    scene = chrono_types::make_shared<ChScene>();
    m_device_list = {0};
}

CH_SENSOR_API ChSensorManager::~ChSensorManager() {}

CH_SENSOR_API void ChSensorManager::SetKeyframeSize(int size) {
    m_num_keyframes = max(size, 2);
}

CH_SENSOR_API void ChSensorManager::SetKeyframeSizeFromTimeStep(float min_timestep, float max_collection_window) {
    int conservative_estimate =
        (int)(max_collection_window / min_timestep + 2);  // always allow set a couple extra frames just in case
    m_num_keyframes = max(conservative_estimate, 2);
}

CH_SENSOR_API std::shared_ptr<ChOptixEngine> ChSensorManager::GetEngine(int context_id) {
    if (context_id < m_engines.size())
        return m_engines[context_id];
    std::cerr << "ERROR: index out of render group vector bounds\n";
    return NULL;
}

CH_SENSOR_API void ChSensorManager::Update() {
    // update the scene
    // scene->PackFrame(m_system);
    //
    // have all the optix engines update their sensor
    for (auto pEngine : m_engines) {
        pEngine->UpdateSensors(scene);
    }

    // have the sensormanager update all of the non-optix sensor (IMU and GPS).
    // TODO: perhaps create a thread that takes care of this? Tradeoff since IMU should require some data from EVERY
    // step
    if (m_dynamics_manager)
        m_dynamics_manager->UpdateSensors();
}

CH_SENSOR_API void ChSensorManager::SetDeviceList(std::vector<unsigned int> device_ids) {
    // set the list of devices to use
    m_device_list = device_ids;
}
CH_SENSOR_API std::vector<unsigned int> ChSensorManager::GetDeviceList() {
    // return the list of devices being used
    return m_device_list;
}

CH_SENSOR_API void ChSensorManager::AddInstancedStaticSceneMeshes(std::vector<ChFrame<>>& frames,
                                                                  std::shared_ptr<ChTriangleMeshShape> mesh) {
    for (auto eng : m_engines) {
        eng->AddInstancedStaticSceneMeshes(frames, mesh);
    }
}

CH_SENSOR_API void ChSensorManager::ReconstructScenes() {
    for (auto eng : m_engines) {
        eng->ConstructScene();
    }
}

CH_SENSOR_API void ChSensorManager::SetMaxEngines(int num_groups) {
    if (num_groups > 0 && num_groups < 1000) {
        m_allowable_groups = num_groups;
    }
}

CH_SENSOR_API void ChSensorManager::SetRayRecursions(int rec) {
    if (rec >= 0)
        m_optix_reflections = rec;
}

CH_SENSOR_API void ChSensorManager::AddSensor(std::shared_ptr<ChSensor> sensor) {
    // check if sensor is already in sensor list
    if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sensor) != m_sensor_list.end()) {
        std::cerr << "WARNING: Sensor already exists in manager. Ignoring this addition\n";
        return;
    }
    m_sensor_list.push_back(sensor);

    if (auto pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(sensor)) {
        m_render_sensor.push_back(sensor);
        //******** give each render group all sensor with same update rate *************//
        bool found_group = false;

        // add the sensor to an engine with sensor of similar update frequencies
        for (auto engine : m_engines) {
            if (!found_group && engine->GetSensor().size() > 0 &&
                abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_group = true;
                engine->AssignSensor(pOptixSensor);
                if (m_verbose)
                    std::cout << "Sensor added to existing engine\n";
            }
        }

        // create new engines only when we need them
        if (!found_group) {
            if (m_engines.size() < m_allowable_groups) {
                auto engine = chrono_types::make_shared<ChOptixEngine>(
                    m_system, m_device_list[(int)m_engines.size()], m_optix_reflections, m_verbose,
                    m_num_keyframes);  // limits to 2 gpus, TODO: check if device supports cuda

                engine->ConstructScene();
                engine->AssignSensor(pOptixSensor);

                m_engines.push_back(engine);
                if (m_verbose)
                    std::cout << "Created another OptiX engine. Now at: " << m_engines.size() << "\n";

            } else {  // if we are not allowed to create additional groups, warn the user and polute the first group
                // std::cout << "No more allowable groups, consider allowing more groups if performace would
                // increase\n";
                m_engines[0]->AssignSensor(pOptixSensor);
                if (m_verbose)
                    std::cout << "Couldn't find suitable existing OptiX engine, so adding to first engine\n";
            }
        }
    } else {
        if (!m_dynamics_manager) {
            m_dynamics_manager = chrono_types::make_shared<ChDynamicsManager>(m_system);
        }

        // add pure dynamic sensor to dynamic manager
        m_dynamics_manager->AssignSensor(sensor);
    }
}

}  // namespace sensor
}  // namespace chrono
