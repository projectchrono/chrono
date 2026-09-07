// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================

#include "chrono_sensor/metal/ChMetalRTEngine.h"

#include <algorithm>
#include <iostream>
#include <vector>

namespace chrono {
namespace sensor {

ChMetalRTEngine::ChMetalRTEngine(ChSystem* sys, int device_id, int max_scene_reflections, bool verbose, bool debug)
    : m_system(sys), m_device_id(static_cast<uint32_t>(std::max(0, device_id))), m_recursions(max_scene_reflections), m_verbose(verbose), m_debug(debug) {
    ChMetalRTDeviceConfig config;
    config.device_index = m_device_id;
    config.verbose = verbose;

    try {
        m_device = chrono_types::make_shared<ChMetalRTDevice>(config);
    } catch (const std::exception& e) {
        m_device.reset();
        if (m_verbose)
            std::cerr << "Chrono::Sensor Metal RT GPU unavailable: " << e.what() << std::endl;
    }

    m_scene = chrono_types::make_shared<ChMetalRTScene>();
}

ChMetalRTEngine::~ChMetalRTEngine() {}

void ChMetalRTEngine::AssignSensor(std::shared_ptr<ChMetalSensor> sensor) {
    if (std::find(m_assigned_sensors.begin(), m_assigned_sensors.end(), sensor) != m_assigned_sensors.end()) {
        std::cerr << "WARNING: This Metal sensor already exists in manager. Ignoring this addition\n";
        return;
    }

    auto renderer = chrono_types::make_shared<ChFilterMetalRTRender>(m_device, m_scene);
    renderer->SetRayRecursions(m_recursions);
    m_assigned_sensors.push_back(sensor);
    m_assigned_renderers.push_back(renderer);

    sensor->PushFilterFront(renderer);
    sensor->LockFilterList();

    std::shared_ptr<SensorBuffer> buffer;
    for (auto& filter : sensor->GetFilterList())
        filter->Initialize(sensor, buffer);
}

void ChMetalRTEngine::ConstructScene() {
    if (m_scene)
        m_scene->SyncFromSystem(m_system);
}

void ChMetalRTEngine::UpdateSensors(std::shared_ptr<ChMetalRTScene> scene) {
    const float time = static_cast<float>(m_system ? m_system->GetChTime() : 0.0);

    std::vector<size_t> due_sensors;
    due_sensors.reserve(m_assigned_sensors.size());
    for (size_t i = 0; i < m_assigned_sensors.size(); ++i) {
        const auto& sensor = m_assigned_sensors[i];
        if (!sensor)
            continue;
        const double launch_time = sensor->GetNumLaunches() / sensor->GetUpdateRate() + sensor->GetCollectionWindow();
        if (time > launch_time - 1e-7)
            due_sensors.push_back(i);
    }

    if (due_sensors.empty())
        return;

    if (scene)
        m_scene = scene;

    if (m_scene)
        m_scene->SyncFromSystem(m_system);

    for (auto& renderer : m_assigned_renderers) {
        if (renderer)
            renderer->SetScene(m_scene);
    }

    for (const size_t i : due_sensors) {
        auto& sensor = m_assigned_sensors[i];
        if (!sensor)
            continue;

        sensor->IncrementNumLaunches();
        m_assigned_renderers[i]->m_time_stamp = time;
        for (auto& filter : sensor->GetFilterList())
            filter->Apply();
    }
}

}  // namespace sensor
}  // namespace chrono
