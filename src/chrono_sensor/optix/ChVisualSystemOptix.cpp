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

#include "chrono_sensor/optix/ChVisualSystemOptix.h"

namespace chrono {
namespace sensor {

ChVisualSystemOptix::ChVisualSystemOptix(ChSystem& sys)
    : ChVisualSystem(sys), m_verbose(false), m_optix_reflections(9) {
    // save the chrono system handle
    scene = chrono_types::make_shared<ChScene>();
    m_device_list = {0};
}

ChVisualSystemOptix::~ChVisualSystemOptix() {}

ChVisualSystemOptix::Update() {
    // have all the optix engines update their sensors
    for (auto pEngine : m_engines) {
        pEngine->UpdateSensors(scene);
    }
}

ChVisualSystemOptix::SetDeviceList(std::vector<unsigned int> device_ids) {
    // set the list of devices to use
    m_device_list = device_ids;
}

std::vector<unsigned int> ChVisualSystemOptix::GetDeviceList() {
    // return the list of devices being used
    return m_device_list;
}

void ChVisualSystemOptix::BindAll() {
    for (auto eng : m_engines) {
        eng->ConstructScene();
    }
}

void ChVisualSystemOptix::SetMaxEngines(int num_groups) {
    if (num_groups > 0 && num_groups < 1000) {
        m_allowable_groups = num_groups;
    }
}

void ChVisualSystemOptix::SetRayRecursions(int rec) {
    if (rec >= 0)
        m_optix_reflections = rec;
}

void ChVisualSystemOptix::AddSensor(std::shared_ptr<ChSensor> sensor) {
    // check if sensor is already in sensor list
    if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sensor) != m_sensor_list.end()) {
        std::cerr << "WARNING: Sensor already exists in this visual system. Ignoring this addition\n";
        return;
    }

    if (auto pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(sensor)) {
        m_sensor_list.push_back(sensor);
        //******** give each render group all sensor with same update rate *************//
        bool found_engine = false;

        // add the sensor to an engine with sensor of similar update frequencies
        for (auto engine : m_engines) {
            if (!found_engine && engine->GetSensor().size() > 0 &&
                abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_engine = true;
                engine->AssignSensor(pOptixSensor);
                if (m_verbose)
                    std::cout << "Sensor added to existing engine\n";
            }
        }

        try {
            // create new engines only when we need them
            if (!found_engine) {
                if (m_engines.size() < m_allowable_groups) {
                    auto engine = chrono_types::make_shared<ChOptixEngine>(
                        m_system, m_device_list[(int)m_engines.size()], m_optix_reflections,
                        m_verbose);  // limits to 2 gpus, TODO: check if device supports cuda

                    // engine->ConstructScene();
                    engine->AssignSensor(pOptixSensor);

                    m_engines.push_back(engine);
                    if (m_verbose)
                        std::cout << "Created another OptiX engine. Now at: " << m_engines.size() << "\n";

                } else {  // if we are not allowed to create additional groups, warn the user and polute the first group
                    m_engines[0]->AssignSensor(pOptixSensor);
                    if (m_verbose)
                        std::cout << "Couldn't find suitable existing OptiX engine, so adding to first engine\n";
                }
            }
        } catch (std::exception& e) {
            std::cerr << "Failed to create a ChOptixEngine, with error:\n" << e.what() << "\n";
            exit(1);
        }
    }
}

}  // namespace sensor
}  // namespace chrono