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
    // update the GPS sensors (sometimes perform the update, but always pack the keyframe)
    for (int i = 0; i < m_gps_list.size(); i++) {
        auto pGPS = m_gps_list[i];

        if (m_system->GetChTime() > pGPS->GetNumLaunches() / pGPS->GetUpdateRate() - 1e-7) {
            ChVector<double> pos_data = pGPS->GetParent()->TransformPointLocalToParent(pGPS->GetOffsetPose().GetPos());
            m_gps_collection_data[i].push_back(
                std::make_tuple((float)pGPS->GetParent()->GetSystem()->GetChTime(), pos_data));

            if (m_system->GetChTime() >
                pGPS->GetNumLaunches() / pGPS->GetUpdateRate() + pGPS->GetCollectionWindow() - 1e-7) {
                pGPS->gps_key_frames = m_gps_collection_data[i];
                std::shared_ptr<SensorBuffer> buffer;
                pGPS->IncrementNumLaunches();
                // step through the filter list, applying each filter
                for (auto filter : pGPS->GetFilterList()) {
                    filter->Apply(pGPS, buffer);
                }

                // clear the keyframes for this gps
                m_gps_collection_data[i].clear();
            }
        }
    }

    // update the IMU sensors (sometimes perform the update, but always pack the keyframe)
    for (int i = 0; i < m_imu_list.size(); i++) {
        auto pIMU = m_imu_list[i];

        if (m_system->GetChTime() > pIMU->GetNumLaunches() / pIMU->GetUpdateRate() - 1e-7) {
            ChVector<float> tran_acc_no_offset =
                pIMU->GetParent()->PointAccelerationLocalToParent(pIMU->GetOffsetPose().GetPos());
            ChVector<float> tran_acc_offset = -pIMU->GetParent()->GetSystem()->Get_G_acc();
            tran_acc_offset = pIMU->GetParent()->GetRot().Rotate(tran_acc_offset);

            ChVector<float> tran_acc = tran_acc_no_offset + tran_acc_offset;

            ChVector<float> ang_vel = pIMU->GetParent()->GetWvel_loc();
            m_imu_collection_data[i].push_back(std::make_tuple(ang_vel, tran_acc));

            if (m_system->GetChTime() >
                pIMU->GetNumLaunches() / pIMU->GetUpdateRate() + pIMU->GetCollectionWindow() - 1e-7) {
                pIMU->imu_key_frames = m_imu_collection_data[i];
                std::shared_ptr<SensorBuffer> buffer;
                pIMU->IncrementNumLaunches();
                // step through the filter list, applying each filter
                for (auto filter : pIMU->GetFilterList()) {
                    filter->Apply(pIMU, buffer);
                }

                // clear the keyframes for this gps
                m_imu_collection_data[i].clear();
            }
        }
    }
}

CH_SENSOR_API void ChDynamicsManager::AssignSensor(std::shared_ptr<ChSensor> sensor) {
    if (auto gps = std::dynamic_pointer_cast<ChGPSSensor>(sensor)) {
        // check if sensor is already in sensor list
        if (std::find(m_gps_list.begin(), m_gps_list.end(), gps) != m_gps_list.end()) {
            std::cerr << "WARNING: This GPS Sensor already exists in manager. Ignoring this addition\n";
            return;
        }
        // add a GPS sensor
        m_gps_list.push_back(gps);
        for (auto f : gps->GetFilterList()) {
            f->Initialize(gps);
        }
        gps->LockFilterList();
        m_gps_collection_data.push_back(std::vector<std::tuple<float, ChVector<double>>>());
    } else if (auto imu = std::dynamic_pointer_cast<ChIMUSensor>(sensor)) {
        // check if sensor is already in sensor list
        if (std::find(m_imu_list.begin(), m_imu_list.end(), imu) != m_imu_list.end()) {
            std::cerr << "WARNING: This IMU Sensor already exists in manager. Ignoring this addition\n";
            return;
        }
        // add an IMU sensor
        m_imu_list.push_back(imu);
        for (auto f : imu->GetFilterList()) {
            f->Initialize(imu);
        }
        imu->LockFilterList();
        m_imu_collection_data.push_back(std::vector<std::tuple<ChVector<float>, ChVector<float>>>());
        // m_imu_collection_data.push_back(std::vector<ChVector>());
    } else {
        std::cerr << "WARNING: unsupported sensor type found in the dynamic sensor manager. Ignoring...\n";
    }
}

}  // namespace sensor
}  // namespace chrono
