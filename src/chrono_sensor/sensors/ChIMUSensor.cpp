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
// Container classes for sensors that make up an IMU (accelerometer, gyroscope,
// magnetometer)
//
// =============================================================================

#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChAccelerometerSensor::ChAccelerometerSensor(std::shared_ptr<chrono::ChBody> parent,
                                                           float updateRate,
                                                           chrono::ChFrame<double> offsetPose,
                                                           std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterAccelerometerUpdate>(noise_model));
}

CH_SENSOR_API void ChAccelerometerSensor::PushKeyFrame() {
    ChVector3d tran_acc_no_offset = m_parent->PointAccelerationLocalToParent(m_offsetPose.GetPos());
    ChVector3d tran_acc_offset = -m_parent->GetSystem()->GetGravitationalAcceleration();
    tran_acc_offset = m_parent->GetRot().Rotate(tran_acc_offset);
    m_keyframes.push_back(tran_acc_no_offset + tran_acc_offset);
}
CH_SENSOR_API void ChAccelerometerSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

CH_SENSOR_API ChGyroscopeSensor::ChGyroscopeSensor(std::shared_ptr<chrono::ChBody> parent,
                                                   float updateRate,
                                                   chrono::ChFrame<double> offsetPose,
                                                   std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterGyroscopeUpdate>(noise_model));
}

CH_SENSOR_API void ChGyroscopeSensor::PushKeyFrame() {
    m_keyframes.push_back(m_parent->GetAngVelLocal());
}
CH_SENSOR_API void ChGyroscopeSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

CH_SENSOR_API ChMagnetometerSensor::ChMagnetometerSensor(std::shared_ptr<chrono::ChBody> parent,
                                                         float updateRate,
                                                         chrono::ChFrame<double> offsetPose,
                                                         std::shared_ptr<ChNoiseModel> noise_model,
                                                         ChVector3d gps_reference)
    : ChDynamicSensor(parent, updateRate, offsetPose), m_gps_reference(gps_reference) {
    m_filters.push_front(chrono_types::make_shared<ChFilterMagnetometerUpdate>(noise_model, gps_reference));
}

CH_SENSOR_API void ChMagnetometerSensor::PushKeyFrame() {
    ChFrame<double> frame = m_parent->ChFrame<double>::TransformLocalToParent(m_offsetPose);
    m_keyframes.push_back(frame);
}
CH_SENSOR_API void ChMagnetometerSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono
