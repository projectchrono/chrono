#include "chrono_sensor/ChTachometerSensor.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChTachometerSensor::ChTachometerSensor(std::shared_ptr<chrono::ChBody> parent,
                                                     float updateRate,
                                                     chrono::ChFrame<double> offsetPose)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterTachometerUpdate>());
}
CH_SENSOR_API void ChTachometerSensor::PushKeyFrame() {
    ChVector<double> rot_speed = m_parent->GetWvel_loc();
    m_keyframes.push_back(rot_speed);
}

CH_SENSOR_API void ChTachometerSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono