#include "chrono_sensor/ChSpeedometerSensor.h"
#include "chrono_sensor/filters/ChFilterSpeedometerUpdate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChSpeedometerSensor::ChSpeedometerSensor(std::shared_ptr<chrono::ChBody> parent,
                                                       float updateRate,
                                                       chrono::ChFrame<double> offsetPose)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterSpeedometerUpdate>());
}
CH_SENSOR_API void ChSpeedometerSensor::PushKeyFrame() {}

CH_SENSOR_API void ChSpeedometerSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono