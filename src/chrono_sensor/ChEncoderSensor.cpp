#include "chrono_sensor/ChEncoderSensor.h"
#include "chrono_sensor/filters/ChFilterEncoderUpdate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChEncoderSensor::ChEncoderSensor(std::shared_ptr<chrono::ChBody> parent,
                                                       float updateRate,
                                                       chrono::ChFrame<double> offsetPose,
                                                       Axis axis)
    : ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterEncoderUpdate>());
}
CH_SENSOR_API void ChEncoderSensor::PushKeyFrame() {}

CH_SENSOR_API void ChEncoderSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono