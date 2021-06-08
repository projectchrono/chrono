#include "chrono_sensor/filters/ChFilterEncoderUpdate.h"
#include "chrono_sensor/ChEncoderSensor.h"

namespace chrono {
namespace sensor {

ChFilterEncoderUpdate::ChFilterEncoderUpdate() : ChFilter("Encoder Updater") {}

CH_SENSOR_API void ChFilterEncoderUpdate::Apply() {}

CH_SENSOR_API void ChFilterEncoderUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Encoder update filter must be applied first in filter graph");
    }
    m_speedSensor = std::dynamic_pointer_cast<ChEncoderSensor>(pSensor);
    if (!m_speedSensor) {
        throw std::runtime_error("Encoder update filter can only be used on a speedometer");
    }
    m_bufferOut = chrono_types::make_shared<SensorHostEncoderBuffer>();
    m_bufferOut->Buffer = std::make_unique<EncoderData[]>(1);
}

}  // namespace sensor
}  // namespace chrono