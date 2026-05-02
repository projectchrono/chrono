#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"

namespace chrono {
namespace sensor {

ChFilterTachometerUpdate::ChFilterTachometerUpdate() : ChFilter("Tachometer Updater") {}

CH_SENSOR_API void ChFilterTachometerUpdate::Apply() {
    switch (m_tachSensor->m_axis) {
        case ChTachometerSensor::Axis::X:
            m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().x() * CH_RAD_S_TO_RPM;
            break;
        case ChTachometerSensor::Axis::Y:
            m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().y() * CH_RAD_S_TO_RPM;
            break;
        case ChTachometerSensor::Axis::Z:
            m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().z() * CH_RAD_S_TO_RPM;
            break;
        default:
            throw std::runtime_error("Axis has to be X Y Z");
    }
}

CH_SENSOR_API void ChFilterTachometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                        std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Tachometer update filter must be applied first in filter graph");
    }
    m_tachSensor = std::dynamic_pointer_cast<ChTachometerSensor>(pSensor);
    if (!m_tachSensor) {
        throw std::runtime_error("Tachometer update filter can only be used on a tachometer");
    }
    m_bufferOut = chrono_types::make_shared<SensorHostTachometerBuffer>();
    m_bufferOut->Buffer = std::make_unique<TachometerData[]>(1);
    m_bufferOut->Width = m_bufferOut->Height = 1;
    m_bufferOut->TimeStamp = 0.f;

    bufferInOut = m_bufferOut;
}

}  // namespace sensor
}  // namespace chrono