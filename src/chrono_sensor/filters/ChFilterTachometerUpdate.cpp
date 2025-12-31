#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"

namespace chrono {
namespace sensor {

ChFilterTachometerUpdate::ChFilterTachometerUpdate() : ChFilter("Tachometer Updater") {}

CH_SENSOR_API void ChFilterTachometerUpdate::Apply() {
    if (m_tachSensor->m_axis == X) {
        m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().x() * 60 / 2 / CH_PI;
        //        printf("tachomter x axis\n");
    } else if (m_tachSensor->m_axis == Y) {
        m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().y() * 60 / 2 / CH_PI;
        //        printf("tachomter y axis\n");
    } else if (m_tachSensor->m_axis == Z) {
        m_bufferOut->Buffer[0].rpm = m_tachSensor->m_parent->GetAngVelLocal().z() * 60 / 2 / CH_PI;
        //        printf("tachomter z axis\n");
    } else {
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