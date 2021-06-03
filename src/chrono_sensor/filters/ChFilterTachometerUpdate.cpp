#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/ChTachometerSensor.h"

namespace chrono {
namespace sensor {

ChFilterTachometerUpdate::ChFilterTachometerUpdate() : ChFilter("Tachometer Updater") {}

CH_SENSOR_API void ChFilterTachometerUpdate::Apply() {}

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
}

}  // namespace sensor
}  // namespace chrono