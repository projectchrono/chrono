#include "chrono_sensor/filters/ChFilterSpeedometerUpdate.h"
#include "chrono_sensor/ChSpeedometerSensor.h"

namespace chrono {
namespace sensor {

ChFilterSpeedometerUpdate::ChFilterSpeedometerUpdate() : ChFilter("Speedometer Updater") {}

CH_SENSOR_API void ChFilterSpeedometerUpdate::Apply() {}

CH_SENSOR_API void ChFilterSpeedometerUpdate::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("Speedometer update filter must be applied first in filter graph");
    }
    m_speedSensor = std::dynamic_pointer_cast<ChSpeedometerSensor>(pSensor);
    if (!m_speedSensor) {
        throw std::runtime_error("Speedometer update filter can only be used on a speedometer");
    }
    m_bufferOut = chrono_types::make_shared<SensorHostSpeedometerBuffer>();
    m_bufferOut->Buffer = std::make_unique<SpeedometerData[]>(1);
}

}  // namespace sensor
}  // namespace chrono