#ifndef CHFILTERTACHOMETERUPDATE_H
#define CHFILTERTACHOMETERUPDATE_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChTachometerSensor;

class CH_SENSOR_API ChFilterTachometerUpdate : public ChFilter {
  public:
    ChFilterTachometerUpdate();
    virtual void Apply();
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChTachometerSensor> m_tachSensor;
    std::shared_ptr<SensorHostTachometerBuffer> m_bufferOut;
};

}  // namespace sensor
}  // namespace chrono

#endif
