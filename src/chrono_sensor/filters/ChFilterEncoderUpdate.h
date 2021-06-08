#ifndef CHFILTERSPEEDOMETERUPDATE_H
#define CHFILTERSPEEDOMETERUPDATE_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/ChEncoderSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChEncoderSensor;

class CH_SENSOR_API ChFilterEncoderUpdate : public ChFilter {
  public:
    ChFilterEncoderUpdate();
    virtual void Apply();
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChEncoderSensor> m_speedSensor;
    std::shared_ptr<SensorHostEncoderBuffer> m_bufferOut;
};

}  // namespace sensor
}  // namespace chrono

#endif
