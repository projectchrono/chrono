#ifndef CHFILTERSPEEDOMETERUPDATE_H
#define CHFILTERSPEEDOMETERUPDATE_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/ChSpeedometerSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSpeedometerSensor;

class CH_SENSOR_API ChFilterSpeedometerUpdate : public ChFilter {
  public:
    ChFilterSpeedometerUpdate();
    virtual void Apply();
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChSpeedometerSensor> m_speedSensor;
    std::shared_ptr<SensorHostSpeedometerBuffer> m_bufferOut;
};

}  // namespace sensor
}  // namespace chrono

#endif
