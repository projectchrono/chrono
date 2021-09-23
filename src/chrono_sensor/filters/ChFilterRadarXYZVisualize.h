#ifndef CHFILTERRADARXYZVISUALIZE_H
#define CHFILTERRADARXYZVISUALIZE_H

#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/ChRadarSensor.h"

namespace chrono {
namespace sensor {

class ChSensor;
class CH_SENSOR_API ChFilterRadarXYZVisualize: public ChFilterVisualize {
  public:
    ChFilterRadarXYZVisualize(int w, int h, float zoom, std::string name = "ChFilterVisualizeRadarPC");

    virtual ~ChFilterRadarXYZVisualize();

    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_zoom;
    std::shared_ptr<ChRadarSensor> m_radar;
    std::shared_ptr<SensorDeviceRadarXYZBuffer> m_buffer_in;
    std::shared_ptr<SensorHostRadarXYZBuffer> m_host_buffer;
    CUstream m_cuda_stream;
};

}  // namespace sensor
}  // namespace chrono

#endif