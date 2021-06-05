#ifndef CHFILTERRADARVISUALIZEDETECTION_H
#define CHFILTERRADARVISUALIZEDETECTION_H

#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/ChRadarSensor.h"

namespace chrono {
namespace sensor {

class ChSensor;
class CH_SENSOR_API ChFilterRadarVisualizeDetection : public ChFilterVisualize {
  public:
    ChFilterRadarVisualizeDetection(int w, int h, float zoom, std::string name = "ChFilterVisualizeRadarPC");

    virtual ~ChFilterRadarVisualizeDetection();

    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_zoom;
    std::shared_ptr<ChRadarSensor> m_radar;
    std::shared_ptr<SensorDeviceRadarBuffer> m_buffer_in;
    CUstream m_cuda_stream;
    std::shared_ptr<ChRadarSensor> m_radar;
};

}  // namespace sensor
}  // namespace chrono

#endif