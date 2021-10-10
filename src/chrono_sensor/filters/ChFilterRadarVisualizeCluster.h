// Depreciated, will remove 
#ifndef CHFILTERRADARVISUALIZECLUSTER_H
#define CHFILTERRADARVISUALIZECLUSTER_H

#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"

namespace chrono {
namespace sensor {

class ChSensor;
class CH_SENSOR_API ChFilterRadarVisualizeCluster : public ChFilterVisualize {
  public:
    ChFilterRadarVisualizeCluster(int w, int h, float zoom, std::string name = "ChFilterVisualizeRadarPC");

    virtual ~ChFilterRadarVisualizeCluster();

    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_zoom;
    std::shared_ptr<SensorHostRadarXYZBuffer> m_buffer_in;  ///< input buffer
    CUstream m_cuda_stream;
    std::shared_ptr<ChRadarSensor> m_radar;
};

}  // namespace sensor
}  // namespace chrono

#endif