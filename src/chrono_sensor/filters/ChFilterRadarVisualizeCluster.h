// Depreciated, will remove
#ifndef CHFILTERRADARVISUALIZECLUSTER_H
#define CHFILTERRADARVISUALIZECLUSTER_H

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"


#ifdef CHRONO_HAS_OPTIX
#include <cuda.h>
#endif

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
#ifdef CHRONO_HAS_OPTIX
    CUstream m_cuda_stream;
#endif
    std::shared_ptr<ChRadarSensor> m_radar;
};

}  // namespace sensor
}  // namespace chrono

#endif