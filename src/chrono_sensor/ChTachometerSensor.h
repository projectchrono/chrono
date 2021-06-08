#ifndef CHTACHOMETER_H
#define CHTACHOMETER_H

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

enum Axis {X, Y, Z};

class CH_SENSOR_API ChTachometerSensor : public ChDynamicSensor {
  public:
    ChTachometerSensor(std::shared_ptr<chrono::ChBody> parent, float update, chrono::ChFrame<double> offsetPose, Axis axis);
    ~ChTachometerSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    std::vector<ChVector<double>> m_keyframes;
    friend class ChFilterTachometerUpdate;
    Axis m_axis;
};

}  // namespace sensor
}  // namespace chrono

#endif