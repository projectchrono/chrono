#ifndef CHSPEEDOMETERSENSOR_H
#define CHSPEEDOMETERSENSOR_H

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

class CH_SENSOR_API ChSpeedometerSensor : public ChDynamicSensor {
  public:
    ChSpeedometerSensor(std::shared_ptr<chrono::ChBody> parent, float update, chrono::ChFrame<double> offsetPose);
    ~ChSpeedometerSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    std::vector<ChVector<double>> m_keyframes;
    friend class ChFilterSpeedometerUpdate;
};

}  // namespace sensor
}  // namespace chrono

#endif