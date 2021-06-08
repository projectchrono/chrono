#ifndef CHENCODERSENSOR_H
#define CHENCODERSENSOR_H

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

/**
 * Encoder class
 * Incremental Encoder: No unique positions, movement is relative to the initial starting point
 * 
 * Absolute Encoders: Every section has a unique code, position can be read immediately with no movement with no reference point
**/
enum Axis {X, Y, Z};

class CH_SENSOR_API ChEncoderSensor : public ChDynamicSensor {
  public:
    ChEncoderSensor(std::shared_ptr<chrono::ChBody> parent, float update, chrono::ChFrame<double> offsetPose, Axis axis);
    ~ChEncoderSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    std::vector<ChVector<double>> m_keyframes;
    friend class ChFilterSpeedometerUpdate;
    Axis axis;
};

}  // namespace sensor
}  // namespace chrono

#endif