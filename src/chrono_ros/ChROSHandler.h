#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include <string>
#include <memory>

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ChROSTopicConfiguration.h"

namespace chrono {
namespace ros {

class ChROSHandler {
  public:
    explicit ChROSHandler();

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;

    virtual void Update(double time, double step) final;

    void SetFrequency(uint64_t frequency) { m_frequency = frequency; }
    uint64_t GetFrequency() { return m_frequency; }

  protected:
    virtual void Tick(double time) = 0;

  private:
    uint64_t m_frequency;  ///< Hz

    double m_time_elapsed_since_last_tick;
};

}  // namespace ros
}  // namespace chrono

#endif