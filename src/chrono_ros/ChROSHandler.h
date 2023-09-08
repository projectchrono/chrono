#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include <string>
#include <memory>

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

class ChROSHandler {
  public:
    explicit ChROSHandler(uint64_t frequency);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;

    virtual void Update(double time, double step) final;

    uint64_t GetFrequency() const { return m_frequency; }

  protected:
    virtual void Tick(double time) = 0;

  private:
    const uint64_t m_frequency;  ///< Hz

    double m_time_elapsed_since_last_tick;
};

}  // namespace ros
}  // namespace chrono

#endif