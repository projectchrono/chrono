#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include <string>
#include <memory>

namespace chrono {
namespace ros {

class ChROSInterface;

class ChROSHandler {
  public:
    ChROSHandler(double update_rate);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;
    virtual void Advance(double step) final;

    virtual const std::string GetNamespace() const = 0;

  protected:
    virtual void Tick() = 0;

  private:
    double m_update_rate;
    double m_time_elapsed_since_last_tick;
};

}  // namespace ros
}  // namespace chrono

#endif