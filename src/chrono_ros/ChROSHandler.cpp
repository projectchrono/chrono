#include "chrono_ros/ChROSHandler.h"

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

ChROSHandler::ChROSHandler(double update_rate) : m_update_rate(update_rate), m_time_elapsed_since_last_tick(0) {}

void ChROSHandler::Advance(double step) {
    // NOTE: If update_rate == 0, tick is called each time
    double frame_time = m_update_rate == 0 ? 0 : 1 / m_update_rate;

    m_time_elapsed_since_last_tick += step;
    if (m_time_elapsed_since_last_tick < frame_time)
        return;

    m_time_elapsed_since_last_tick -= frame_time;

    Tick();
}

}  // namespace ros
}  // namespace chrono