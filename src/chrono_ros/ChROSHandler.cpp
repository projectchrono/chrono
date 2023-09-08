#include "chrono_ros/ChROSHandler.h"

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

ChROSHandler::ChROSHandler(uint64_t m_frequency) : m_frequency(m_frequency), m_time_elapsed_since_last_tick(0) {}

void ChROSHandler::Update(double time, double step) {
    // NOTE: If update_rate == 0, tick is called each time
    double frame_time = m_frequency == 0 ? 0 : 1 / m_frequency;

    m_time_elapsed_since_last_tick += step;
    if (m_time_elapsed_since_last_tick < frame_time)
        return;

    m_time_elapsed_since_last_tick -= frame_time;

    Tick(time);
}

}  // namespace ros
}  // namespace chrono