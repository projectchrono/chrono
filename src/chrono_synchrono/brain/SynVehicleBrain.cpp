#include "chrono_synchrono/brain/SynVehicleBrain.h"

namespace chrono {
namespace synchrono {

void SynVehicleBrain::Advance(double step) {
    m_driver->Advance(step);
}

void SynVehicleBrain::Synchronize(double time) {
    m_driver->Synchronize(time);
}

}  // namespace synchrono
}  // namespace chrono
