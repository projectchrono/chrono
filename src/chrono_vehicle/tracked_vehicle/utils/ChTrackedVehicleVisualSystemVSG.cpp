#include "ChTrackedVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {
ChTrackedVehicleVisualSystemVSG::ChTrackedVehicleVisualSystemVSG() {}

void ChTrackedVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(vehicle);
    assert(m_tvehicle);
}

}  // namespace vehicle
}  // namespace chrono