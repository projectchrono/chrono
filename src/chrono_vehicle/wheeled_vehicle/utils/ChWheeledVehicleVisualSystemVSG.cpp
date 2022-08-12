#include "ChWheeledVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

ChWheeledVehicleVisualSystemVSG::ChWheeledVehicleVisualSystemVSG() {}

void ChWheeledVehicleVisualSystemVSG::AttachVehicle(ChVehicle *vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(m_vehicle);
    assert(m_wvehicle);
}

}  // end namespace vehicle
}  // end namespace chrono
