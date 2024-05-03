//
// Created by Rainer Gericke on 16.04.24.
//

#include "Cherokee_Wheel.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables

const double Cherokee_Wheel::m_mass = 11.38;
const ChVector3d Cherokee_Wheel::m_inertia(0.5334, 0.9708, 0.5334);

const double Cherokee_Wheel::m_radius = 0.1905;
const double Cherokee_Wheel::m_width = 0.1524;

// -----------------------------------------------------------------------------

Cherokee_Wheel::Cherokee_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "jeep/Cherokee_Wheel.obj";
}

void Cherokee_Wheel::Initialize(std::shared_ptr<ChChassis> chassis,
                                std::shared_ptr<ChBody> spindle,
                                VehicleSide side,
                                double offset) {
    ChWheel::Initialize(chassis, spindle, side, offset);

    ChContactMaterialData mat_info;
    auto material = mat_info.CreateMaterial(spindle->GetSystem()->GetContactMethod());
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, m_radius, m_width);
    spindle->AddCollisionShape(ct_shape, ChFrame<>(ChVector3d(0, 0, m_offset), QuatFromAngleX(CH_PI_2)));
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
