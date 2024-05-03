//
// Created by Rainer Gericke on 16.04.24.
//

#include "Cherokee_Driveline4WD.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Cherokee_Driveline4WD::m_central_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_front_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_rear_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_driveshaft_inertia = 0.5;
const double Cherokee_Driveline4WD::m_frontshaft_inertia = 0.5;
const double Cherokee_Driveline4WD::m_rearshaft_inertia = 0.5;

const double Cherokee_Driveline4WD::m_front_conicalgear_ratio = 0.2;
const double Cherokee_Driveline4WD::m_rear_conicalgear_ratio = 0.2;

const double Cherokee_Driveline4WD::m_axle_differential_locking_limit = 100;
const double Cherokee_Driveline4WD::m_central_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the Cherokee_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Cherokee_Driveline4WD::Cherokee_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
