//
// Created by Rainer Gericke on 18.04.24.
//

#include "Cherokee_Steering.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_Steering::m_pitmanArmMass = 1.605;
const double Cherokee_Steering::m_pitmanArmRadius = 0.02;
const double Cherokee_Steering::m_maxAngle = 10 * (CH_PI / 180);
const ChVector3d Cherokee_Steering::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d Cherokee_Steering::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

Cherokee_Steering::Cherokee_Steering(const std::string& name) : ChRotaryArm(name) {}

const ChVector3d Cherokee_Steering::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(-0.20, 0.5, 0.0381);
        case ARM_C:
            return ChVector3d(-0.45, 0.5, 0.0381);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d Cherokee_Steering::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 0, 1);
        default:
            return ChVector3d(0, 0, 1);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
