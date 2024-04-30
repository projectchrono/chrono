//
// Created by Rainer Gericke on 16.04.24.
//

#include "Cherokee_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_BrakeShafts::m_maxtorque = 4000;
const double Cherokee_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_BrakeShafts::Cherokee_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
