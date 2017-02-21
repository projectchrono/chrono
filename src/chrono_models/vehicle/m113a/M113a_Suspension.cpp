// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 suspension subsystem.
//
// =============================================================================

#include "chrono_models/vehicle/m113a/M113a_RoadWheel.h"
#include "chrono_models/vehicle/m113a/M113a_Suspension.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_Suspension::m_arm_mass = 16.513;
const ChVector<> M113a_Suspension::m_arm_inertia(3.8, 0.08, 3.8);
const double M113a_Suspension::m_arm_radius = 0.019;

const double M113a_Suspension::m_torsion_a0 = 0;
const double M113a_Suspension::m_torsion_k = 9557;
const double M113a_Suspension::m_torsion_c = 191;
const double M113a_Suspension::m_torsion_t = 0;

// -----------------------------------------------------------------------------
// M113 shock functor class - implements a (non)linear damper
// -----------------------------------------------------------------------------
class M113a_ShockForce : public ChSpringForceCallback {
  public:
    M113a_ShockForce(){}

    virtual double operator()(double time, double rest_length, double length, double vel) {
        //clip the velocity to within +/- 0.254 m/s [10 in/s] 
        vel = vel <= -0.254 ? -0.254 : vel >= 0.254 ? 0.254 : vel;

        //Velocity is positive in extension
        //linear interpolate between 500lbf in extension at 10 in/s and
        // -2000lbf at -10in/s (converted to N)
        return vel >= 0 ? (-8756.341762 * vel) : (-35025.36705 * vel);
    }
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_Suspension::M113a_Suspension(VehicleSide side, bool has_shock)
    : ChLinearDamperRWAssembly("", has_shock), m_side(side) {
    // Set subsystem name.
    SetName((side == LEFT) ? "M113a_SuspensionLeft" : "M113a_SuspensionRight");

    // Instantiate the force callback for the shock (damper).
    m_shock_forceCB = new M113a_ShockForce();

    // Create the torsional force component.
    m_torsion_force = new ChLinkForce;
    m_torsion_force->Set_active(1);
    m_torsion_force->Set_K(m_torsion_k);
    m_torsion_force->Set_R(m_torsion_c);
    m_torsion_force->Set_iforce(m_torsion_t);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = std::make_shared<M113a_RoadWheelLeft>();
    else
        m_road_wheel = std::make_shared<M113a_RoadWheelRight>();
}

M113a_Suspension::~M113a_Suspension() {
    delete m_shock_forceCB;
    //// NOTE: Do not delete m_torsion_force here (it is deleted in the destructor for the revolute joint)
    ////delete m_torsion_force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> M113a_Suspension::GetLocation(PointId which) {
    ChVector<> point;

    switch (which) {
        case ARM:
            point = ChVector<>(0.213, -0.12, 0.199);
            break;
        case ARM_WHEEL:
            point = ChVector<>(0, -0.12, 0);
            break;
        case ARM_CHASSIS:
            point = ChVector<>(0.232, -0.12, 0.217);
            break;
        case SHOCK_A:
            point = ChVector<>(0, -0.12, 0);
            break;
        case SHOCK_C:
            point = ChVector<>(0, -0.12, 1);
            break;
        default:
            point = ChVector<>(0, 0, 0);
            break;
    }

    if (m_side == RIGHT)
        point.y() *= -1;

    return point;
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
