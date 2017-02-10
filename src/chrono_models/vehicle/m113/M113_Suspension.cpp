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

#include "chrono_models/vehicle/m113/M113_RoadWheel.h"
#include "chrono_models/vehicle/m113/M113_Suspension.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Suspension::m_arm_mass = 75.26;
const ChVector<> M113_Suspension::m_arm_inertia(0.37, 0.77, 0.77);
const double M113_Suspension::m_arm_radius = 0.03;

const double M113_Suspension::m_torsion_a0 = 0;
const double M113_Suspension::m_torsion_k = 2.5e4;
const double M113_Suspension::m_torsion_c = 5e2;
const double M113_Suspension::m_torsion_t = -1e4;

const double M113_Suspension::m_shock_c = 1e2;

// -----------------------------------------------------------------------------
// M113 shock functor class - implements a (non)linear damper
// -----------------------------------------------------------------------------
class M113_ShockForce : public ChSpringForceCallback {
  public:
    M113_ShockForce(double c) : m_c(c) {}

    virtual double operator()(double time, double rest_length, double length, double vel) { return -m_c * vel; }

  private:
    double m_c;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Suspension::M113_Suspension(VehicleSide side, bool has_shock)
    : ChLinearDamperRWAssembly("", has_shock), m_side(side) {
    // Set subsystem name.
    SetName((side == LEFT) ? "M113_SuspensionLeft" : "M113_SuspensionRight");

    // Instantiate the force callback for the shock (damper).
    m_shock_forceCB = new M113_ShockForce(m_shock_c);

    // Create the torsional force component.
    m_torsion_force = new ChLinkForce;
    m_torsion_force->Set_active(1);
    m_torsion_force->Set_K(m_torsion_k);
    m_torsion_force->Set_R(m_torsion_c);
    m_torsion_force->Set_iforce(m_torsion_t);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = std::make_shared<M113_RoadWheelLeft>();
    else
        m_road_wheel = std::make_shared<M113_RoadWheelRight>();
}

M113_Suspension::~M113_Suspension() {
    delete m_shock_forceCB;
    //// NOTE: Do not delete m_torsion_force here (it is deleted in the destructor for the revolute joint)
    ////delete m_torsion_force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> M113_Suspension::GetLocation(PointId which) {
    ChVector<> point;

    switch (which) {
        case ARM:
            point = ChVector<>(0.144, -0.12, 0.067);
            break;
        case ARM_WHEEL:
            point = ChVector<>(0, -0.12, 0);
            break;
        case ARM_CHASSIS:
            point = ChVector<>(0.288, -0.12, 0.134);
            break;
        case SHOCK_A:
            point = ChVector<>(0.184, -0.12, -0.106);
            break;
        case SHOCK_C:
            point = ChVector<>(-0.3, -0.12, 0.3);
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
