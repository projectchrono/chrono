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
const double M113a_Suspension::m_torsion_kstop = 350800;
const double M113a_Suspension::m_torsion_lowerstop = -7.0 / 180.0 * CH_C_PI;
const double M113a_Suspension::m_torsion_upperstop = 47.59 / 180.0 * CH_C_PI;

// -----------------------------------------------------------------------------
// M113 spring functor class - implements a (non)linear rotational spring
// -----------------------------------------------------------------------------
class M113a_SpringTorque : public ChLinkRotSpringCB::TorqueFunctor {
public:
    M113a_SpringTorque(double k, double c, double t, double kstop, double lowerstop, double upperstop) 
        : m_k(k), m_c(c), m_t(t), m_kstop(kstop), m_lowerstop(lowerstop), m_upperstop(upperstop) {}

    virtual double operator()(double time, double angle, double vel) override {
        double force = m_t - m_k * angle - m_c * vel;

        //Apply bump stop spring rates if needed
        if (angle < m_lowerstop) {
            force -= m_kstop*(angle - m_lowerstop);
        }
        else if (angle > m_upperstop) {
            force -= m_kstop*(angle - m_upperstop);
        }

        return force;
    }

private:
    double m_k;
    double m_c;
    double m_t;
    double m_kstop;
    double m_lowerstop;
    double m_upperstop;
};

// -----------------------------------------------------------------------------
// M113 shock functor class - implements a (non)linear translational damper
// -----------------------------------------------------------------------------
class M113a_ShockForce : public ChLinkSpringCB::ForceFunctor {
  public:
    M113a_ShockForce(){}

    virtual double operator()(double time, double rest_length, double length, double vel) {
        // Clip the velocity to within +/- 0.254 m/s [10 in/s] 
        ChClampValue(vel, -0.254, 0.254);

        // Velocity is positive in extension.
        // Linear interpolate between 500lbf in extension at 10 in/s and
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

    // Instantiate the torque callback for the spring.
    m_spring_torqueCB = new M113a_SpringTorque(m_torsion_k, m_torsion_c, m_torsion_t, m_torsion_kstop,
                                               m_torsion_lowerstop, m_torsion_upperstop);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = std::make_shared<M113a_RoadWheelLeft>();
    else
        m_road_wheel = std::make_shared<M113a_RoadWheelRight>();
}

M113a_Suspension::~M113a_Suspension() {
    delete m_shock_forceCB;
    delete m_spring_torqueCB;
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
