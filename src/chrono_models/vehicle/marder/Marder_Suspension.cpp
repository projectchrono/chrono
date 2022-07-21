// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Marder suspension subsystem.
//
// =============================================================================

#include "chrono_models/vehicle/marder/Marder_RoadWheel.h"
#include "chrono_models/vehicle/marder/Marder_Suspension.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_Suspension::m_arm_mass = 75.26;
const ChVector<> Marder_Suspension::m_arm_inertia(0.37, 0.77, 0.77);
const double Marder_Suspension::m_arm_radius = 0.03;

const double Marder_Suspension::m_torsion_a0 = 0;
const double Marder_Suspension::m_torsion_k = 84220.62422;
const double Marder_Suspension::m_torsion_c = 5e2;
const double Marder_Suspension::m_torsion_t = -2.7e4;

const double Marder_Suspension::m_shock_c = 17771.53175;

// -----------------------------------------------------------------------------
// Marder spring functor class - implements a (non)linear rotational spring
// -----------------------------------------------------------------------------
class Marder_SpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    Marder_SpringTorque(double k, double c, double t) : m_k(k), m_c(c), m_t(t) {}

    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return m_t - m_k * angle - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_t;
};

// -----------------------------------------------------------------------------
// Marder shock functor class - implements a (non)linear translational damper
// -----------------------------------------------------------------------------
class Marder_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    Marder_ShockForce(double c) : m_c(c) {}

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override {
        return -m_c * vel;
    }

  private:
    double m_c;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_Suspension::Marder_Suspension(const std::string& name, VehicleSide side, int index, bool has_shock)
    : ChTranslationalDamperSuspension(name, has_shock), m_side(side) {
    // Instantiate the force callback for the shock (damper).
    m_shock_forceCB = chrono_types::make_shared<Marder_ShockForce>(m_shock_c);

    // Instantiate the torque callback for the spring.
    m_spring_torqueCB = chrono_types::make_shared<Marder_SpringTorque>(m_torsion_k, m_torsion_c, m_torsion_t);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = chrono_types::make_shared<Marder_RoadWheelLeft>(index);
    else
        m_road_wheel = chrono_types::make_shared<Marder_RoadWheelRight>(index);
}

Marder_Suspension::~Marder_Suspension() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<> Marder_Suspension::GetLocation(PointId which) {
    ChVector<> point;

    switch (which) {
        case ARM:
            point = ChVector<>(0.2, -0.168, 0.0325);
            break;
        case ARM_WHEEL:
            point = ChVector<>(0, -0.168, 0);
            break;
        case ARM_CHASSIS:
            point = ChVector<>(0.4, -0.168, 0.065);
            break;
        case SHOCK_A:
            point = ChVector<>(-0.034, -0.168, 0.152);
            break;
        case SHOCK_C:
            point = ChVector<>(-0.128, -0.168, 0.542);
            break;
        default:
            point = ChVector<>(0, 0, 0);
            break;
    }

    if (m_side == RIGHT)
        point.y() *= -1;

    return point;
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
