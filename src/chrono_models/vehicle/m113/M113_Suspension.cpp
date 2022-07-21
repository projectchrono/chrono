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
// M113 spring functor class - implements a (non)linear rotational spring
// -----------------------------------------------------------------------------
class M113_SpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    M113_SpringTorque(double k, double c, double t) : m_k(k), m_c(c), m_t(t) {}

    virtual double evaluate(double time, double angle, double vel, const ChLinkRSDA& link) override {
        return m_t - m_k * angle - m_c * vel;
    }

  private:
    double m_k;
    double m_c;
    double m_t;
};

// -----------------------------------------------------------------------------
// M113 shock functor class - implements a (non)linear translational damper
// -----------------------------------------------------------------------------
class M113_ShockForce : public ChLinkTSDA::ForceFunctor {
  public:
    M113_ShockForce(double c) : m_c(c) {}

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
M113_Suspension::M113_Suspension(const std::string& name,
                                 VehicleSide side,
                                 int index,
                                 bool use_bushings,
                                 bool has_shock)
    : ChTranslationalDamperSuspension(name, has_shock), m_side(side) {
    // Instantiate the force callback for the shock (damper).
    m_shock_forceCB = chrono_types::make_shared<M113_ShockForce>(m_shock_c);

    // Instantiate the torque callback for the spring.
    m_spring_torqueCB = chrono_types::make_shared<M113_SpringTorque>(m_torsion_k, m_torsion_c, m_torsion_t);

    // Create the associated road wheel.
    if (side == LEFT)
        m_road_wheel = chrono_types::make_shared<M113_RoadWheelLeft>(index);
    else
        m_road_wheel = chrono_types::make_shared<M113_RoadWheelRight>(index);

    // Create bushing data (if enabled)
    if (use_bushings) {
        m_bushing_data = chrono_types::make_shared<ChVehicleBushingData>();
        m_bushing_data = chrono_types::make_shared<ChVehicleBushingData>();
        m_bushing_data->K_lin = 35000000;
        m_bushing_data->K_rot = 300;
        m_bushing_data->D_lin = 100;
        m_bushing_data->D_rot = 100;
    }
}

M113_Suspension::~M113_Suspension() {}

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
