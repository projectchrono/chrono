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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MAN 10t (front1) steered and driven solid three link axle.
//
// =============================================================================

#include "chrono_models/vehicle/man/suspension/MAN_10t_Front1Axle.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_10t_Front1Axle::m_axleTubeMass = 405.0;
const double MAN_10t_Front1Axle::m_spindleMass = 14.705 * 4.1;
const double MAN_10t_Front1Axle::m_bellcrankMass = 24.4;
const double MAN_10t_Front1Axle::m_knuckleMass = 145.6;
const double MAN_10t_Front1Axle::m_draglinkMass = 10.3;
const double MAN_10t_Front1Axle::m_triangleMass = 50.0;
const double MAN_10t_Front1Axle::m_linkMass = 25.0;
const double MAN_10t_Front1Axle::m_tierodMass = 10.0;

const double MAN_10t_Front1Axle::m_axleTubeRadius = 0.0476;
const double MAN_10t_Front1Axle::m_spindleRadius = 0.10;
const double MAN_10t_Front1Axle::m_spindleWidth = 0.06;

const ChVector<> MAN_10t_Front1Axle::m_axleTubeInertia(21.23, 8.12, 21.31);
const ChVector<> MAN_10t_Front1Axle::m_spindleInertia(0.04117 * 6.56, 0.07352 * 6.56, 0.04117 * 6.56);
const ChVector<> MAN_10t_Front1Axle::m_bellcrankInertia(0.05, 0.29, 0.30);
const ChVector<> MAN_10t_Front1Axle::m_knuckleInertia(2.40, 3.97, 2.45);
const ChVector<> MAN_10t_Front1Axle::m_draglinkInertia(0.29, 0.67, 0.95);
const ChVector<> MAN_10t_Front1Axle::m_triangleInertia(0.2, 0.2, 0.2);
const ChVector<> MAN_10t_Front1Axle::m_linkInertia(0.05, 0.1, 0.1);
const ChVector<> MAN_10t_Front1Axle::m_tierodInertia(0.05, 0.05, 0.5);

const double MAN_10t_Front1Axle::m_springDesignLength = 0.480919952;
const double MAN_10t_Front1Axle::m_springCoefficient1 = 85490.0;   // linear
const double MAN_10t_Front1Axle::m_springCoefficient2 = 495208.0;  // quadratic
const double MAN_10t_Front1Axle::m_springRestLength = 0.649;
const double MAN_10t_Front1Axle::m_springMinLength = 0.335;
const double MAN_10t_Front1Axle::m_springMaxLength = m_springDesignLength + 0.15;
const double MAN_10t_Front1Axle::m_damperCoefExpansion = 98727.9;
const double MAN_10t_Front1Axle::m_damperDegresExpansion = 4.77954;
const double MAN_10t_Front1Axle::m_damperCoefCompression = 52526.6;
const double MAN_10t_Front1Axle::m_damperDegresCompression = 3.0;
const double MAN_10t_Front1Axle::m_axleShaftInertia = 0.4 * 6.56;

const double MAN_10t_Front1Axle::m_twin_tire_dist = 0.0;

// ---------------------------------------------------------------------------------------
// MAN 10t spring functor class - implements a spring + bump stop + rebound stop
//
// Differences to MAN 5t and 7t front axle
// - lower spring stiffness
// - modified knuckle/tierod levers to consider the different axle spaces
// ---------------------------------------------------------------------------------------
class MAN_10t_SpringForceFront1 : public ChLinkTSDA::ForceFunctor {
  public:
    MAN_10t_SpringForceFront1(double spring_constant1,
                              double spring_coefficient2,
                              double min_length,
                              double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant1;
    double m_spring_constant2;
    double m_min_length;
    double m_max_length;
    double m_scale;

    ChFunction_Recorder m_bump;
};

MAN_10t_SpringForceFront1::MAN_10t_SpringForceFront1(double spring_constant1,
                                                     double spring_constant2,
                                                     double min_length,
                                                     double max_length)
    : m_spring_constant1(spring_constant1),
      m_spring_constant2(spring_constant2),
      m_min_length(min_length),
      m_max_length(max_length),
      m_scale(0.934239016) {
    // From ADAMS/Car
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3, 200.0);
    m_bump.AddPoint(4.0e-3, 400.0);
    m_bump.AddPoint(6.0e-3, 600.0);
    m_bump.AddPoint(8.0e-3, 800.0);
    m_bump.AddPoint(10.0e-3, 1000.0);
    m_bump.AddPoint(20.0e-3, 2500.0);
    m_bump.AddPoint(30.0e-3, 4500.0);
    m_bump.AddPoint(40.0e-3, 7500.0);
    m_bump.AddPoint(50.0e-3, 12500.0);
}

double MAN_10t_SpringForceFront1::evaluate(double time,
                                           double rest_length,
                                           double length,
                                           double vel,
                                           const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = m_scale * (defl_spring * m_spring_constant1 + defl_spring * std::abs(defl_spring) * m_spring_constant2) +
            m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

MAN_10t_Front1Axle::MAN_10t_Front1Axle(const std::string& name) : ChSolidBellcrankThreeLinkAxle(name) {
    m_springForceCB = chrono_types::make_shared<MAN_10t_SpringForceFront1>(m_springCoefficient1, m_springCoefficient2,
                                                                           m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
MAN_10t_Front1Axle::~MAN_10t_Front1Axle() {}

const ChVector<> MAN_10t_Front1Axle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.000, 0.590, 0.090);
        case SPRING_C:
            return ChVector<>(0.010, 0.552, 0.588);
        case SHOCK_A:
            return ChVector<>(0.246, 0.523, -0.125);
        case SHOCK_C:
            return ChVector<>(0.235, 0.562, 0.570);
        case SPINDLE:
            return ChVector<>(0.0, 2.066 / 2.0, 0.0);
        case TRIANGLE_A:
            return ChVector<>(0.125, 0.000, 0.260);
        case TRIANGLE_C:
            return ChVector<>(0.871, 0.420, 0.165);
        case LINK_A:
            return ChVector<>(-0.115, 0.490, -0.090);
        case LINK_C:
            return ChVector<>(-1.138, 0.270, 0.115);
        case DRAGLINK_S:
            return ChVector<>(0.741, -0.217, 0.089);
        case BELLCRANK_A:
            return ChVector<>(-0.023, 0.000, 0.250);
        case BELLCRANK_D:
            return ChVector<>(0.045, 0.256, 0.153);
        case BELLCRANK_T:
            return ChVector<>(-0.273, 0.042, 0.153);
        case KNUCKLE_L:
            return ChVector<>(0.000, 0.845 + 2 * 0.004374433, -0.10);
        case KNUCKLE_U:
            return ChVector<>(0.000, 0.845 - 2 * 0.004374433, 0.10);
        case KNUCKLE_T:
            // return ChVector<>(-0.236, 0.800, 0.153); 5t/7t front axle
            return ChVector<>(-0.238119607, 0.813061735, 0.153);  // 10 front 1 axle
            // return ChVector<>(-0.235882642, 0.799388826 , 0.153);  // 10 front 2 axle
        case KNUCKLE_CM:
            return ChVector<>(0.000, 0.937, 0.000);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
