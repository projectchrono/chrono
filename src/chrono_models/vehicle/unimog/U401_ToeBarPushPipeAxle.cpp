// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Front U401 suspension subsystems (toebar pushpipe).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/unimog/U401_ToeBarPushPipeAxle.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_ToeBarPushPipeAxle::m_axleTubeMass = 124.0;
const double U401_ToeBarPushPipeAxle::m_panhardRodMass = 15.0;
const double U401_ToeBarPushPipeAxle::m_spindleMass = 14.705;
const double U401_ToeBarPushPipeAxle::m_knuckleMass = 10.0;
const double U401_ToeBarPushPipeAxle::m_tierodMass = 5.0;
const double U401_ToeBarPushPipeAxle::m_draglinkMass = 5.0;

const double U401_ToeBarPushPipeAxle::m_axleTubeRadius = 0.0476;
const double U401_ToeBarPushPipeAxle::m_spindleRadius = 0.10;
const double U401_ToeBarPushPipeAxle::m_spindleWidth = 0.06;
const double U401_ToeBarPushPipeAxle::m_knuckleRadius = 0.05;
const double U401_ToeBarPushPipeAxle::m_tierodRadius = 0.02;
const double U401_ToeBarPushPipeAxle::m_draglinkRadius = 0.02;
const double U401_ToeBarPushPipeAxle::m_panhardRodRadius = 0.04;

const double U401_ToeBarPushPipeAxle::m_portalOffset = 0.05;

const ChVector<> U401_ToeBarPushPipeAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> U401_ToeBarPushPipeAxle::m_panhardRodInertia(0.1, 0.02, 0.1);
const ChVector<> U401_ToeBarPushPipeAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> U401_ToeBarPushPipeAxle::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector<> U401_ToeBarPushPipeAxle::m_tierodInertia(1.0, 0.1, 1.0);
const ChVector<> U401_ToeBarPushPipeAxle::m_draglinkInertia(0.1, 1.0, 0.1);

const double U401_ToeBarPushPipeAxle::m_springDesignLength = 0.32;
const double U401_ToeBarPushPipeAxle::m_springCoefficient = 94748.2022504578;
const double U401_ToeBarPushPipeAxle::m_springRestLength = m_springDesignLength + 0.0621225507207084;
const double U401_ToeBarPushPipeAxle::m_springMinLength = m_springDesignLength - 0.08;
const double U401_ToeBarPushPipeAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double U401_ToeBarPushPipeAxle::m_damperCoefficient = 15079.644737231;
const double U401_ToeBarPushPipeAxle::m_damperDegressivityCompression = 3.0;
const double U401_ToeBarPushPipeAxle::m_damperDegressivityExpansion = 1.0;
const double U401_ToeBarPushPipeAxle::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// U401 spring functor class - implements a linear spring + bump stop +
// rebound stop
// ---------------------------------------------------------------------------------------
class U401_FPPSpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    U401_FPPSpringForceFront(double spring_constant, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

U401_FPPSpringForceFront::U401_FPPSpringForceFront(double spring_constant, double min_length, double max_length)
    : m_spring_constant(spring_constant), m_min_length(min_length), m_max_length(max_length) {
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

double U401_FPPSpringForceFront::evaluate(double time,
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

    force = defl_spring * m_spring_constant + m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

// -----------------------------------------------------------------------------
// U401 shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class U401_FPPShockForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    U401_FPPShockForceFront(double compression_slope,
                               double compression_degressivity,
                               double expansion_slope,
                               double expansion_degressivity);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

U401_FPPShockForceFront::U401_FPPShockForceFront(double compression_slope,
                                                       double compression_degressivity,
                                                       double expansion_slope,
                                                       double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double U401_FPPShockForceFront::evaluate(double time,
                                            double rest_length,
                                            double length,
                                            double vel,
                                            const ChLinkTSDA& link) {
    // Simple model of a degressive damping characteristic
    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = -m_slope_expand / (1.0 + m_degres_expand * std::abs(vel)) * vel;
    } else {
        force = -m_slope_compr / (1.0 + m_degres_compr * std::abs(vel)) * vel;
    }

    return force;
}

U401_ToeBarPushPipeAxle::U401_ToeBarPushPipeAxle(const std::string& name) : ChToeBarPushPipeAxle(name) {
    m_springForceCB = chrono_types::make_shared<U401_FPPSpringForceFront>(m_springCoefficient, m_springMinLength,
                                                                             m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<U401_FPPShockForceFront>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
U401_ToeBarPushPipeAxle::~U401_ToeBarPushPipeAxle() {}

const ChVector<> U401_ToeBarPushPipeAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.18, 0.4242, 0.124);
        case SPRING_C:
            return ChVector<>(0.18, 0.4242, 0.468);
        case SHOCK_A:
            return ChVector<>(-0.1, 0.4242, 0.124);
        case SHOCK_C:
            return ChVector<>(-0.1, 0.4242, 0.468);
        case SPINDLE:
            return ChVector<>(0.0, 0.635, 0.0);
        case KNUCKLE_CM:
            return ChVector<>(0.0, 0.635 - 0.07, 0.0);
        case KNUCKLE_L:
            return ChVector<>(0.0, 0.635 - 0.07 + 0.0098058067569092, -0.1);
        case KNUCKLE_U:
            return ChVector<>(0.0, 0.635 - 0.07 - 0.0098058067569092, 0.1);
        case KNUCKLE_DRL:
            return ChVector<>(0.0, 0.635 - 0.2, 0.2);
        case TIEROD_K:
            return ChVector<>(-0.190568826619798, 0.635 - 0.07 - 0.060692028477827, 0.1);
        case DRAGLINK_C:
            return ChVector<>(0.6, 0.635 - 0.2, 0.2);
        case AXLE_C:
            return ChVector<>(-0.662, 0.1, 0.181);
        case PANHARD_A:
            return ChVector<>(0.1, 0.45, m_portalOffset);
        case PANHARD_C:
            return ChVector<>(0.1, -0.45, m_portalOffset);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono

