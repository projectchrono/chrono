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
// Rear U401 suspension subsystems (pushpipe).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/unimog/U401_PushPipeAxle.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_PushPipeAxle::m_axleTubeMass = 124.0;
const double U401_PushPipeAxle::m_spindleMass = 14.705;
const double U401_PushPipeAxle::m_panhardRodMass = 15.0;

const double U401_PushPipeAxle::m_portalOffset = 0.05;

const double U401_PushPipeAxle::m_axleTubeRadius = 0.0476;
const double U401_PushPipeAxle::m_spindleRadius = 0.10;
const double U401_PushPipeAxle::m_spindleWidth = 0.06;
const double U401_PushPipeAxle::m_panhardRodRadius = 0.02;

const ChVector<> U401_PushPipeAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> U401_PushPipeAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> U401_PushPipeAxle::m_panhardRodInertia(0.1, 0.02, 0.1);

const double U401_PushPipeAxle::m_springDesignLength = 0.35;
const double U401_PushPipeAxle::m_springCoefficient = 102643.885771329;
const double U401_PushPipeAxle::m_springRestLength = m_springDesignLength + 0.0621225507207084;
const double U401_PushPipeAxle::m_springMinLength = m_springDesignLength - 0.08;
const double U401_PushPipeAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double U401_PushPipeAxle::m_damperCoefficient = 16336.2817986669;
const double U401_PushPipeAxle::m_damperDegressivityCompression = 3.0;
const double U401_PushPipeAxle::m_damperDegressivityExpansion = 1.0;
const double U401_PushPipeAxle::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// U401 spring functor class - implements a linear spring + bump stop +
// rebound stop
// ---------------------------------------------------------------------------------------
class U401_PPSpringForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    U401_PPSpringForceRear(double spring_constant, double min_length, double max_length);

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

U401_PPSpringForceRear::U401_PPSpringForceRear(double spring_constant, double min_length, double max_length)
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

double U401_PPSpringForceRear::evaluate(double time,
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
class U401_PPShockForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    U401_PPShockForceRear(double compression_slope,
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

U401_PPShockForceRear::U401_PPShockForceRear(double compression_slope,
                                                   double compression_degressivity,
                                                   double expansion_slope,
                                                   double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double U401_PPShockForceRear::evaluate(double time,
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

U401_PushPipeAxle::U401_PushPipeAxle(const std::string& name) : ChPushPipeAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<U401_PPSpringForceRear>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<U401_PPShockForceRear>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
U401_PushPipeAxle::~U401_PushPipeAxle() {}

const ChVector<> U401_PushPipeAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(-0.18, 0.4242, 0.124);
        case SPRING_C:
            return ChVector<>(-0.18, 0.4242, 0.468);
        case SHOCK_A:
            return ChVector<>(0.1, 0.4242, 0.124);
        case SHOCK_C:
            return ChVector<>(0.1, 0.4242, 0.468);
        case SPINDLE:
            return ChVector<>(0.0, 0.635, 0.0);
        case AXLE_C:
            return ChVector<>(0.68, -0.1, 0.335);
        case PANHARD_A:
            return ChVector<>(-0.1, -0.45, 0.0 + m_portalOffset);
        case PANHARD_C:
            return ChVector<>(-0.1, 0.45, 0.0 + m_portalOffset);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono

