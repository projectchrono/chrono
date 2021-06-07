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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Rear Kraz 64431 suspension subsystems (simple leafspring work-a-like).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_trailer_Suspension.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Kraz_trailer_Suspension::m_axleTubeMass = 500.0;
const double Kraz_trailer_Suspension::m_spindleMass = 14.705;

const double Kraz_trailer_Suspension::m_axleTubeRadius = 0.08;
const double Kraz_trailer_Suspension::m_spindleRadius = 0.10;
const double Kraz_trailer_Suspension::m_spindleWidth = 0.06;

const ChVector<> Kraz_trailer_Suspension::m_axleTubeInertia(178.4760417, 1.6, 178.4760417);
const ChVector<> Kraz_trailer_Suspension::m_spindleInertia(0.04117, 0.07352, 0.04117);

const double Kraz_trailer_Suspension::m_springDesignLength = 0.2;
const double Kraz_trailer_Suspension::m_springCoefficient = 710611.5169;
const double Kraz_trailer_Suspension::m_springRestLength = m_springDesignLength + 0.062122551;
const double Kraz_trailer_Suspension::m_springMinLength = m_springDesignLength - 0.08;
const double Kraz_trailer_Suspension::m_springMaxLength = m_springDesignLength + 0.08;
const double Kraz_trailer_Suspension::m_damperCoefficient = 113097.3355;
const double Kraz_trailer_Suspension::m_damperDegressivityCompression = 3.0;
const double Kraz_trailer_Suspension::m_damperDegressivityExpansion = 1.0;
const double Kraz_trailer_Suspension::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// Trailer spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class Trailer_SpringForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    Trailer_SpringForceRear(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

Trailer_SpringForceRear::Trailer_SpringForceRear(double spring_constant, double min_length, double max_length)
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

double Trailer_SpringForceRear::operator()(double time,
                                           double rest_length,
                                           double length,
                                           double vel,
                                           ChLinkTSDA* link) {
    /*
     *
     */

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
// Trailer shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class Trailer_ShockForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    Trailer_ShockForceRear(double compression_slope,
                           double compression_degressivity,
                           double expansion_slope,
                           double expansion_degressivity);

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

Trailer_ShockForceRear::Trailer_ShockForceRear(double compression_slope,
                                               double compression_degressivity,
                                               double expansion_slope,
                                               double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double Trailer_ShockForceRear::operator()(double time,
                                          double rest_length,
                                          double length,
                                          double vel,
                                          ChLinkTSDA* link) {
    /*
     * Simple model of a degressive damping characteristic
     */

    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = -m_slope_expand / (1.0 + m_degres_expand * std::abs(vel)) * vel;
    } else {
        force = -m_slope_compr / (1.0 + m_degres_compr * std::abs(vel)) * vel;
    }

    return force;
}

Kraz_trailer_Suspension::Kraz_trailer_Suspension(const std::string& name) : ChLeafspringAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<Trailer_SpringForceRear>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<Trailer_ShockForceRear>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

const ChVector<> Kraz_trailer_Suspension::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.729, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.729, m_axleTubeRadius + m_springDesignLength);
        case SHOCK_A:
            return ChVector<>(-0.15, 0.7075, m_axleTubeRadius - 0.05);
        case SHOCK_C:
            return ChVector<>(0.0, 0.629, m_axleTubeRadius + m_springDesignLength + 0.2);
        case SPINDLE:
            return ChVector<>(0.0, 1.0325, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
