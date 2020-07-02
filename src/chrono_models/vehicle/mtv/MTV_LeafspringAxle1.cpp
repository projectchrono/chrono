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
// Rear FMTV suspension subsystems (simple leafspring work a like).
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/MTV_LeafspringAxle1.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double MTV_LeafspringAxle1::m_axleTubeMass = 717.0;
const double MTV_LeafspringAxle1::m_spindleMass = 14.705;

const double MTV_LeafspringAxle1::m_axleTubeRadius = 0.06;
const double MTV_LeafspringAxle1::m_spindleRadius = 0.10;
const double MTV_LeafspringAxle1::m_spindleWidth = 0.06;

const ChVector<> MTV_LeafspringAxle1::m_axleTubeInertia(240.8417938, 1.2906, 240.8417938);
const ChVector<> MTV_LeafspringAxle1::m_spindleInertia(0.04117, 0.07352, 0.04117);

const double MTV_LeafspringAxle1::m_springDesignLength = 0.2;
const double MTV_LeafspringAxle1::m_springCoefficient = 366991.3701;
const double MTV_LeafspringAxle1::m_springRestLength = m_springDesignLength + 0.062122551;
const double MTV_LeafspringAxle1::m_springMinLength = m_springDesignLength - 0.08;
const double MTV_LeafspringAxle1::m_springMaxLength = m_springDesignLength + 0.08;
const double MTV_LeafspringAxle1::m_damperCoefficient = 41301.03979;
const double MTV_LeafspringAxle1::m_damperDegressivityCompression = 3.0;
const double MTV_LeafspringAxle1::m_damperDegressivityExpansion = 1.0;
const double MTV_LeafspringAxle1::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
class MTV_SpringForceRear1 : public ChLinkTSDA::ForceFunctor {
  public:
    MTV_SpringForceRear1(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

MTV_SpringForceRear1::MTV_SpringForceRear1(double spring_constant, double min_length, double max_length)
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

double MTV_SpringForceRear1::operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) {
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
// MTV shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class MTV_ShockForceRear1 : public ChLinkTSDA::ForceFunctor {
  public:
    MTV_ShockForceRear1(double compression_slope,
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

MTV_ShockForceRear1::MTV_ShockForceRear1(double compression_slope,
                                         double compression_degressivity,
                                         double expansion_slope,
                                         double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double MTV_ShockForceRear1::operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) {
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

MTV_LeafspringAxle1::MTV_LeafspringAxle1(const std::string& name) : ChLeafspringAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<MTV_SpringForceRear1>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<MTV_ShockForceRear1>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

const ChVector<> MTV_LeafspringAxle1::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius + m_springDesignLength);
        case SHOCK_A:
            return ChVector<>(0.15, 0.7075, m_axleTubeRadius - 0.05);
        case SHOCK_C:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius + m_springDesignLength + 0.2);
        case SPINDLE:
            return ChVector<>(0.0, 1.0025, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
