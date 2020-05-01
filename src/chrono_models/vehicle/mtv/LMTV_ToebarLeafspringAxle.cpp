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
// Front LMTV suspension subsystems.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_ToebarLeafspringAxle.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double LMTV_ToebarLeafspringAxle::m_axleTubeMass = 581;
const double LMTV_ToebarLeafspringAxle::m_spindleMass = 14.705;
const double LMTV_ToebarLeafspringAxle::m_knuckleMass = 68.0;
const double LMTV_ToebarLeafspringAxle::m_tierodMass = 5.0;
const double LMTV_ToebarLeafspringAxle::m_draglinkMass = 5.0;

const double LMTV_ToebarLeafspringAxle::m_axleTubeRadius = 0.06;
const double LMTV_ToebarLeafspringAxle::m_spindleRadius = 0.10;
const double LMTV_ToebarLeafspringAxle::m_spindleWidth = 0.06;
const double LMTV_ToebarLeafspringAxle::m_knuckleRadius = 0.06;
const double LMTV_ToebarLeafspringAxle::m_tierodRadius = 0.02;
const double LMTV_ToebarLeafspringAxle::m_draglinkRadius = 0.02;

const ChVector<> LMTV_ToebarLeafspringAxle::m_axleTubeInertia(160.3141845, 1.0458, 160.3141845);
const ChVector<> LMTV_ToebarLeafspringAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> LMTV_ToebarLeafspringAxle::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector<> LMTV_ToebarLeafspringAxle::m_tierodInertia(1.0, 0.1, 1.0);
const ChVector<> LMTV_ToebarLeafspringAxle::m_draglinkInertia(0.1, 1.0, 0.1);

const double LMTV_ToebarLeafspringAxle::m_springDesignLength = 0.2;
const double LMTV_ToebarLeafspringAxle::m_springCoefficient = 428656.6583;
const double LMTV_ToebarLeafspringAxle::m_springRestLength = m_springDesignLength + 0.062122551;
const double LMTV_ToebarLeafspringAxle::m_springMinLength = m_springDesignLength - 0.08;
const double LMTV_ToebarLeafspringAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double LMTV_ToebarLeafspringAxle::m_damperCoefficient = 48240.82294;
const double LMTV_ToebarLeafspringAxle::m_damperDegressivityCompression = 3.0;
const double LMTV_ToebarLeafspringAxle::m_damperDegressivityExpansion = 1.0;
const double LMTV_ToebarLeafspringAxle::m_axleShaftInertia = 0.4;

// ---------------------------------------------------------------------------------------
// UAZBUS spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class UAZBUS_SpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    UAZBUS_SpringForceFront(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

UAZBUS_SpringForceFront::UAZBUS_SpringForceFront(double spring_constant, double min_length, double max_length)
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

double UAZBUS_SpringForceFront::operator()(double time,
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
// UAZBUS shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class UAZBUS_ShockForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    UAZBUS_ShockForceFront(double compression_slope,
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

UAZBUS_ShockForceFront::UAZBUS_ShockForceFront(double compression_slope,
                                               double compression_degressivity,
                                               double expansion_slope,
                                               double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double UAZBUS_ShockForceFront::operator()(double time,
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

LMTV_ToebarLeafspringAxle::LMTV_ToebarLeafspringAxle(const std::string& name) : ChToeBarLeafspringAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<UAZBUS_SpringForceFront>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<UAZBUS_ShockForceFront>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
LMTV_ToebarLeafspringAxle::~LMTV_ToebarLeafspringAxle() {}

const ChVector<> LMTV_ToebarLeafspringAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius + m_springDesignLength);
        case SHOCK_A:
            return ChVector<>(-0.15, 0.7075, m_axleTubeRadius - 0.05);
        case SHOCK_C:
            return ChVector<>(0.0, 0.529, m_axleTubeRadius + m_springDesignLength + 0.2);
        case SPINDLE:
            return ChVector<>(0.0, 1.0025, 0.0);
        case KNUCKLE_CM:
            return ChVector<>(0.0, 0.908341392, 0.0);
        case KNUCKLE_L:
            return ChVector<>(0.0, 0.92597409, -0.1);
        case KNUCKLE_U:
            return ChVector<>(0.0, 0.890708694, 0.1);
        case KNUCKLE_DRL:
            return ChVector<>(0.0, 0.708341392, 0.1);
        case TIEROD_K:
            return ChVector<>(-0.2, 0.862974035, 0.1);
        case DRAGLINK_C:
            return ChVector<>(1.0, 0.708341392, 0.1);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
