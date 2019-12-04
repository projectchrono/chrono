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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear UAZBUS suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double UAZBUS_ToeBarLeafspringAxle::m_axleTubeMass = 124.0;
const double UAZBUS_ToeBarLeafspringAxle::m_spindleMass = 14.705;
const double UAZBUS_ToeBarLeafspringAxle::m_knuckleMass = 10.0;
const double UAZBUS_ToeBarLeafspringAxle::m_tierodMass  = 5.0;
const double UAZBUS_ToeBarLeafspringAxle::m_draglinkMass = 5.0;

const double UAZBUS_ToeBarLeafspringAxle::m_axleTubeRadius = 0.0476;
const double UAZBUS_ToeBarLeafspringAxle::m_spindleRadius = 0.10;
const double UAZBUS_ToeBarLeafspringAxle::m_spindleWidth = 0.06;
const double UAZBUS_ToeBarLeafspringAxle::m_knuckleRadius = 0.05;
const double UAZBUS_ToeBarLeafspringAxle::m_tierodRadius = 0.02;
const double UAZBUS_ToeBarLeafspringAxle::m_draglinkRadius = 0.02;

const ChVector<> UAZBUS_ToeBarLeafspringAxle::m_axleTubeInertia(22.21, 0.0775, 22.21);
const ChVector<> UAZBUS_ToeBarLeafspringAxle::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> UAZBUS_ToeBarLeafspringAxle::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector<> UAZBUS_ToeBarLeafspringAxle::m_tierodInertia(1.0, 0.1, 1.0);
const ChVector<> UAZBUS_ToeBarLeafspringAxle::m_draglinkInertia(0.1, 1.0, 0.1);

const double UAZBUS_ToeBarLeafspringAxle::m_springDesignLength = 0.2;
const double UAZBUS_ToeBarLeafspringAxle::m_springCoefficient = 94748.2022504578;
const double UAZBUS_ToeBarLeafspringAxle::m_springRestLength  = m_springDesignLength + 0.0621225507207084;
const double UAZBUS_ToeBarLeafspringAxle::m_springMinLength = m_springDesignLength - 0.08;
const double UAZBUS_ToeBarLeafspringAxle::m_springMaxLength = m_springDesignLength + 0.08;
const double UAZBUS_ToeBarLeafspringAxle::m_damperCoefficient = 15079.644737231;
const double UAZBUS_ToeBarLeafspringAxle::m_damperDegressivityCompression = 3.0;
const double UAZBUS_ToeBarLeafspringAxle::m_damperDegressivityExpansion = 1.0;
const double UAZBUS_ToeBarLeafspringAxle::m_axleShaftInertia = 0.4;


// ---------------------------------------------------------------------------------------
// UAZBUS spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class UAZBUS_SpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    UAZBUS_SpringForceFront(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkTSDA* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;
    
    ChFunction_Recorder m_bump;

};

UAZBUS_SpringForceFront::UAZBUS_SpringForceFront(double spring_constant, double min_length, double max_length) :
    m_spring_constant(spring_constant),
    m_min_length(min_length),
    m_max_length(max_length)  {
    
    // From ADAMS/Car
    m_bump.AddPoint(0.0,          0.0);
    m_bump.AddPoint(2.0e-3,     200.0);
    m_bump.AddPoint(4.0e-3,     400.0);
    m_bump.AddPoint(6.0e-3,     600.0);
    m_bump.AddPoint(8.0e-3,     800.0);
    m_bump.AddPoint(10.0e-3,   1000.0);
    m_bump.AddPoint(20.0e-3,   2500.0);
    m_bump.AddPoint(30.0e-3,   4500.0);
    m_bump.AddPoint(40.0e-3,   7500.0);
    m_bump.AddPoint(50.0e-3,  12500.0);

}

double UAZBUS_SpringForceFront::operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) {
    /*
     * 
    */

    double force = 0;

    double defl_spring  = rest_length - length;
    double defl_bump    = 0.0;
    double defl_rebound = 0.0;
    
    if(length < m_min_length) {
        defl_bump = m_min_length - length;
    }
    
    if(length > m_max_length) {
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

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkTSDA* link) override;

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

double UAZBUS_ShockForceFront::operator()(double time, double rest_length, double length, double vel, ChLinkTSDA* link) {
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


UAZBUS_ToeBarLeafspringAxle::UAZBUS_ToeBarLeafspringAxle(const std::string& name) : ChToeBarLeafspringAxle(name) {
/*
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new LinearDamperForce(m_damperCoefficient  // coefficient for linear damper
                    );
*/
    m_springForceCB = new UAZBUS_SpringForceFront(m_springCoefficient,m_springMinLength,m_springMaxLength);
    
    m_shockForceCB = new UAZBUS_ShockForceFront(m_damperCoefficient,
        m_damperDegressivityCompression,
        m_damperCoefficient,
        m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
UAZBUS_ToeBarLeafspringAxle::~UAZBUS_ToeBarLeafspringAxle() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

const ChVector<> UAZBUS_ToeBarLeafspringAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.3824, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.3824, m_axleTubeRadius+m_springDesignLength);
        case SHOCK_A:
            return ChVector<>(-0.125, 0.441, -0.0507);
        case SHOCK_C:
            return ChVector<>(-0.3648,  0.4193, 0.3298);
        case SPINDLE:
            return ChVector<>(0.0, 0.7325, 0.0);
        case KNUCKLE_CM:
            return ChVector<>(0.0, 0.7325-0.07, 0.0);
        case KNUCKLE_L:
            return ChVector<>(0.0, 0.7325-0.07+0.0098058067569092, -0.1);
        case KNUCKLE_U :
            return ChVector<>(0.0, 0.7325-0.07-0.0098058067569092, 0.1);
        case KNUCKLE_DRL:
            return ChVector<>(0.0, 0.7325-0.2, 0.2);
        case TIEROD_K:
            return ChVector<>(-0.190568826619798, 0.7325-0.07-0.060692028477827, 0.1);
        case DRAGLINK_C:
            return ChVector<>(0.6, 0.7325-0.2, 0.2);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
