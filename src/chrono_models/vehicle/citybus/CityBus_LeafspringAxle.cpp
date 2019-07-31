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
// Authors: Radu Serban, Rainer Gericke, Shuo He
// =============================================================================
//
// Rear CityBus suspension subsystems (simple leafspring work a like).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_LeafspringAxle.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double CityBus_LeafspringAxle::m_axleTubeMass = 124.0*4.1;
const double CityBus_LeafspringAxle::m_spindleMass = 14.705*4.1;

const double CityBus_LeafspringAxle::m_axleTubeRadius = 0.0476;
const double CityBus_LeafspringAxle::m_spindleRadius = 0.10;
const double CityBus_LeafspringAxle::m_spindleWidth = 0.06;

const ChVector<> CityBus_LeafspringAxle::m_axleTubeInertia(22.21*6.56, 0.0775*6.56, 22.21*6.56);
const ChVector<> CityBus_LeafspringAxle::m_spindleInertia(0.04117*6.56, 0.07352*6.56, 0.04117*6.56);

const double CityBus_LeafspringAxle::m_springDesignLength = 0.4;
const double CityBus_LeafspringAxle::m_springCoefficient = 565480 / 3.184 * 4.0;
const double CityBus_LeafspringAxle::m_springRestLength  = m_springDesignLength + 0.0621225507207084;
const double CityBus_LeafspringAxle::m_springMinLength = m_springDesignLength - 0.10;
const double CityBus_LeafspringAxle::m_springMaxLength = m_springDesignLength + 0.10;
const double CityBus_LeafspringAxle::m_damperCoefficient = 30276 / 3.184 * 4*2;
const double CityBus_LeafspringAxle::m_damperDegressivityCompression = 3.0;
const double CityBus_LeafspringAxle::m_damperDegressivityExpansion = 1.0;
const double CityBus_LeafspringAxle::m_axleShaftInertia = 0.4 * 6.56;


// ---------------------------------------------------------------------------------------
// CityBus spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class CityBus_SpringForceRear : public ChLinkSpringCB::ForceFunctor {
  public:
    CityBus_SpringForceRear(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;
    
    ChFunction_Recorder m_bump;

};

CityBus_SpringForceRear::CityBus_SpringForceRear(double spring_constant, double min_length, double max_length) :
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

double CityBus_SpringForceRear::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
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
// CityBus shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class CityBus_ShockForceRear : public ChLinkSpringCB::ForceFunctor {
  public:
    CityBus_ShockForceRear(double compression_slope,
                     double compression_degressivity,
                     double expansion_slope,
                     double expansion_degressivity);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

CityBus_ShockForceRear::CityBus_ShockForceRear(double compression_slope,
                                   double compression_degressivity,
                                   double expansion_slope,
                                   double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double CityBus_ShockForceRear::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
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


CityBus_LeafspringAxle::CityBus_LeafspringAxle(const std::string& name) : ChLeafspringAxle(name) {
/*
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new LinearDamperForce(m_damperCoefficient  // coefficient for linear damper
                    );
*/
    m_springForceCB = new CityBus_SpringForceRear(m_springCoefficient,m_springMinLength,m_springMaxLength);
    
    m_shockForceCB = new CityBus_ShockForceRear(m_damperCoefficient,
        m_damperDegressivityCompression,
        m_damperCoefficient,
        m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
CityBus_LeafspringAxle::~CityBus_LeafspringAxle() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

const ChVector<> CityBus_LeafspringAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.5142, m_axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.5142, m_axleTubeRadius+m_springDesignLength-0.1);
        case SHOCK_A:
            return ChVector<>(-0.125, 0.441, -0.0507);
        case SHOCK_C:
            return ChVector<>(-0.3648,  0.4193, 0.4298-0.1);
        case SPINDLE:
            return ChVector<>(0.0, 0.7325+0.375, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
