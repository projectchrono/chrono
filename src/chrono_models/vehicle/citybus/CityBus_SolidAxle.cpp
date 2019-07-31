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
// Authors: Daniel Melanz, Radu Serban, Evan Hoerl, Shuo He
// =============================================================================
//
// City Bus solid axle suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origin at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_SolidAxle.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_SolidAxleFront::m_ULMass = 25.0;//1.446;
const double CityBus_SolidAxleFront::m_LLMass = 25.0;//2.892;
const double CityBus_SolidAxleFront::m_knuckleMass = 15.0;//1.356;
const double CityBus_SolidAxleFront::m_spindleMass = 5.0;//0.248;
const double CityBus_SolidAxleFront::m_axleTubeMass = 100.0;//44.958;
const double CityBus_SolidAxleFront::m_tierodMass = 5.0;//1.633;
const double CityBus_SolidAxleFront::m_draglinkMass = 2.5;//0.464;
const double CityBus_SolidAxleFront::m_bellCrankMass = 2.5;//0.218;

const double CityBus_SolidAxleFront::m_spindleRadius = 0.10;
const double CityBus_SolidAxleFront::m_spindleWidth = 0.05;
const double CityBus_SolidAxleFront::m_ULRadius = 0.02;
const double CityBus_SolidAxleFront::m_LLRadius = 0.02;
const double CityBus_SolidAxleFront::m_axleTubeRadius = 0.05;
const double CityBus_SolidAxleFront::m_knuckleRadius = 0.02;
const double CityBus_SolidAxleFront::m_tierodRadius = 0.01;
const double CityBus_SolidAxleFront::m_draglinkRadius = 0.01;
const double CityBus_SolidAxleFront::m_bellCrankRadius = 0.01;

const ChVector<> CityBus_SolidAxleFront::m_axleTubeCOM(0, 0, 0);

const ChVector<> CityBus_SolidAxleFront::m_axleTubeInertia(52.333, .5, 52.333);//(7.744, 0.045, 7.744);
const ChVector<> CityBus_SolidAxleFront::m_spindleInertia(.0135, .025, .0135);//(0.0558, 0.0279, 0.0558);
const ChVector<> CityBus_SolidAxleFront::m_ULInertia(1.023, .005, 1.023);//(0.011, 0.011, 0.00142);
const ChVector<> CityBus_SolidAxleFront::m_LLInertia(1.023, .005, 1.023);//(0.0514, 0.0514, 0.0037);
const ChVector<> CityBus_SolidAxleFront::m_knuckleInertia(.0219, .019, .0219);//(0.0255, 0.0134, 0.0196);
const ChVector<> CityBus_SolidAxleFront::m_tierodInertia(2.604, .00025, 2.604);//(0.252, 0.01, 0.252);
const ChVector<> CityBus_SolidAxleFront::m_draglinkInertia(.05,.0001,.05);//(0.005, 0.005, 0.001);
const ChVector<> CityBus_SolidAxleFront::m_bellCrankInertia(.05,.0001,.05);//(0.001, 0.001, 0.001);

const double CityBus_SolidAxleFront::m_axleInertia = 0.4;

const double CityBus_SolidAxleFront::m_springCoefficient = 500000;//26706.20;
const double CityBus_SolidAxleFront::m_dampingCoefficient = 20000;//22459.0;
const double CityBus_SolidAxleFront::m_springRestLength = .7;//0.3948;

// -----------------------------------------------------------------------------

const double CityBus_SolidAxleRear::m_ULMass = 25.0;//1.446;
const double CityBus_SolidAxleRear::m_LLMass = 25.0;//2.892;
const double CityBus_SolidAxleRear::m_knuckleMass = 15.0;//1.356;
const double CityBus_SolidAxleRear::m_spindleMass = 5.0;//0.248;
const double CityBus_SolidAxleRear::m_axleTubeMass = 100.0;//44.958;
const double CityBus_SolidAxleRear::m_tierodMass = 5.0;//1.633;
const double CityBus_SolidAxleRear::m_draglinkMass = 2.5;//0.464;
const double CityBus_SolidAxleRear::m_bellCrankMass = 2.5;//0.218;

const double CityBus_SolidAxleRear::m_spindleRadius = 0.10;
const double CityBus_SolidAxleRear::m_spindleWidth = 0.05;
const double CityBus_SolidAxleRear::m_ULRadius = 0.02;
const double CityBus_SolidAxleRear::m_LLRadius = 0.02;
const double CityBus_SolidAxleRear::m_axleTubeRadius = 0.10;
const double CityBus_SolidAxleRear::m_knuckleRadius = 0.05;
const double CityBus_SolidAxleRear::m_tierodRadius = 0.01;
const double CityBus_SolidAxleRear::m_draglinkRadius = 0.01;
const double CityBus_SolidAxleRear::m_bellCrankRadius = 0.01;

const ChVector<> CityBus_SolidAxleRear::m_axleTubeCOM(0, 0, 0);

const ChVector<> CityBus_SolidAxleRear::m_axleTubeInertia(52.333, .5, 52.333);//(7.744, 0.045, 7.744);
const ChVector<> CityBus_SolidAxleRear::m_spindleInertia(.0135, .025, .0135);//(0.0558, 0.0279, 0.0558);
const ChVector<> CityBus_SolidAxleRear::m_ULInertia(1.023, .005, 1.023);//(0.011, 0.011, 0.00142);
const ChVector<> CityBus_SolidAxleRear::m_LLInertia(1.023, .005, 1.023);//(0.0514, 0.0514, 0.0037);
const ChVector<> CityBus_SolidAxleRear::m_knuckleInertia(.0219, .019, .0219);//(0.0255, 0.0134, 0.0196);
const ChVector<> CityBus_SolidAxleRear::m_tierodInertia(2.604, .00025, 2.604);//(0.252, 0.01, 0.252);
const ChVector<> CityBus_SolidAxleRear::m_draglinkInertia(.05,.0001,.05);//(0.005, 0.005, 0.001);
const ChVector<> CityBus_SolidAxleRear::m_bellCrankInertia(.05,.0001,.05);//(0.001, 0.001, 0.001);

const double CityBus_SolidAxleRear::m_axleInertia = 0.4;

const double CityBus_SolidAxleRear::m_springCoefficient = 900000; //26706.20;
const double CityBus_SolidAxleRear::m_dampingCoefficient = 35000;//22459.0;
const double CityBus_SolidAxleRear::m_springRestLength = 0.7;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
CityBus_SolidAxleFront::CityBus_SolidAxleFront(const std::string& name) : ChSolidAxle(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient);
    m_shockForceCB = new LinearDamperForce(m_dampingCoefficient);
}

CityBus_SolidAxleRear::CityBus_SolidAxleRear(const std::string& name) : ChSolidAxle(name) {
    m_springForceCB = new LinearSpringForce(m_springCoefficient);
    m_shockForceCB = new LinearDamperForce(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
CityBus_SolidAxleFront::~CityBus_SolidAxleFront() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

CityBus_SolidAxleRear::~CityBus_SolidAxleRear() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> CityBus_SolidAxleFront::getLocation(PointId which) {
    switch (which) {
        case SHOCK_A:
            return ChVector<>(-0.2, 0.55, -0.15);
        case SHOCK_C:
            return ChVector<>(-0.2, 0.5, 0.45);
        case KNUCKLE_L:
            return ChVector<>(0, 1.1, -0.145);
        case KNUCKLE_U:
            return ChVector<>(0, 1.08, 0.145);
        case LL_A:
            return ChVector<>(0.09, 0.8, -0.075);
        case LL_C:
            return ChVector<>(0.64, 0.6, -0.075);
        case UL_A:
            return ChVector<>(0.08, 0.275, 0.325);
        case UL_C:
            return ChVector<>(0.62, 0.5, 0.325);
        case SPRING_A:
            return ChVector<>(-0.065, 0.575, -0.025);
        case SPRING_C:
            return ChVector<>(-0.080, 0.55, 0.5);
        case TIEROD_K:
            return ChVector<>(-0.3, 1, -0.05);
        case SPINDLE:
            return ChVector<>(0, 1.1275, 0);
        case KNUCKLE_CM:
            return ChVector<>(0, 1.09, 0);
        case LL_CM:
            return ChVector<>(0.365, 0.6, -0.075);
        case UL_CM:
            return ChVector<>(0.35, 0.4875, 0.325);
        case BELLCRANK_TIEROD:
            return ChVector<>(-0.3, -0.1, -0.05);
        case BELLCRANK_AXLE:
            return ChVector<>(0, 0, -0.05);
        case BELLCRANK_DRAGLINK:
            return ChVector<>(0, 0.2, -0.05);
        case DRAGLINK_C:
            return ChVector<>(0.4125, -.14, -.15);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> CityBus_SolidAxleRear::getLocation(PointId which) {
    switch (which) {
        case SHOCK_A:
            return ChVector<>(-0.2, 0.55, -0.15);
        case SHOCK_C:
            return ChVector<>(-0.2, 0.5, 0.45);
        case KNUCKLE_L:
            return ChVector<>(0, 1.1, -0.145);
        case KNUCKLE_U:
            return ChVector<>(0, 1.08, 0.145);
        case LL_A:
            return ChVector<>(0.09, 0.6, -0.075);
        case LL_C:
            return ChVector<>(0.64, 0.6, -0.075);
        case UL_A:
            return ChVector<>(0.08, 0.475, 0.325);
        case UL_C:
            return ChVector<>(0.62, 0.5, 0.325);
        case SPRING_A:
            return ChVector<>(-0.065, 0.575, -0.025);
        case SPRING_C:
            return ChVector<>(-0.080, 0.55, 0.5);
        case TIEROD_K:
            return ChVector<>(-0.3, 1, -0.05);
        case SPINDLE:
            return ChVector<>(0, 1.1275, 0);
        case KNUCKLE_CM:
            return ChVector<>(0, 1.09, 0);
        case LL_CM:
            return ChVector<>(0.365, 0.6, -0.075);
        case UL_CM:
            return ChVector<>(0.35, 0.4875, 0.325);
        case BELLCRANK_TIEROD:
            return ChVector<>(-0.3, 0, -0.05);
        case BELLCRANK_AXLE:
            return ChVector<>(-.5, 0, -0.05);
        case BELLCRANK_DRAGLINK:
            return ChVector<>(-.5, 0.2, -0.05);
        case DRAGLINK_C:
            return ChVector<>(0.4125, -0.2, -.15);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
