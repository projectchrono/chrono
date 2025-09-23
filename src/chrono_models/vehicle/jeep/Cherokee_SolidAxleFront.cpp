// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Classes for modeling the front suspension of Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
// The steering mechanism is slightly different from the SAE Paper, since
// the chrono class ChSolidAxle has a bellcrank and the Jeep Cherokee has not.
//
// =============================================================================

#include "Cherokee_SolidAxleFront.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_SolidAxleFront::m_axleTubeMass = 96.79661176;
const double Cherokee_SolidAxleFront::m_spindleMass = 20.0;
const double Cherokee_SolidAxleFront::m_trackbarMass = 4.263768278;
const double Cherokee_SolidAxleFront::m_knuckleMass = 10.0;
const double Cherokee_SolidAxleFront::m_tierodMass = 2.72155422;
const double Cherokee_SolidAxleFront::m_dragLinkMass = 4.67;
const double Cherokee_SolidAxleFront::m_ulMass = 1.9;
const double Cherokee_SolidAxleFront::m_llMass = 2.08;
const double Cherokee_SolidAxleFront::m_bellCrankMass = 10.0;

const double Cherokee_SolidAxleFront::m_axleTubeRadius = 0.03175;
const double Cherokee_SolidAxleFront::m_spindleRadius = 0.07;
const double Cherokee_SolidAxleFront::m_spindleWidth = 0.04;
const double Cherokee_SolidAxleFront::m_trackbarRadius = 0.015;
const double Cherokee_SolidAxleFront::m_knuckleRadius = 0.02;
const double Cherokee_SolidAxleFront::m_dragLinkRadius = 0.015;
const double Cherokee_SolidAxleFront::m_tierodRadius = 0.01;
const double Cherokee_SolidAxleFront::m_ulRadius = 0.02;
const double Cherokee_SolidAxleFront::m_llRadius = 0.02;
const double Cherokee_SolidAxleFront::m_bellCrankRadius = 0.02;

const ChVector3d Cherokee_SolidAxleFront::m_axleTubeInertia(24.32563394, 0.562664454, 24.32563394);
const ChVector3d Cherokee_SolidAxleFront::m_spindleInertia(0.027166667, 0.049, 0.027166667);
const ChVector3d Cherokee_SolidAxleFront::m_trackbarInertia(0.282462076, 0.001129848, 0.282462076);
const ChVector3d Cherokee_SolidAxleFront::m_knuckleInertia(0.1, 0.1, 0.1);
const ChVector3d Cherokee_SolidAxleFront::m_tierodInertia(0.384148423, 0.001129848, 0.384148423);
const ChVector3d Cherokee_SolidAxleFront::m_dragLinkInertia(0.394317057, 0.001129848, 0.394317057);
const ChVector3d Cherokee_SolidAxleFront::m_ulInertia(0.001129848, 0.056492415, 0.056492415);
const ChVector3d Cherokee_SolidAxleFront::m_llInertia(0.001129848, 0.056492415, 0.056492415);
const ChVector3d Cherokee_SolidAxleFront::m_bellCrankInertia(0.1, 0.1, 0.1);

const double Cherokee_SolidAxleFront::m_springDesignLength = 0.288713097;
const double Cherokee_SolidAxleFront::m_springPreload = 4527;
const double Cherokee_SolidAxleFront::m_springCoefficient = 25000.0;
const double Cherokee_SolidAxleFront::m_springRestLength = m_springDesignLength;
const double Cherokee_SolidAxleFront::m_springMinLength = m_springDesignLength - 0.04;
const double Cherokee_SolidAxleFront::m_springMaxLength = m_springDesignLength + 0.04;
const double Cherokee_SolidAxleFront::m_damperCoefficientExpansion = 9704.555729;
const double Cherokee_SolidAxleFront::m_damperCoefficientCompression = 4528.053701;
const double Cherokee_SolidAxleFront::m_damperDegressivityCompression = 3.0;
const double Cherokee_SolidAxleFront::m_damperDegressivityExpansion = 1.0;
const double Cherokee_SolidAxleFront::m_axleShaftInertia = 0.4;

Cherokee_SolidAxleFront::Cherokee_SolidAxleFront(const std::string& name) : ChSolidAxle(name) {
    m_springForceCB = chrono_types::make_shared<utils::LinearSpringForce>(m_springCoefficient, m_springPreload);
    auto ptr = std::static_pointer_cast<utils::LinearSpringForce>(m_springForceCB);
    ptr->enable_stops(m_springMinLength, m_springMaxLength);
    ptr->set_stops(2.0 * m_springCoefficient, 2.0 * m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<utils::DegressiveDamperForce>(
        m_damperCoefficientCompression, m_damperDegressivityCompression, m_damperCoefficientExpansion,
        m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Cherokee_SolidAxleFront::~Cherokee_SolidAxleFront() {}

const ChVector3d Cherokee_SolidAxleFront::getLocation(PointId which) {
    switch (which) {
        case BELLCRANK_TIEROD:
            return ChVector3d(0.14859, 0, 0.021082);
        case BELLCRANK_AXLE:
            return ChVector3d(0, 0, -1.2 * m_axleTubeRadius);
        case BELLCRANK_DRAGLINK:
            return ChVector3d(-0.2, 0, m_axleTubeRadius);
        case DRAGLINK_C:
            return ChVector3d(-0.20, 0.5, m_axleTubeRadius);
        case TIEROD_K:
            return ChVector3d(0.14859, 0.6604, 0.021082);
        case KNUCKLE_CM:
            return ChVector3d(-0.000889, 0.59309, 0.042672);
        case KNUCKLE_U:
            return ChVector3d(-0.015748, 0.57277, 0.145542);
        case KNUCKLE_L:
            return ChVector3d(0.01397, 0.61341, -0.060198);
        case LL_A:
            return ChVector3d(-0.061468, 0.41402, -0.058928);
        case LL_C:
            return ChVector3d(-0.45974, 0.38735, -0.039878);
        case LL_CM:
            return ChVector3d(-0.260604, 0.400685, -0.049403);
        case UL_A:
            return ChVector3d(0.00762, 0.23876, 0.14351);
        case UL_C:
            return ChVector3d(-0.3683, 0.33782, 0.144272);
        case UL_CM:
            return ChVector3d(-0.18034, 0.28829, 0.143891);
        case SHOCK_A:
            return ChVector3d(-0.087376, 0.45212, 0.029972);
        case SHOCK_C:
            return ChVector3d(-0.1143, 0.58801, 0.464312);
        case SPRING_A:
            return ChVector3d(0.036322, 0.45466, 0.085852);
        case SPRING_C:
            return ChVector3d(0.01651, 0.47879, 0.372872);
        case SPINDLE:
            return ChVector3d(0.0, 0.74803, 0.0);
        case TRACKBAR_A:
            return ChVector3d(0.09144, -0.45974, 0.005842);
        case TRACKBAR_C:
            return ChVector3d(0.086868, 0.33274, 0.080772);
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
