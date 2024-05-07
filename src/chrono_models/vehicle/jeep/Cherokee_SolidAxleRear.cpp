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
// Classes for modeling the rear suspension of Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
// The rear suspension is simplified to the chrono ChLeafspring model.
//
// =============================================================================

#include "Cherokee_SolidAxleRear.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_SolidAxleRear::m_axleTubeMass = 70.03466193;
const double Cherokee_SolidAxleRear::m_spindleMass = 20.0;

const double Cherokee_SolidAxleRear::m_axleTubeRadius = 0.033401;
const double Cherokee_SolidAxleRear::m_spindleRadius = 0.07;
const double Cherokee_SolidAxleRear::m_spindleWidth = 0.04;

const ChVector3d Cherokee_SolidAxleRear::m_axleTubeInertia(12.74468885, 0.406745389, 12.74468885);
const ChVector3d Cherokee_SolidAxleRear::m_spindleInertia(0.027166667, 0.049, 0.027166667);

const double Cherokee_SolidAxleRear::m_springDesignLength = 0.2;
const double Cherokee_SolidAxleRear::m_springPreload = 3629;
const double Cherokee_SolidAxleRear::m_springCoefficient = 25000.0;
const double Cherokee_SolidAxleRear::m_springRestLength = m_springDesignLength;
const double Cherokee_SolidAxleRear::m_springMinLength = m_springDesignLength - 0.04;
const double Cherokee_SolidAxleRear::m_springMaxLength = m_springDesignLength + 0.04;
const double Cherokee_SolidAxleRear::m_damperCoefficientExpansion = 8189.490177;
const double Cherokee_SolidAxleRear::m_damperCoefficientCompression = 3821.138478;
const double Cherokee_SolidAxleRear::m_damperDegressivityCompression = 3.0;
const double Cherokee_SolidAxleRear::m_damperDegressivityExpansion = 1.0;
const double Cherokee_SolidAxleRear::m_axleShaftInertia = 0.4;

Cherokee_SolidAxleRear::Cherokee_SolidAxleRear(const std::string& name) : ChLeafspringAxle(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient, m_springPreload);
    auto ptr = std::static_pointer_cast<LinearSpringForce>(m_springForceCB);
    ptr->enable_stops(m_springMinLength, m_springMaxLength);
    ptr->set_stops(2.0 * m_springCoefficient, 2.0 * m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(m_damperCoefficientCompression, m_damperDegressivityCompression,
                                                                      m_damperCoefficientExpansion, m_damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Cherokee_SolidAxleRear::~Cherokee_SolidAxleRear() {}

const ChVector3d Cherokee_SolidAxleRear::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector3d(0, 0.55245, 0.054864);
        case SPRING_C:
            return ChVector3d(0, 0.55245, 0.054864 + m_springDesignLength);
        case SHOCK_A:
            return ChVector3d(-0.09906, 0.507238, -0.093218);
        case SHOCK_C:
            return ChVector3d(-0.0889, 0.29337, 0.280162);
        case SPINDLE:
            return ChVector3d(0.0, 0.73787, 0.0);
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
