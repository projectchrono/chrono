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
#include "chrono_models/vehicle/mtv/MTV_SpringDamper.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

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

MTV_LeafspringAxle1::MTV_LeafspringAxle1(const std::string& name) : ChLeafspringAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<MTV_SpringForceRear>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<MTV_ShockForceRear>(
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
