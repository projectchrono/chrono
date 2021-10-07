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
// 1st rear Solid 3-link axle subsystem for the MTV vehicle.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/MTV_Solid3LinkAxle1.h"
#include "chrono_models/vehicle/mtv/MTV_SpringDamper.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MTV_Solid3LinkAxle1::m_axleTubeMass = 709;
const double MTV_Solid3LinkAxle1::m_spindleMass = 14.705 * 4.1;
const double MTV_Solid3LinkAxle1::m_triangleMass = 50.0;
const double MTV_Solid3LinkAxle1::m_linkMass = 25.0;

const double MTV_Solid3LinkAxle1::m_axleTubeRadius = 0.0476;
const double MTV_Solid3LinkAxle1::m_spindleRadius = 0.10;
const double MTV_Solid3LinkAxle1::m_spindleWidth = 0.06;

const ChVector<> MTV_Solid3LinkAxle1::m_axleTubeInertia(329.00, 16.46, 330.00);
const ChVector<> MTV_Solid3LinkAxle1::m_spindleInertia(0.04117 * 6.56, 0.07352 * 6.56, 0.04117 * 6.56);
const ChVector<> MTV_Solid3LinkAxle1::m_triangleInertia(0.2, 0.2, 0.2);
const ChVector<> MTV_Solid3LinkAxle1::m_linkInertia(0.05, 0.1, 0.1);

const double MTV_Solid3LinkAxle1::m_axleShaftInertia = 0.4;

const double MTV_Solid3LinkAxle1::m_springDesignLength = 0.3;
const double MTV_Solid3LinkAxle1::m_springCoefficient = 367000.0;
const double MTV_Solid3LinkAxle1::m_springRestLength = m_springDesignLength + 0.06;
const double MTV_Solid3LinkAxle1::m_springMinLength = m_springDesignLength - 0.08;
const double MTV_Solid3LinkAxle1::m_springMaxLength = m_springDesignLength + 0.08;
const double MTV_Solid3LinkAxle1::m_damperCoefficient = 41300.0;
const double MTV_Solid3LinkAxle1::m_damperDegressivityCompression = 3.0;
const double MTV_Solid3LinkAxle1::m_damperDegressivityExpansion = 1.0;

// -----------------------------------------------------------------------------

MTV_Solid3LinkAxle1::MTV_Solid3LinkAxle1(const std::string& name) : ChSolidThreeLinkAxle(name) {
    m_springForceCB =
        chrono_types::make_shared<MTV_SpringForceRear>(m_springCoefficient, m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<MTV_ShockForceRear>(
        m_damperCoefficient, m_damperDegressivityCompression, m_damperCoefficient, m_damperDegressivityExpansion);
}

const ChVector<> MTV_Solid3LinkAxle1::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.529, 0.1);
        case SPRING_C:
            return ChVector<>(0.0, 0.529, 0.4);
        case SHOCK_A:
            return ChVector<>(0.1, 0.529, -0.1);
        case SHOCK_C:
            return ChVector<>(0.2, 0.529, 0.570);
        case SPINDLE:
            return ChVector<>(0.0, 2.07 / 2.0, 0.0);
        case TRIANGLE_A:
            return ChVector<>(0.0, 0.0, 0.260);
        case TRIANGLE_C:
            return ChVector<>(-0.50, 0.30, 0.260);
        case LINK_A:
            return ChVector<>(0.0, 0.420, -0.090);
        case LINK_C:
            return ChVector<>(-0.50, 0.420, -0.090);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
