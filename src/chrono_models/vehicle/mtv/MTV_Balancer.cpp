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
// Authors: Radu Serban
// =============================================================================
//
// MTV balancer subsystem (installed on rear chassis)
//
// =============================================================================

#include "chrono_models/vehicle/mtv/MTV_Balancer.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables
const double MTV_Balancer::m_beam_max_pitch = 8.0 * CH_C_DEG_TO_RAD;
const double MTV_Balancer::m_beam_mass = 50.0;
const ChVector<> MTV_Balancer::m_beam_inertia(0.052, 8.177, 8.208);
const ChVector<> MTV_Balancer::m_beam_dimensions(1.4, 0.1, 0.05);

MTV_Balancer::MTV_Balancer(const std::string& name) : ChBalancer(name) {}

const ChVector<> MTV_Balancer::GetLocation(PointId which) {
    switch (which) {
        case BEAM:
            return ChVector<>(0.0, 0.529, 0.0);
        case REVOLUTE:
            return ChVector<>(0.0, 0.529, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
