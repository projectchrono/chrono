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
// Authors: Radu Serban
// =============================================================================
//
// Base class for all vehicle subsystems.
//
// =============================================================================

#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPart::ChPart(const std::string& name)
    : m_name(name),
      m_friction(0.7f),
      m_restitution(0.1f),
      m_young_modulus(1e7f),
      m_poisson_ratio(0.3f),
      m_kn(2e6),
      m_kt(2e5),
      m_gn(40),
      m_gt(20) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPart::SetContactMaterialProperties(float young_modulus, float poisson_ratio) {
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void ChPart::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPart::SetVisualizationType(VisualizationType vis) {
    RemoveVisualizationAssets();
    AddVisualizationAssets(vis);
}

}  // end namespace vehicle
}  // end namespace chrono
