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

#include "chrono/fea/ChContactSurface.h"

namespace chrono {
namespace fea {

ChContactSurface::ChContactSurface(std::shared_ptr<ChMaterialSurface> material, ChPhysicsItem* physics_item)
    : m_material(material), m_physics_item(physics_item) {}

}  // end namespace fea
}  // end namespace chrono
