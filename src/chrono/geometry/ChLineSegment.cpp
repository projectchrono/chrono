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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/geometry/ChLineSegment.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineSegment)

ChLineSegment::ChLineSegment(const ChLineSegment& source) : ChLine(source) {
    pA = source.pA;
    pB = source.pB;
}

void ChLineSegment::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    pos = pA * (1 - parU) + pB * parU;
}

}  // end namespace geometry
}  // end namespace chrono
