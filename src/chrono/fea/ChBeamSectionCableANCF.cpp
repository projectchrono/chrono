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

#include "chrono/fea/ChBeamSectionCableANCF.h"

namespace chrono {
namespace fea {

ChBeamSectionCableANCF::ChBeamSectionCableANCF() : E(1e7), density(1000), rdamping(0.01) {
    SetDiameter(0.01);
}

void ChBeamSectionCableANCF::SetDiameter(double diameter) {
    this->Area = CH_PI * std::pow((0.5 * diameter), 2);
    this->I = (CH_PI / 4.0) * std::pow((0.5 * diameter), 4);

    this->SetDrawCircularRadius(diameter / 2);
}

}  // end namespace fea
}  // end namespace chrono
