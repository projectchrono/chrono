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
// Authors: Andrea Favali
// =============================================================================

#include "chrono_fea/ChElementHexa_8.h"

namespace chrono {
namespace fea {

ChElementHexa_8::ChElementHexa_8() {
    nodes.resize(8);
    StiffnessMatrix.Resize(24, 24);
    this->ir = new ChGaussIntegrationRule;
    this->SetDefaultIntegrationRule();
}

ChElementHexa_8::~ChElementHexa_8() {}

}  // end namespace fea
}  // end namespace chrono
