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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChElementHexa_20.h"

namespace chrono {
namespace fea {

ChElementHexa_20::ChElementHexa_20() {
    nodes.resize(20);
    this->StiffnessMatrix.Resize(60, 60);
    this->ir = new ChGaussIntegrationRule;
    this->SetDefaultIntegrationRule();
}

ChElementHexa_20::~ChElementHexa_20() {}

}  // end namespace fea
}  // end namespace chrono
