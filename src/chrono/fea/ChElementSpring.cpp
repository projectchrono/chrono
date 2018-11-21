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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChElementSpring.h"

namespace chrono {
namespace fea {

ChElementSpring::ChElementSpring() {
    spring_k = 1.0;
    damper_r = 0.01;

    nodes.resize(2);
}

ChElementSpring::~ChElementSpring() {}

}  // end namespace fea
}  // end namespace chrono
