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

#include "chrono_fea/ChElementTetra_10.h"

namespace chrono {
namespace fea {

ChElementTetra_10::ChElementTetra_10() {
    nodes.resize(10);
    MatrB.resize(4);  // standard: 4 integration points
    MatrB[0].Resize(6, 30);
    MatrB[1].Resize(6, 30);
    MatrB[2].Resize(6, 30);
    MatrB[3].Resize(6, 30);
    this->StiffnessMatrix.Resize(30, 30);
}

ChElementTetra_10::~ChElementTetra_10() {}

}  // end namespace fea
}  // end namespace chrono
