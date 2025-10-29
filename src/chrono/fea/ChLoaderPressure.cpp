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


#include "chrono/fea/ChLoaderPressure.h"

namespace chrono {
namespace fea {


inline void ChLoaderPressure::ComputeF(double U, double V, ChVectorDynamic<>& F, ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    ChVector3d mnorm = this->loadable->ComputeNormal(U, V);
    F.segment(0, 3) = -pressure * mnorm.eigen();
}


}  // end namespace fea
}  // end namespace chrono
