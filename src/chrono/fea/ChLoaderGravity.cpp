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


#include "chrono/fea/ChLoaderGravity.h"

namespace chrono {
namespace fea {



void ChLoaderGravity::ComputeF(double U,
    double V,
    double W,
    ChVectorDynamic<>& F,
    ChVectorDynamic<>* state_x,
    ChVectorDynamic<>* state_w
) {
    double point_density = this->override_loadable_density;
    if (override_loadable_density < 0) {
        point_density = loadable->GetDensity();
    }

    if ((F.size() == 3) || (F.size() == 6) || (F.size() == 9)) {
        // only for force or wrench fields
        F(0) = G_acc.x() * point_density;
        F(1) = G_acc.y() * point_density;
        F(2) = G_acc.z() * point_density;
    }
}



}  // end namespace fea
}  // end namespace chrono
