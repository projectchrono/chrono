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


#include "chrono/fea/ChLoaderHeatVolumetricSource.h"

namespace chrono {
namespace fea {


inline void ChLoaderHeatVolumetricSource::ComputeF(double U, double V, double W, ChVectorDynamic<>& F, ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    F(0) = m_heat_flux;
}

}  // end namespace fea
}  // end namespace chrono
