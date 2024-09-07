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


#include "chrono_multidomain/ChSystemMultidomain.h"

namespace chrono {
namespace multidomain {


// Increment a vector R with the term c*F:
//    R += c*F
void ChSystemNSCmultidomain::LoadResidual_F(ChVectorDynamic<>& R, const double c) {
    
    ChSystemNSC::LoadResidual_F(R, c);

}

// Increment a vector R with a term that has M multiplied a given vector w:
//    R += c*M*w
void ChSystemNSCmultidomain::LoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {

    ChSystemNSC::LoadResidual_Mv(R, w, c);

}

}  // end namespace multidomain
}  // end namespace chrono
