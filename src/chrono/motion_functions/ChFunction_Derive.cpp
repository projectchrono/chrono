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

#include "chrono/motion_functions/ChFunction_Derive.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Derive)

ChFunction_Derive::ChFunction_Derive(const ChFunction_Derive& other) {
    order = other.order;
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
}

double ChFunction_Derive::Get_y(double x) const {
    return fa->Get_y_dx(x);
}

void ChFunction_Derive::Estimate_x_range(double& xmin, double& xmax) const {
    fa->Estimate_x_range(xmin, xmax);
}

void ChFunction_Derive::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Derive>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fa);
    marchive << CHNVP(order);
}

void ChFunction_Derive::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Derive>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(fa);
    marchive >> CHNVP(order);
}

}  // end namespace chrono
