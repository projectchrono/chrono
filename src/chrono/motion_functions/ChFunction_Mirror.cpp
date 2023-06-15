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

#include "chrono/motion_functions/ChFunction_Mirror.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Mirror)

ChFunction_Mirror::ChFunction_Mirror() : mirror_axis(0) {
    fa = chrono_types::make_shared<ChFunction_Const>();
}

ChFunction_Mirror::ChFunction_Mirror(const ChFunction_Mirror& other) {
    mirror_axis = other.mirror_axis;
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
}

double ChFunction_Mirror::Get_y(double x) const {
    if (x <= this->mirror_axis)
        return fa->Get_y(x);
    return fa->Get_y(2 * this->mirror_axis - x);
}

void ChFunction_Mirror::Estimate_x_range(double& xmin, double& xmax) const {
    fa->Estimate_x_range(xmin, xmax);
}

void ChFunction_Mirror::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Mirror>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fa);
    marchive << CHNVP(mirror_axis);
}

void ChFunction_Mirror::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Mirror>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(fa);
    marchive >> CHNVP(mirror_axis);
}

}  // end namespace chrono
