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

#include <cmath>

#include "chrono/fea/ChFeaMaterial.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
/*
 ChFeaMaterialModel::ChFeaMaterialModel(const ChFeaMaterial& other) {
    //m_density = other.m_density;
}
*/

// Register into the object factory, to enable run-time dynamic creation and persistence
    /*
CH_FACTORY_REGISTER(ChFea3DContinuum)

void ChFea3DContinuum::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFeaMaterial>();
    // serialize parent class
    // serialize all member data:
    //archive_out << CHNVP(m_density);
}

void ChFea3DContinuum::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    int version = archive_in.VersionRead<ChFeaMaterial>();
    // deserialize parent class
    // stream in all member data:
    ///archive_in >> CHNVP(m_density);
}
*/



}  // end namespace fea
}  // end namespace chrono
