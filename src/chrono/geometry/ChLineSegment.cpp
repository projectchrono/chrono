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

#include "chrono/geometry/ChLineSegment.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineSegment)

ChLineSegment::ChLineSegment(const ChLineSegment& source) : ChLine(source) {
    pA = source.pA;
    pB = source.pB;
}

ChFrame<> ChLineSegment::GetFrame() const {
    ChVector<> dir = (pB - pA).GetNormalized();
    ChVector<> u, v, w;
    dir.DirToDxDyDz(w, u, v);

    return ChFrame<>(0.5 * (pB + pA), ChMatrix33<>(u, v, w));
}

void ChLineSegment::Evaluate(ChVector<>& pos, const double parU) const {
    pos = pA * (1 - parU) + pB * parU;
}

void ChLineSegment::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLineSegment>();
    // serialize parent class
    ChLine::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(pA);
    marchive << CHNVP(pB);
}

void ChLineSegment::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLineSegment>();
    // deserialize parent class
    ChLine::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(pA);
    marchive >> CHNVP(pB);
}

}  // end namespace geometry
}  // end namespace chrono
