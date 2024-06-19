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

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineSegment)

ChLineSegment::ChLineSegment(const ChLineSegment& source) : ChLine(source) {
    pA = source.pA;
    pB = source.pB;
}

ChFrame<> ChLineSegment::GetFrame() const {
    ChVector3d dir = (pB - pA).GetNormalized();
    ChVector3d u, v, w;
    dir.GetDirectionAxesAsX(w, u, v);

    return ChFrame<>(0.5 * (pB + pA), ChMatrix33<>(u, v, w));
}

ChVector3d ChLineSegment::Evaluate(double parU) const {
    return pA * (1 - parU) + pB * parU;
}

void ChLineSegment::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineSegment>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(pA);
    archive_out << CHNVP(pB);
}

void ChLineSegment::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineSegment>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(pA);
    archive_in >> CHNVP(pB);
}

}  // end namespace chrono
