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

#include "chrono/geometry/ChLineArc.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineArc)

ChLineArc::ChLineArc(const ChCoordsys<>& morigin,
                     double mradius,
                     double mangle1,
                     double mangle2,
                     bool mcounterclockwise)
    : origin(morigin), radius(mradius), angle1(mangle1), angle2(mangle2), counterclockwise(mcounterclockwise) {}

ChLineArc::ChLineArc(const ChLineArc& source) : ChLine(source) {
    origin = source.origin;
    radius = source.radius;
    angle1 = source.angle1;
    angle2 = source.angle2;
    counterclockwise = source.counterclockwise;
}

ChVector<> ChLineArc::Evaluate(double parU) const {
    double ang1 = this->angle1;
    double ang2 = this->angle2;
    if (this->counterclockwise) {
        if (ang2 < ang1)
            ang2 += CH_C_2PI;
    } else {
        if (ang2 > ang1)
            ang2 -= CH_C_2PI;
    }
    double mangle = ang1 * (1 - parU) + ang2 * (parU);
    ChVector<> localP(radius * cos(mangle), radius * sin(mangle), 0);
    return localP >> origin;  // transform to absolute coordinates
}

void ChLineArc::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLineArc>();
    // serialize parent class
    ChLine::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(origin);
    marchive << CHNVP(radius);
    marchive << CHNVP(angle1);
    marchive << CHNVP(angle2);
    marchive << CHNVP(counterclockwise);
}

void ChLineArc::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLineArc>();
    // deserialize parent class
    ChLine::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(origin);
    marchive >> CHNVP(radius);
    marchive >> CHNVP(angle1);
    marchive >> CHNVP(angle2);
    marchive >> CHNVP(counterclockwise);
}

}  // end namespace geometry
}  // end namespace chrono
