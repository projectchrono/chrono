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

#include <cmath>

#include "chrono/geometry/ChLineArc.h"

namespace chrono {

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

ChVector3d ChLineArc::Evaluate(double parU) const {
    double ang1 = this->angle1;
    double ang2 = this->angle2;
    if (this->counterclockwise) {
        if (ang2 < ang1)
            ang2 += CH_2PI;
    } else {
        if (ang2 > ang1)
            ang2 -= CH_2PI;
    }
    double mangle = ang1 * (1 - parU) + ang2 * (parU);
    ChVector3d localP(radius * std::cos(mangle), radius * std::sin(mangle), 0);
    return localP >> origin;  // transform to absolute coordinates
}

void ChLineArc::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineArc>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(origin);
    archive_out << CHNVP(radius);
    archive_out << CHNVP(angle1);
    archive_out << CHNVP(angle2);
    archive_out << CHNVP(counterclockwise);
}

void ChLineArc::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineArc>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(origin);
    archive_in >> CHNVP(radius);
    archive_in >> CHNVP(angle1);
    archive_in >> CHNVP(angle2);
    archive_in >> CHNVP(counterclockwise);
}

}  // end namespace chrono
