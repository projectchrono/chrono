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

#include "chrono/geometry/ChLinePoly.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinePoly)

ChLinePoly::ChLinePoly(int mnumpoints) : degree(1) {
    points.resize(mnumpoints);
}

ChLinePoly::ChLinePoly(const ChLinePoly& source) : ChLine(source) {
    points = source.points;
    degree = source.degree;
}

size_t ChLinePoly::GetNumPoints() const {
    return points.size();
}

int ChLinePoly::GetDegree() const {
    return degree;
}

ChVector3d ChLinePoly::GetPoint(size_t mnum) const {
    if (mnum >= GetNumPoints())
        return VNULL;

    return points[mnum];
}

bool ChLinePoly::SetPoint(int mnum, const ChVector3d& mpoint) {
    if (mnum >= GetNumPoints())
        return false;

    this->points[mnum] = mpoint;

    return true;
}

//
// Curve evaluation.
//

ChVector3d ChLinePoly::Evaluate(double parU) const {
    double par = parU;

    if (par < 0)
        par = 0;
    if (par > 1)
        par = 1;
    size_t pA = 0;
    size_t pB = 0;
    double epar;
    if (!closed)
        epar = par * (GetNumPoints() - 1);
    else
        epar = par * GetNumPoints();
    pA = (size_t)floor(epar);
    pB = (size_t)ceil(epar);

    if (pA >= (GetNumPoints() - 1))
        pA = (GetNumPoints() - 1);
    if (pB >= GetNumPoints()) {
        if (!closed)
            pB = (GetNumPoints() - 1);
        else
            pB = 0;
    }
    // linear interpolation
    return Vadd(Vmul(GetPoint(pA), 1 - (epar - (double)pA)), Vmul(GetPoint(pB), epar - (double)pA));
}

double ChLinePoly::Length(int sampling) const {
    return ChLine::Length(1);
}

void ChLinePoly::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinePoly>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    archive_out << CHNVP(degree);
}

void ChLinePoly::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinePoly>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    archive_in >> CHNVP(degree);
}

}  // end namespace chrono
