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

size_t ChLinePoly::Get_numpoints() const {
    return points.size();
}

int ChLinePoly::Get_degree() const {
    return degree;
}

ChVector3d ChLinePoly::Get_point(size_t mnum) const {
    if (mnum >= Get_numpoints())
        return VNULL;

    return points[mnum];
}

bool ChLinePoly::Set_point(int mnum, ChVector3d mpoint) {
    if (mnum >= Get_numpoints())
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
        epar = par * (Get_numpoints() - 1);
    else
        epar = par * Get_numpoints();
    pA = (size_t)floor(epar);
    pB = (size_t)ceil(epar);

    if (pA >= (Get_numpoints() - 1))
        pA = (Get_numpoints() - 1);
    if (pB >= Get_numpoints()) {
        if (!closed)
            pB = (Get_numpoints() - 1);
        else
            pB = 0;
    }
    // linear interpolation
    return Vadd(Vmul(Get_point(pA), 1 - (epar - (double)pA)), Vmul(Get_point(pB), epar - (double)pA));
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
    /*int version =*/ archive_in.VersionRead<ChLinePoly>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    archive_in >> CHNVP(degree);
}


}  // end namespace chrono
