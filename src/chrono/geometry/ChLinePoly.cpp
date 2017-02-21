// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
namespace geometry {

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

ChVector<> ChLinePoly::Get_point(size_t mnum) const {
    if (mnum >= Get_numpoints())
        return VNULL;

    return points[mnum];
}

bool ChLinePoly::Set_point(int mnum, ChVector<> mpoint) {
    if (mnum >= Get_numpoints())
        return false;

    this->points[mnum] = mpoint;

    return true;
}

//
// Curve evaluation.
//

void ChLinePoly::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    double par = parU;
    pos = VNULL;

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
    pos = Vadd(Vmul(Get_point(pA), 1 - (epar - (double)pA)), Vmul(Get_point(pB), epar - (double)pA));
}

double ChLinePoly::Length(int sampling) const {
    return ChLine::Length(1);
}

// Draw into the current graph viewport of a ChFile_ps file

bool ChLinePoly::DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate) {
    ChVector2<> mp1;
    ChVector<> mv1;

    mfle->GrSave();
    mfle->ClipRectangle(mfle->Get_G_p(), mfle->Get_Gs_p(), ChFile_ps::Space::PAGE);
    // start a line, move cursor to beginning
    mfle->StartLine();
    mp1.x() = Get_point(0).x();
    mp1.y() = Get_point(0).y();
    mp1 = mfle->To_page_from_graph(mp1);
    mfle->MoveTo(mp1);
    // add points into line
    for (int i = 1; i < this->Get_numpoints(); i++) {
        mv1 = Get_point(i);
        mp1.x() = mv1.x();
        mp1.y() = mv1.y();
        mp1 = mfle->To_page_from_graph(mp1);
        mfle->AddLinePoint(mp1);
    }
    if (this->Get_closed())
        mfle->CloseLine();  // if periodic curve, close it

    mfle->PaintStroke();  // draw it!
    mfle->GrRestore();    // restore old modes, with old clipping

    return true;
}

}  // end namespace geometry
}  // end namespace chrono
