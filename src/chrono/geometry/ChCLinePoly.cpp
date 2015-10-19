//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChCLinePoly.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCLinePoly.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinePoly> a_registration_ChLinePoly;

//
// CLASS FOR POLY LINE
//
// The object which represent 3d lines/splines
//

ChLinePoly::ChLinePoly(int mnumpoints) {
    closed = 0;
    points.resize(mnumpoints);
    int degree = 1;
}

ChLinePoly::~ChLinePoly() {
}

void ChLinePoly::Copy(const ChLinePoly* source) {
    // Copy parent data;
    ChLine::Copy(source);

    // Copy custom data;
    points = source->points;
    degree = source->degree;
}

bool ChLinePoly::Get_closed() {
    return closed;
}

void ChLinePoly::Set_closed(bool mc) {
    closed = mc;
}

size_t ChLinePoly::Get_numpoints() {
    return points.size();
}

int ChLinePoly::Get_degree() {
    return degree;
}

Vector ChLinePoly::Get_point(size_t mnum) {
    if (mnum >= Get_numpoints())
        return VNULL;

    return this->points[mnum];
}

int ChLinePoly::Set_point(int mnum, Vector mpoint) {
    if (mnum >= Get_numpoints())
        return FALSE;

    this->points[mnum] = mpoint;

    return TRUE;
}

//
// Curve evaluation.
//

void ChLinePoly::Evaluate(Vector& pos, const double parU, const double parV, const double parW) {
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

double ChLinePoly::Length(int sampling) {
    return ChLine::Length(1);
}

// Draw into the current graph viewport of a ChFile_ps file

int ChLinePoly::DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate) {
    ChPageVect mp1;
    Vector mv1;

    mfle->GrSave();
    mfle->ClipRectangle(mfle->Get_G_p(), mfle->Get_Gs_p(), PS_SPACE_PAGE);
    // start a line, move cursor to beginning
    mfle->StartLine();
    mp1.x = Get_point(0).x;
    mp1.y = Get_point(0).y;
    mp1 = mfle->To_page_from_graph(mp1);
    mfle->MoveTo(mp1);
    // add points into line
    for (int i = 1; i < this->Get_numpoints(); i++) {
        mv1 = Get_point(i);
        mp1.x = mv1.x;
        mp1.y = mv1.y;
        mp1 = mfle->To_page_from_graph(mp1);
        mfle->AddLinePoint(mp1);
    }
    if (this->Get_closed())
        mfle->CloseLine();  // if periodic curve, close it

    mfle->PaintStroke();  // draw it!
    mfle->GrRestore();    // restore old modes, with old clipping

    return TRUE;
}



}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

////// end
