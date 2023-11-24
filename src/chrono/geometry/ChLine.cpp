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

#include <memory.h>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "chrono/geometry/ChLine.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLine)  // NO! Abstract class!

ChLine::ChLine(const ChLine& source) {
    closed = source.closed;
    complexityU = source.complexityU;
}

ChVector<> ChLine::GetTangent(double parU) const {
    double bdf = 10e-9;
    double uA = 0, uB = 0;

    if (parU > 0.5) {
        uB = parU;
        uA = parU - bdf;
    } else {
        uB = parU + bdf;
        uA = parU;
    }

    auto vA = Evaluate(uA);
    auto vB = Evaluate(uB);

    return (vB - vA) * (1 / bdf);
}

bool ChLine::FindNearestLinePoint(ChVector<>& point, double& resU, double approxU, double tol) const {
    double mu;
    int points = 20;
    bool closed = false;
    double bestU = 0;
    double bestdist = 9999999;
    double dist, d1, d2;
    int iters = 0;
    int maxiters = 11;

    ChVector<> vres, vp1, vp2;

    points = this->Get_complexity();
    closed = this->Get_closed();

    points = points * 4;  // double sampling along line.

    // first approximation
    for (int i = 0; i <= points; i++) {
        mu = (double)i / (double)points;
        vres = Evaluate(mu);
        dist = Vlength(Vsub(vres, point));
        if (dist < bestdist) {
            bestdist = dist;
            bestU = mu;
        }
    }
    // refine position with pseudo-NR
    double step = 1 / (double)points;
    double nrU = bestU;
    double u1, u2;

    u1 = nrU - step;
    if (u1 < 0) {
        if (!closed)
            u1 = 0;
        else
            u1 = u1 + 1;
    }
    vres = this->Evaluate(u1);
    d1 = Vlength(Vsub(vres, point));
    vp1 = vres;

    u2 = nrU + step;
    if (u2 > 1) {
        if (!closed)
            u2 = 1;
        else
            u2 = u2 - 1;
    }
    vres = this->Evaluate(u2);
    d2 = Vlength(Vsub(vres, point));
    vp2 = vres;

    while (true) {
        iters++;

        if (nrU < 0) {
            if (!closed)
                nrU = 0;
            else
                nrU = nrU + 1;
        }
        if (nrU > 1) {
            if (!closed)
                nrU = 1;
            else
                nrU = nrU - 1;
        }
        vres = this->Evaluate(nrU);
        dist = Vlength(Vsub(vres, point));

        bestU = nrU;

        if (d1 < d2) {
            u2 = nrU;
            d2 = dist;
            vp2 = vres;  // move point 2 to nrU
            step = step / 2;
            nrU = nrU - step;  // move nrU in middle
            if (d1 < dist)
                bestU = u1;
        } else {
            u1 = nrU;
            d1 = dist;
            vp1 = vres;  // move point 1 to nrU
            step = step / 2;
            nrU = nrU + step;  // move nrU in middle
            if (d2 < dist)
                bestU = u2;
        }

        if ((Vlength(Vsub(vp1, vp2)) <= tol) || (dist <= tol)) {
            resU = bestU;
            return true;
        }
        if (iters > maxiters) {
            resU = bestU;
            return false;
        }
    }

    resU = bestU;
    return true;
}

double ChLine::CurveCurveDist(ChLine* compline, int samples) const {
    double mres = 0;
    double par;
    // distances this<->compare_line
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = compline->Evaluate(par);
        this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = this->Evaluate(mpos);
        mres += Vlength(Vsub(ptA, ptB));
    }
    // ..and viceversa
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = this->Evaluate(par);
        compline->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = compline->Evaluate(mpos);
        mres += Vlength(Vsub(ptA, ptB));
    }

    return (mres / (samples * 2));
}

double ChLine::CurveCurveDistMax(ChLine* compline, int samples) const {
    double mres = 0;
    double par;
    double mdis;
    // distances this<->compare_line
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = compline->Evaluate(par);
        this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = this->Evaluate(mpos);
        mdis = Vlength(Vsub(ptA, ptB));
        if (mres < mdis)
            mres = mdis;
    }
    // ..and viceversa
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = this->Evaluate(par);
        compline->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = compline->Evaluate(mpos);
        mdis = Vlength(Vsub(ptA, ptB));
        if (mres < mdis)
            mres = mdis;
    }

    return mres;
}

double ChLine::CurveSegmentDist(ChLine* complinesegm, int samples) const {
    double mres = 0;
    double par;
    // distances this<->compare_line
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = complinesegm->Evaluate(par);
        this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = this->Evaluate(mpos);
        mres += Vlength(Vsub(ptA, ptB));
    }
    return (mres / samples);
}

double ChLine::CurveSegmentDistMax(ChLine* complinesegm, int samples) const {
    double mres = 0;
    double par;
    double mdis;
    // distances this<->compare_line
    for (par = 0; par < 1; par = par + 1 / ((double)samples)) {
        double mpos;
        ChVector<> ptB = complinesegm->Evaluate(par);
        this->FindNearestLinePoint(ptB, mpos, 0, 0.00002);
        ChVector<> ptA = this->Evaluate(mpos);
        mdis = Vlength(Vsub(ptA, ptB));
        if (mres < mdis)
            mres = mdis;
    }
    return (mres);
}

double ChLine::Length(int sampling) const {
    double mres = 0;
    double par, step;

    if (!closed)
        step = 1 / ((double)(Get_complexity() - 1));
    else
        step = 1 / ((double)(Get_complexity()));

    if (sampling > 1)
        step = step / (double)sampling;

    ChVector<> pA = this->Evaluate(0.0);
    ChVector<> pB;
    for (par = 0; par <= 1.000000001; par = par + step) {
        pB = this->Evaluate(par);
        mres += Vlength(Vsub(pA, pB));
        pA = pB;
    }
    return mres;
}

// Draw into the current graph viewport of a ChFile_ps file

bool ChLine::DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate) {
    ChVector2<> mp1;

    mfle->GrSave();
    mfle->ClipRectangle(mfle->Get_G_p(), mfle->Get_Gs_p(), ChFile_ps::Space::PAGE);
    // start a line, move cursor to beginning
    mfle->StartLine();
    auto mv1 = this->Evaluate(0.0);
    mp1.x() = mv1.x();
    mp1.y() = mv1.y();
    mp1 = mfle->To_page_from_graph(mp1);
    mfle->MoveTo(mp1);
    double maxpoints = this->Get_complexity() * 10;
    // add points into line
    for (int i = 1; i <= maxpoints; i++) {
        mv1 = this->Evaluate(i / (double)maxpoints);
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

void ChLine::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLine>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(closed);
    marchive << CHNVP(complexityU);
}

void ChLine::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLine>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(closed);
    marchive >> CHNVP(complexityU);
}

}  // end namespace geometry
}  // end namespace chrono
