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

#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationBSpline)

ChFunctionRotationBSpline::ChFunctionRotationBSpline() {
    const std::vector<ChQuaternion<> > mrotations = {QUNIT, QUNIT};
    this->closed = false;
    this->Setup(1, mrotations);

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionRotationBSpline::ChFunctionRotationBSpline(int morder,
                                                     const std::vector<ChQuaternion<> >& mrotations,
                                                     ChVectorDynamic<>* mknots) {
    this->closed = false;
    this->Setup(morder, mrotations, mknots);

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionRotationBSpline::ChFunctionRotationBSpline(const ChFunctionRotationBSpline& other) {
    this->rotations = other.rotations;
    this->p = other.p;
    this->knots = other.knots;
    this->space_fx = other.space_fx;
    this->closed = other.closed;
}

ChFunctionRotationBSpline::~ChFunctionRotationBSpline() {}

void ChFunctionRotationBSpline::Setup(int morder,
                                      const std::vector<ChQuaternion<> >& mrotations,
                                      ChVectorDynamic<>* mknots) {
    if (morder < 1)
        throw std::invalid_argument("ChFunctionRotationBSpline::Setup requires order >= 1.");

    if (mrotations.size() < morder + 1)
        throw std::invalid_argument("ChFunctionRotationBSpline::Setup requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mrotations.size() + morder + 1))
        throw std::invalid_argument("ChFunctionRotationBSpline::Setup: knots must have size=n_points+order+1");

    this->p = morder;
    this->rotations = mrotations;
    int n = (int)rotations.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n + p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);
    }
}

ChQuaternion<> ChFunctionRotationBSpline::GetQuat(double s) const {
    double fs = space_fx->GetVal(s);

    double mU;
    if (this->closed)
        mU = fmod(fs, 1.0);
    else
        mU = fs;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(this->p, u, this->knots);

    ChVectorDynamic<> N(this->p + 1);
    ChBasisToolsBSpline::BasisEvaluate(this->p, spanU, u, this->knots, N);

    // Use the quaternion spline interpolation as in Kim & Kim 1995 paper, ie. with cumulative basis.
    // Note: now, instead of creating a ChVectorDynamic<> Bi(this->p + 1); vector containing all the basis
    // recomputed in cumulative form, we just use a single scalar Bi and update it only when needed.

    int uind = spanU - p;
    ChQuaternion<> qqpowBi;
    ChQuaternion<> q0 = rotations[uind + 0];  // q0
    ChQuaternion<> q;
    double Bi = 0;
    // should be Bi(0) = 1 in most cases, anyway: compute first cumulative basis:
    for (int j = 0; j < N.size(); ++j)
        Bi += N(j);
    q.SetFromRotVec(q0.GetRotVec() * Bi);

    for (int i = 1; i <= this->p; i++) {
        // update cumulative basis - avoids repeating the sums, as it should be:  Bi=0; for (int j = i; j < N.size();
        // ++j) {Bi += N(j);}
        Bi -= N(i - 1);
        // compute delta rotation
        qqpowBi.SetFromRotVec((rotations[uind + i - 1].GetConjugate() * rotations[uind + i]).GetRotVec() * Bi);
        // apply delta rotation
        q *= qqpowBi;
    }

    return q;
}

void ChFunctionRotationBSpline::SetClosed(bool mc) {
    if (this->closed == mc)
        return;

    // switch open->closed
    if (mc == true) {
        // add p control points to be wrapped: resize knots and control points
        auto n = this->rotations.size();
        n += p;
        this->rotations.resize(n);
        this->knots.setZero(n + p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(this->knots, p);

        // wrap last control points
        for (int i = 0; i < p; ++i)
            this->rotations[n - p + i] = this->rotations[i];
    }

    // switch closed->open
    if (mc == false) {
        // remove p control points that was wrapped: resize knots and control points
        auto n = this->rotations.size();
        n -= p;
        this->rotations.resize(n);
        this->knots.setZero(n + p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);
    }

    this->closed = mc;
}

void ChFunctionRotationBSpline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotationBSpline>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(rotations);
    ////archive_out << CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    archive_out << CHNVP(p);
    archive_out << CHNVP(space_fx);
    archive_out << CHNVP(closed);
}

void ChFunctionRotationBSpline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotationBSpline>();
    // deserialize parent class
    ChFunctionRotation::ArchiveIn(archive_in);
    // deserialize all member data:
    archive_in >> CHNVP(rotations);
    ////archive_in >> CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    archive_in >> CHNVP(p);
    archive_in >> CHNVP(space_fx);
    archive_in >> CHNVP(closed);
}

}  // end namespace chrono