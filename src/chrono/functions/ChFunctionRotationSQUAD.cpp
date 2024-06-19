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

#include "chrono/functions/ChFunctionRotationSQUAD.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationSQUAD)

ChFunctionRotationSQUAD::ChFunctionRotationSQUAD() {
    const std::vector<ChQuaternion<> > mrotations = {QUNIT, QUNIT};
    this->closed = false;
    this->Setup(mrotations);

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionRotationSQUAD::ChFunctionRotationSQUAD(const std::vector<ChQuaternion<> >& mrotations,
                                                 ChVectorDynamic<>* mknots) {
    this->closed = false;
    this->Setup(mrotations, mknots);

    // default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunctionRamp>(0, 1.);
}

ChFunctionRotationSQUAD::ChFunctionRotationSQUAD(const ChFunctionRotationSQUAD& other) {
    this->rotations = other.rotations;
    this->p = other.p;
    this->knots = other.knots;
    this->space_fx = other.space_fx;
    this->closed = other.closed;
}

ChFunctionRotationSQUAD::~ChFunctionRotationSQUAD() {}

void ChFunctionRotationSQUAD::Setup(const std::vector<ChQuaternion<> >& mrotations, ChVectorDynamic<>* mknots) {
    if (mrotations.size() < 2)
        throw std::invalid_argument("ChFunctionRotationSQUAD::Setup requires at least 2control points.");

    if (mknots && (size_t)mknots->size() != mrotations.size())
        throw std::invalid_argument("ChFunctionRotationSQUAD::Setup: knots must be as many as control points");

    this->rotations = mrotations;
    auto n = (int)rotations.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n);
        ChBasisToolsBSpline::ComputeKnotUniform(this->knots, 1);
    }
}

ChQuaternion<> SLERP(const ChQuaternion<>& qa, const ChQuaternion<>& qb, double t) {
    ChQuaternion<> qdelta = qa.GetConjugate() * qb;
    ChQuaternion<> qdelta_t;
    qdelta_t.SetFromRotVec(qdelta.GetRotVec() * t);

    return qa * qdelta_t;
};

ChQuaternion<> QUADRANGLE(const ChQuaternion<>& q0, const ChQuaternion<>& q1, const ChQuaternion<>& q2) {
    ChQuaternion<> qInv = q1.GetConjugate();

    ChQuaternion<> sq1 = qInv * q2;

    ChVector3d cart0 = sq1.GetRotVec();

    ChQuaternion<> sq0 = qInv * q0;

    ChVector3d cart1 = sq0.GetRotVec();

    ChVector3d cart_aux = (cart0 + cart1) * -0.25;

    ChQuaternion<> q_aux;
    q_aux.SetFromRotVec(cart_aux);

    return q1 * q_aux;
};

ChQuaternion<> ChFunctionRotationSQUAD::GetQuat(double s) const {
    double fs = space_fx->GetVal(s);

    double mU;
    if (this->closed)
        mU = fmod(fs, 1.0);
    else
        mU = fs;

    double u = ComputeKnotUfromU(mU);

    // Linear search of span in knot vector. Bisection could be faster, but for small number of knots this is efficient
    // anyway.
    int spanU = 0;
    for (; spanU < this->knots.size() - 2; ++spanU)
        if (u < this->knots(spanU + 1))
            break;

    double span_t = (u - this->knots(spanU)) / (this->knots(spanU + 1) - this->knots(spanU));

    ChQuaternion<> q1 = rotations[spanU];
    ChQuaternion<> q2 = rotations[spanU + 1];

    ChQuaternion<> q0;
    ChQuaternion<> q3;

    if (spanU > 0) {
        q0 = rotations[spanU - 1];
    } else {
        if (this->closed)
            q0 = rotations[rotations.size() - 2];
        else
            q0 = q1;
    }

    if (spanU < rotations.size() - 2) {
        q3 = rotations[spanU + 2];
    } else {
        if (this->closed)
            q3 = rotations[1];
        else
            q3 = q2;
    }

    // TEST: do a simple SLERP
    // return SLERP(q1, q2, span_t);

    ChQuaternion<> s0 = QUADRANGLE(q0, q1, q2);
    ChQuaternion<> s1 = QUADRANGLE(q1, q2, q3);

    ChQuaternion<> slerp0 = SLERP(q1, q2, span_t);
    ChQuaternion<> slerp1 = SLERP(s0, s1, span_t);

    return SLERP(slerp0, slerp1, 2.0 * span_t * (1.0 - span_t));
}

void ChFunctionRotationSQUAD::SetClosed(bool mc) {
    if (this->closed == mc)
        return;

    // switch open->closed
    if (mc == true) {
        // add p control points to be wrapped: resize knots and control points
        auto n = this->rotations.size();
        n += 1;
        this->rotations.resize(n);
        this->knots.setZero(n);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(this->knots, 1);

        // wrap last control point
        this->rotations[n - 1] = this->rotations[0];
    }

    // switch closed->open
    if (mc == false) {
        // remove p control points that was wrapped: resize knots and control points
        auto n = this->rotations.size();
        n -= 1;
        this->rotations.resize(n);
        this->knots.setZero(n);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(this->knots, 1);
    }

    this->closed = mc;
}

void ChFunctionRotationSQUAD::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotationSQUAD>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(rotations);
    ////archive_out << CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    archive_out << CHNVP(p);
    archive_out << CHNVP(space_fx);
    archive_out << CHNVP(closed);
}

void ChFunctionRotationSQUAD::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotationSQUAD>();
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