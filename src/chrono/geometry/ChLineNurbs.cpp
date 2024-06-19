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

#include "chrono/geometry/ChLineNurbs.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineNurbs)

ChLineNurbs::ChLineNurbs() {
    std::vector<ChVector3d> mpoints = {ChVector3d(-1, 0, 0), ChVector3d(1, 0, 0)};
    this->Setup(1, mpoints);
}

ChLineNurbs::ChLineNurbs(
    int morder,                        ///< order p: 1= linear, 2=quadratic, etc.
    std::vector<ChVector3d>& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots,         ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    ChVectorDynamic<>* weights         ///< weights, size w. Required w=n. If not provided, all weights as 1.
) {
    this->Setup(morder, mpoints, mknots, weights);
}

ChLineNurbs::ChLineNurbs(const ChLineNurbs& source) : ChLine(source) {
    this->points = source.points;
    this->p = source.p;
    this->knots = source.knots;
    this->weights = source.weights;
}

ChVector3d ChLineNurbs::Evaluate(double parU) const {
    double u = ComputeKnotUfromU(parU);

    ChVectorDynamic<> mR(this->p + 1);
    ChBasisToolsNurbs::BasisEvaluate(this->p, u, this->weights, this->knots, mR);

    int spanU = ChBasisToolsBSpline::FindSpan(this->p, u, this->knots);

    ChVector3d pos = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        pos += points[uind + i] * mR(i);
    }

    return pos;
}

ChVector3d ChLineNurbs::GetTangent(double parU) const {
    double u = ComputeKnotUfromU(parU);

    ChVectorDynamic<> mR(this->p + 1);
    ChVectorDynamic<> mdR(this->p + 1);
    ChBasisToolsNurbs::BasisEvaluateDeriv(this->p, u, this->weights, this->knots, mR, mdR);

    int spanU = ChBasisToolsBSpline::FindSpan(this->p, u, this->knots);

    ChVector3d dir = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        dir += points[uind + i] * mdR(i);
    }

    return dir;
}

void ChLineNurbs::Setup(
    int morder,                        ///< order p: 1= linear, 2=quadratic, etc.
    std::vector<ChVector3d>& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots,         ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    ChVectorDynamic<>* weights         ///< weights, size w. Required w=n. If not provided, all weights as 1.
) {
    if (morder < 1)
        throw std::invalid_argument("ChLineNurbs::Setup requires order >= 1.");

    if (mpoints.size() < morder + 1)
        throw std::invalid_argument("ChLineNurbs::Setup requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mpoints.size() + morder + 1))
        throw std::invalid_argument("ChLineNurbs::Setup knots must have size=n_points+order+1");

    if (weights && (size_t)weights->size() != mpoints.size())
        throw std::invalid_argument("ChLineNurbs::Setup weights must have size=n_points");

    this->p = morder;
    this->points = mpoints;
    int n = (int)points.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n + p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);
    }

    if (weights)
        this->weights = *weights;
    else
        this->weights.setConstant(n, 1.0);
}

void ChLineNurbs::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineNurbs>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    ////archive_out << CHNVP(weights); //**TODO MATRIX DESERIALIZATION
    ////archive_out << CHNVP(knots); //**TODO MATRIX DESERIALIZATION
    archive_out << CHNVP(p);
}

void ChLineNurbs::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineNurbs>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    ////archive_in >> CHNVP(weights); //**TODO MATRIX DESERIALIZATION
    ////archive_in >> CHNVP(knots); //**TODO MATRIX DESERIALIZATION
    archive_in >> CHNVP(p);
}

}  // end namespace chrono
