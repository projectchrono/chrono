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
    m_points = source.m_points;
    m_p = source.m_p;
    m_knots = source.m_knots;
    m_weights = source.m_weights;
}

ChVector3d ChLineNurbs::Evaluate(double parU) const {
    double u = ComputeKnotUfromU(parU);

    ChVectorDynamic<> mR(m_p + 1);
    ChBasisToolsNurbs::BasisEvaluate(m_p, u, m_weights, m_knots, mR);

    int spanU = ChBasisToolsBSpline::FindSpan(m_p, u, m_knots);

    ChVector3d pos = VNULL;
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; i++) {
        pos += m_points[uind + i] * mR(i);
    }

    return pos;
}

ChVector3d ChLineNurbs::GetTangent(double parU) const {
    double u = ComputeKnotUfromU(parU);

    ChVectorDynamic<> mR(m_p + 1);
    ChVectorDynamic<> mdR(m_p + 1);
    ChBasisToolsNurbs::BasisEvaluateDeriv(m_p, u, m_weights, m_knots, mR, mdR);

    int spanU = ChBasisToolsBSpline::FindSpan(m_p, u, m_knots);

    ChVector3d dir = VNULL;
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; i++) {
        dir += m_points[uind + i] * mdR(i);
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

    m_p = morder;
    m_points = mpoints;
    int n = (int)m_points.size();

    if (mknots)
        m_knots = *mknots;
    else {
        m_knots.setZero(n + m_p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(m_knots, m_p);
    }

    if (weights)
        m_weights = *weights;
    else
        m_weights.setConstant(n, 1.0);
}

double ChLineNurbs::ComputeUfromKnotU(double u) const {
    return (u - m_knots(m_p)) / (m_knots(m_knots.size() - 1 - m_p) - m_knots(m_p));
}

double ChLineNurbs::ComputeKnotUfromU(double U) const {
    return U * (m_knots(m_knots.size() - 1 - m_p) - m_knots(m_p)) + m_knots(m_p);
}

void ChLineNurbs::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineNurbs>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_points);
    ////archive_out << CHNVP(m_weights); //**TODO MATRIX DESERIALIZATION
    ////archive_out << CHNVP(m_knots); //**TODO MATRIX DESERIALIZATION
    archive_out << CHNVP(m_p);
}

void ChLineNurbs::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineNurbs>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_points);
    ////archive_in >> CHNVP(m_weights); //**TODO MATRIX DESERIALIZATION
    ////archive_in >> CHNVP(m_knots); //**TODO MATRIX DESERIALIZATION
    archive_in >> CHNVP(m_p);
}

}  // end namespace chrono
