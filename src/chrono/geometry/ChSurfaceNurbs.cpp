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

#include "chrono/geometry/ChSurfaceNurbs.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSurfaceNurbs)

ChSurfaceNurbs::ChSurfaceNurbs() {
    ChMatrixDynamic<ChVector3d> mpoints(2, 2);
    mpoints(0, 0) = ChVector3d(-1, -1, 0);
    mpoints(1, 0) = ChVector3d(1, -1, 0);
    mpoints(0, 1) = ChVector3d(-1, 1, 0);
    mpoints(1, 1) = ChVector3d(1, 1, 0);
    Setup(1, 1, mpoints);
}

ChSurfaceNurbs::ChSurfaceNurbs(int order_u,                                // order pu: 1= linear, 2=quadratic, etc.
                               int order_v,                                // order pv: 1= linear, 2=quadratic, etc.
                               const ChMatrixDynamic<ChVector3d>& points,  // control points, size nuxnv. Required: at least nu >= pu+1, same for v
                               const ChVectorDynamic<>* knots_u,           // knots, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
                               const ChVectorDynamic<>* knots_v,           // knots, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
                               const ChMatrixDynamic<>* weights            // weights, size nuxnv. If not provided, all weights as 1.
) {
    Setup(order_u, order_v, points, knots_u, knots_v, weights);
}

ChSurfaceNurbs::ChSurfaceNurbs(const ChSurfaceNurbs& source) : ChSurface(source) {
    m_points = source.m_points;
    m_p_u = source.m_p_u;
    m_p_v = source.m_p_v;
    m_knots_u = source.m_knots_u;
    m_knots_v = source.m_knots_v;
    m_weights = source.m_weights;
}

ChVector3d ChSurfaceNurbs::Evaluate(double parU, double parV) const {
    double u = ComputeKnotUfromU(parU);
    double v = ComputeKnotVfromV(parV);

    ChMatrixDynamic<> mR(m_p_u + 1, m_p_v + 1);
    ChBasisToolsNurbsSurfaces::BasisEvaluate(m_p_u, m_p_v, u, v, m_weights, m_knots_u, m_knots_v, mR);

    int spanU = ChBasisToolsBSpline::FindSpan(m_p_u, u, m_knots_u);
    int spanV = ChBasisToolsBSpline::FindSpan(m_p_v, v, m_knots_v);

    ChVector3d pos = VNULL;
    int uind = spanU - m_p_u;
    int vind = spanV - m_p_v;
    for (int iu = 0; iu <= this->m_p_u; iu++) {
        for (int iv = 0; iv <= this->m_p_v; iv++) {
            pos += m_points(uind + iu, vind + iv) * mR(iu, iv);
        }
    }

    return pos;
}

void ChSurfaceNurbs::Setup(int order_u,                                // order pu: 1= linear, 2=quadratic, etc.
                           int order_v,                                // order pv: 1= linear, 2=quadratic, etc.
                           const ChMatrixDynamic<ChVector3d>& points,  // control points, size nuxnv. Required: at least nu >= pu+1, same for v
                           const ChVectorDynamic<>* knots_u,           // knots u, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
                           const ChVectorDynamic<>* knots_v,           // knots v, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
                           const ChMatrixDynamic<>* new_weights        // weights, size nuxnv. If not provided, all weights as 1.
) {
    if (order_u < 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires u order >= 1.");

    if (order_v < 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires v order >= 1.");

    if (points.rows() < order_u + 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires at least (order_u+1)x(order_v+1) control points.");
    if (points.cols() < order_v + 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires at least (order_u+1)x(order_v+1) control points.");

    if (knots_u && knots_u->size() != (points.rows() + order_u + 1))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: knots_u must have size=n_points_u+order_u+1");
    if (knots_v && knots_v->size() != (points.cols() + order_v + 1))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: knots_v must have size=n_points_v+order_v+1");

    if (new_weights && (new_weights->rows() != points.rows() || new_weights->cols() != points.cols()))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: weights matrix must have size as point matrix");

    m_p_u = order_u;
    m_p_v = order_v;
    m_points = points;
    int n_u = (int)m_points.rows();
    int n_v = (int)m_points.cols();

    if (knots_u)
        m_knots_u = *knots_u;
    else {
        m_knots_u.setZero(n_u + m_p_u + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(m_knots_u, m_p_u);
    }
    if (knots_v)
        m_knots_v = *knots_v;
    else {
        m_knots_v.setZero(n_v + m_p_v + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(m_knots_v, m_p_v);
    }

    if (new_weights)
        m_weights = *new_weights;
    else
        m_weights.setConstant(n_u, n_v, 1.0);
}

void ChSurfaceNurbs::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSurfaceNurbs>();
    // serialize parent class
    ChSurface::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_points);
    archive_out << CHNVP(m_weights);
    archive_out << CHNVP(m_knots_u);
    archive_out << CHNVP(m_knots_v);
    archive_out << CHNVP(m_p_u);
    archive_out << CHNVP(m_p_v);
}

void ChSurfaceNurbs::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSurfaceNurbs>();
    // deserialize parent class
    ChSurface::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_points);
    archive_in >> CHNVP(m_weights);
    archive_in >> CHNVP(m_knots_u);
    archive_in >> CHNVP(m_knots_v);
    archive_in >> CHNVP(m_p_u);
    archive_in >> CHNVP(m_p_v);
}

}  // end namespace chrono
