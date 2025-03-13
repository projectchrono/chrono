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
    this->Setup(1, 1, mpoints);
}

ChSurfaceNurbs::ChSurfaceNurbs(
    int morder_u,                          // order pu: 1= linear, 2=quadratic, etc.
    int morder_v,                          // order pv: 1= linear, 2=quadratic, etc.
    ChMatrixDynamic<ChVector3d>& mpoints,  // control points, size nuxnv. Required: at least nu >= pu+1, same for v
    ChVectorDynamic<>* mknots_u,  // knots, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
    ChVectorDynamic<>* mknots_v,  // knots, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
    ChMatrixDynamic<>* weights    // weights, size nuxnv. If not provided, all weights as 1.
) {
    this->Setup(morder_u, morder_v, mpoints, mknots_u, mknots_v, weights);
}

ChSurfaceNurbs::ChSurfaceNurbs(const ChSurfaceNurbs& source) : ChSurface(source) {
    this->points = source.points;
    this->p_u = source.p_u;
    this->p_v = source.p_v;
    this->knots_u = source.knots_u;
    this->knots_v = source.knots_v;
    this->weights = source.weights;
}

ChVector3d ChSurfaceNurbs::Evaluate(double parU, double parV) const {
    double u = ComputeKnotUfromU(parU);
    double v = ComputeKnotVfromV(parV);

    ChMatrixDynamic<> mR(p_u + 1, p_v + 1);
    ChBasisToolsNurbsSurfaces::BasisEvaluate(p_u, p_v, u, v, weights, knots_u, knots_v, mR);

    int spanU = ChBasisToolsBSpline::FindSpan(p_u, u, knots_u);
    int spanV = ChBasisToolsBSpline::FindSpan(p_v, v, knots_v);

    ChVector3d pos = VNULL;
    int uind = spanU - p_u;
    int vind = spanV - p_v;
    for (int iu = 0; iu <= this->p_u; iu++) {
        for (int iv = 0; iv <= this->p_v; iv++) {
            pos += points(uind + iu, vind + iv) * mR(iu, iv);
        }
    }

    return pos;
}

void ChSurfaceNurbs::Setup(
    int morder_u,                          // order pu: 1= linear, 2=quadratic, etc.
    int morder_v,                          // order pv: 1= linear, 2=quadratic, etc.
    ChMatrixDynamic<ChVector3d>& mpoints,  // control points, size nuxnv. Required: at least nu >= pu+1, same for v
    ChVectorDynamic<>* mknots_u,    // knots u, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
    ChVectorDynamic<>* mknots_v,    // knots v, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
    ChMatrixDynamic<>* new_weights  // weights, size nuxnv. If not provided, all weights as 1.
) {
    if (morder_u < 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires u order >= 1.");

    if (morder_v < 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires v order >= 1.");

    if (mpoints.rows() < morder_u + 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires at least (order_u+1)x(order_v+1) control points.");
    if (mpoints.cols() < morder_v + 1)
        throw std::invalid_argument("ChSurfaceNurbs::Setup requires at least (order_u+1)x(order_v+1) control points.");

    if (mknots_u && mknots_u->size() != (mpoints.rows() + morder_u + 1))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: knots_u must have size=n_points_u+order_u+1");
    if (mknots_v && mknots_v->size() != (mpoints.cols() + morder_v + 1))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: knots_v must have size=n_points_v+order_v+1");

    if (new_weights && (new_weights->rows() != mpoints.rows() || new_weights->cols() != mpoints.cols()))
        throw std::invalid_argument("ChSurfaceNurbs::Setup: weights matrix must have size as point matrix");

    this->p_u = morder_u;
    this->p_v = morder_v;
    this->points = mpoints;
    int n_u = (int)points.rows();
    int n_v = (int)points.cols();

    if (mknots_u)
        this->knots_u = *mknots_u;
    else {
        this->knots_u.setZero(n_u + p_u + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots_u, p_u);
    }
    if (mknots_v)
        this->knots_v = *mknots_v;
    else {
        this->knots_v.setZero(n_v + p_v + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots_v, p_v);
    }

    if (new_weights)
        this->weights = *new_weights;
    else
        this->weights.setConstant(n_u, n_v, 1.0);
}

void ChSurfaceNurbs::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSurfaceNurbs>();
    // serialize parent class
    ChSurface::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    archive_out << CHNVP(weights);
    archive_out << CHNVP(knots_u);
    archive_out << CHNVP(knots_v);
    archive_out << CHNVP(p_u);
    archive_out << CHNVP(p_v);
}

void ChSurfaceNurbs::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSurfaceNurbs>();
    // deserialize parent class
    ChSurface::ArchiveIn(archive_in);
    // stream in all member data:
    ////archive_in >> CHNVP(points);
    ////archive_in >> CHNVP(weights);
    ////archive_in >> CHNVP(knots_u);
    ////archive_in >> CHNVP(knots_v);
    archive_in >> CHNVP(p_u);
    archive_in >> CHNVP(p_v);
}

}  // end namespace chrono
