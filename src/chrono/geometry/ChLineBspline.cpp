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

#include "chrono/geometry/ChLineBspline.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBspline)

ChLineBspline::ChLineBspline() {
    std::vector< ChVector<> > mpoints = { ChVector<>(-1,0,0), ChVector<>(1,0,0)};
    this->SetupData(1, mpoints);
}

ChLineBspline::ChLineBspline(int morder,            ///< order p: 1= linear, 2=quadratic, etc.
                std::vector< ChVector<> >& mpoints, ///< control points, size n. Required: at least n >= p+1
                ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                ) {
    this->SetupData(morder, mpoints, mknots);
}

ChLineBspline::ChLineBspline(const ChLineBspline& source) : ChLine(source) {
    
    this->points = source.points;
    this->p = source.p;
    this->knots = source.knots;
}

void ChLineBspline::Evaluate(
                ChVector<>& pos, 
                const double parU) const {

        double u = ComputeKnotUfromU(parU);

        int spanU = ChBasisToolsBspline::FindSpan(this->p, u, this->knots);

        ChVectorDynamic<> N(this->p+1);
        ChBasisToolsBspline::BasisEvaluate(this->p, spanU, u, this->knots, N);
        
        pos = VNULL;
        int uind = spanU - p;
        for (int i = 0; i <= this->p; i++) {
            pos += points[uind + i] * N(i);
        }
}

void ChLineBspline::Derive(
                ChVector<>& dir, 
                const double parU) const {

        double u = ComputeKnotUfromU(parU);

        int spanU = ChBasisToolsBspline::FindSpan(this->p, u, this->knots);

        ChMatrixDynamic<>  NdN  (2, p + 1); // basis on 1st row and their 1st derivatives on 2nd row
        ChBasisToolsBspline::BasisEvaluateDeriv(this->p, spanU, u, this->knots, NdN);
        
        dir = VNULL;
        int uind = spanU - p;
        for (int i = 0; i <= this->p; i++) {
            dir += points[uind + i] * NdN(1,i);
        }
}



void ChLineBspline::SetupData( int morder,              ///< order p: 1= linear, 2=quadratic, etc.
                    std::vector< ChVector<> >& mpoints, ///< control points, size n. Required: at least n >= p+1
                    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                    ) {
    if (morder < 1) 
        throw ChException ("ChLineBspline::SetupData requires order >= 1."); 

    if (mpoints.size() < morder+1) 
        throw ChException ("ChLineBspline::SetupData requires at least order+1 control points.");

    if (mknots && mknots->GetLength() != (mpoints.size()+morder+1))
        throw ChException ("ChLineBspline::SetupData: knots must have size=n_points+order+1");

    this->p = morder;
    this->points = mpoints;
    int n = (int)points.size();
    int k = n+p+1;

    if (mknots)
        this->knots.CopyFromMatrix(*mknots);
    else {
        this->knots.Reset(n+p+1);
        ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(this->knots,p);
    }
}



}  // end namespace geometry
}  // end namespace chrono
