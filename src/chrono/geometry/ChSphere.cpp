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

#include <cstdio>

#include "chrono/geometry/ChSphere.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSphere)

ChSphere::ChSphere(const ChSphere& source) {
    center = source.center;
    rad = source.rad;
}

void ChSphere::GetBoundingBox(double& xmin,
                              double& xmax,
                              double& ymin,
                              double& ymax,
                              double& zmin,
                              double& zmax,
                              ChMatrix33<>* Rot) const {
    ChVector<> trsfCenter = center;
    if (Rot) {
        trsfCenter = Rot->MatrT_x_Vect(center);
    }
    xmin = trsfCenter.x() - rad;
    xmax = trsfCenter.x() + rad;
    ymin = trsfCenter.y() - rad;
    ymax = trsfCenter.y() + rad;
    zmin = trsfCenter.z() - rad;
    zmax = trsfCenter.z() + rad;
}

void ChSphere::CovarianceMatrix(ChMatrix33<>& C) const {
    C.Reset();
    C(0, 0) = center.x() * center.x();
    C(1, 1) = center.y() * center.y();
    C(2, 2) = center.z() * center.z();
}

}  // end namespace geometry
}  // end namespace chrono
