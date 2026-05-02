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


#include "chrono/fea/ChFieldElement.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"

namespace chrono {
namespace fea {



    // Sets the quadrature order when integrating over the element

    
/// Compute Jacobian J = dX/deta, and returns its determinant.
/// It should be GetSpatialDimensions() x 1 = 3x1, but return as 3x3 by
/// padding 2nd and 3rd columns with auxiliary orthogonal columns.
/// FALLBACK default implementation; but if possible implement a faster ad hoc computation.
double ChFieldElementLine::ComputeJ(const ChVector3d eta, ChMatrix33d& J) {
    ChMatrixDynamic<double> Xhat(3, this->GetNumNodes());
    for (unsigned int i = 0; i < this->GetNumNodes(); ++i)
        Xhat.block<3, 1>(0, i) = std::static_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(i))->eigen();
    // J = Xhat * dNde^T
    ChMatrixDynamic<double> dNde(1, this->GetNumNodes());
    ComputedNde(eta, dNde);
    J.col(0) = Xhat * dNde.transpose();
    J.col(1) = Vcross(ChVector3d((Xhat * dNde.transpose()).col(0)), VECT_X).eigen();
    J.col(2) = Vcross(ChVector3d(J.col(0)), ChVector3d(J.col(1))).eigen();
    return J.col(0).norm();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Compute Jacobian J = dX/deta, and returns its determinant.
/// It should be GetSpatialDimensions() x 2 = 3x2, but return as 3x3 by
/// padding 3rd columns with auxiliary orthogonal column.
/// FALLBACK default implementation; but if possible implement a faster ad hoc computation.
double ChFieldElementSurface::ComputeJ(const ChVector3d eta, ChMatrix33d& J) {
    ChMatrixDynamic<double> Xhat(3, this->GetNumNodes());
    for (unsigned int i = 0; i < this->GetNumNodes(); ++i)
        Xhat.block<3, 1>(0, i) = std::static_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(i))->eigen();
    // J = Xhat * dNde^T
    ChMatrixDynamic<double> dNde(1, this->GetNumNodes());
    ComputedNde(eta, dNde);
    J.col(0) = Xhat * dNde.row(0).transpose();
    J.col(1) = Xhat * dNde.row(1).transpose();
    J.col(2) = Vcross(ChVector3d(J.col(0)), ChVector3d(J.col(1))).eigen();
    return J.col(0).norm();
}


/// Compute Jacobian J = dX/deta, and returns its determinant.
/// FALLBACK default implementation; but if possible implement a faster ad hoc computation.
double ChFieldElementVolume::ComputeJ(const ChVector3d eta, ChMatrix33d& J) {
    ChMatrixDynamic<double> Xhat(3, this->GetNumNodes());
    for (unsigned int i = 0; i < this->GetNumNodes(); ++i)
        Xhat.block<3, 1>(0, i) = std::static_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(i))->eigen();
    // J = Xhat * dNde^T
    ChMatrixDynamic<double> dNde(3, this->GetNumNodes());
    ComputedNde(eta, dNde);
    J = Xhat * dNde.transpose();
    return J.determinant();
}

}  // end namespace fea
}  // end namespace chrono
