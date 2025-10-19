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

#include <cmath>

#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"


namespace chrono {
namespace fea {



    // Compute the 8 shape functions N at eta parametric coordinates. 


    /// Access the nth node.

    inline std::shared_ptr<ChNodeFEAbase> ChFieldElementHexahedron8::GetNode(unsigned int n) {
        return nodes[n];
    }

    void ChFieldElementHexahedron8::ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) {
        N.resize(GetNumNodes());
        N[0] = 0.125 * (1 - eta[0]) * (1 - eta[1]) * (1 - eta[2]);
        N[1] = 0.125 * (1 + eta[0]) * (1 - eta[1]) * (1 - eta[2]);
        N[2] = 0.125 * (1 + eta[0]) * (1 + eta[1]) * (1 - eta[2]);
        N[3] = 0.125 * (1 - eta[0]) * (1 + eta[1]) * (1 - eta[2]);
        N[4] = 0.125 * (1 - eta[0]) * (1 - eta[1]) * (1 + eta[2]);
        N[5] = 0.125 * (1 + eta[0]) * (1 - eta[1]) * (1 + eta[2]);
        N[6] = 0.125 * (1 + eta[0]) * (1 + eta[1]) * (1 + eta[2]);
        N[7] = 0.125 * (1 - eta[0]) * (1 + eta[1]) * (1 + eta[2]);
    }

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 

    void ChFieldElementHexahedron8::ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) {
        dNde.setZero(GetManifoldDimensions(), GetNumNodes());
        dNde(0, 0) = -(1 - eta[1]) * (1 - eta[2]);
        dNde(0, 1) = +(1 - eta[1]) * (1 - eta[2]);
        dNde(0, 2) = +(1 + eta[1]) * (1 - eta[2]);
        dNde(0, 3) = -(1 + eta[1]) * (1 - eta[2]);
        dNde(0, 4) = -(1 - eta[1]) * (1 + eta[2]);
        dNde(0, 5) = +(1 - eta[1]) * (1 + eta[2]);
        dNde(0, 6) = +(1 + eta[1]) * (1 + eta[2]);
        dNde(0, 7) = -(1 + eta[1]) * (1 + eta[2]);

        dNde(1, 0) = -(1 - eta[0]) * (1 - eta[2]);
        dNde(1, 1) = -(1 + eta[0]) * (1 - eta[2]);
        dNde(1, 2) = +(1 + eta[0]) * (1 - eta[2]);
        dNde(1, 3) = +(1 - eta[0]) * (1 - eta[2]);
        dNde(1, 4) = -(1 - eta[0]) * (1 + eta[2]);
        dNde(1, 5) = -(1 + eta[0]) * (1 + eta[2]);
        dNde(1, 6) = +(1 + eta[0]) * (1 + eta[2]);
        dNde(1, 7) = +(1 - eta[0]) * (1 + eta[2]);

        dNde(2, 0) = -(1 - eta[0]) * (1 - eta[1]);
        dNde(2, 1) = -(1 + eta[0]) * (1 - eta[1]);
        dNde(2, 2) = -(1 + eta[0]) * (1 + eta[1]);
        dNde(2, 3) = -(1 - eta[0]) * (1 + eta[1]);
        dNde(2, 4) = +(1 - eta[0]) * (1 - eta[1]);
        dNde(2, 5) = +(1 + eta[0]) * (1 - eta[1]);
        dNde(2, 6) = +(1 + eta[0]) * (1 + eta[1]);
        dNde(2, 7) = +(1 - eta[0]) * (1 + eta[1]);

        dNde *= 0.125; // the 1/8 factor
    }

    // Compute Jacobian J, and returns its determinant. J is square 3x3

    double ChFieldElementHexahedron8::ComputeJ(const ChVector3d eta, ChMatrix33d& J) {
        ChMatrixNM<double, 3, 8> Xhat;
        for (int i = 0; i < 8; ++i)
            Xhat.block<3, 1>(0, i) = std::static_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(i))->eigen();
        // J = Xhat * dNde^T
        ChMatrixDynamic<double> dNde(3, 8);
        ComputedNde(eta, dNde);
        J = Xhat * dNde.transpose();
        return J.determinant();
    }

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3

    double ChFieldElementHexahedron8::ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) {
        ChMatrix33<double> J;
        this->ComputeJ(eta, J);
        double mdet;
        bool isinvertible;
        J.computeInverseAndDetWithCheck(Jinv, mdet, isinvertible, 1e-9);
        return mdet;
    }


    // Tell how many Gauss points are needed for integration

    int ChFieldElementHexahedron8::GetNumQuadraturePointsForOrder(const int order) const {
        if (order == 1)
            return 1; // shortcut
        ChQuadratureTables* mtables = ChQuadrature::GetStaticTables();
        int points_on_abscissa = (int)mtables->Weight[order - 1].size();
        return (points_on_abscissa * points_on_abscissa * points_on_abscissa);
    }

    // Get i-th Gauss point weight and parametric coordinates

    void ChFieldElementHexahedron8::GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords) const {
        ChQuadratureTables* mtables = ChQuadrature::GetStaticTables();
        int points_on_abscissa = (int)mtables->Weight[order - 1].size();
        int j = i / (points_on_abscissa * points_on_abscissa);
        int k = (i / points_on_abscissa) % points_on_abscissa;
        int l = i % points_on_abscissa;
        coords.x() = mtables->Lroots[order - 1][j];
        coords.y() = mtables->Lroots[order - 1][k];
        coords.z() = mtables->Lroots[order - 1][l];
        weight = mtables->Weight[order - 1][j] * mtables->Weight[order - 1][k] * mtables->Weight[order - 1][l];
    }

}  // end namespace fea
}  // end namespace chrono
