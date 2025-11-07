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

#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"

namespace chrono {
namespace fea {






    /// Set the nodes used by this tetrahedron.

    void ChFieldElementTetrahedron4::SetNodes(std::shared_ptr<ChNodeFEAfieldXYZ> nodeA, std::shared_ptr<ChNodeFEAfieldXYZ> nodeB, std::shared_ptr<ChNodeFEAfieldXYZ> nodeC, std::shared_ptr<ChNodeFEAfieldXYZ> nodeD) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        nodes[2] = nodeC;
        nodes[3] = nodeD;
    }

    // Compute the 4 shape functions N at eta parametric coordinates. 


    /// Access the nth node.

    inline std::shared_ptr<ChNodeFEAbase> ChFieldElementTetrahedron4::GetNode(unsigned int n) {
        return nodes[n];
    }

    inline void ChFieldElementTetrahedron4::ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) {
        N.resize(GetNumNodes());
        N[0] = eta[0];
        N[1] = eta[1];
        N[2] = eta[2];
        N[3] = 1.0 - eta[0] - eta[1] - eta[2];
    }

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 

    inline void ChFieldElementTetrahedron4::ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) {
        dNde.setZero(GetManifoldDimensions(), GetNumNodes());
        dNde(0, 0) = 1.0;
        dNde(1, 1) = 1.0;
        dNde(2, 2) = 1.0;
        dNde(0, 3) = -1.0;
        dNde(1, 3) = -1.0;
        dNde(2, 3) = -1.0;
    }

    // Compute shape function spatial derivatives dN/dX at eta parametric coordinates.
    // Write shape functions dN_j(eta)/dX_i in dNdX, a matrix with 4 columns, and 3 rows.
    // Instead of falling back to default dNdX = J^{-T} * dNde; for lin tetrahedron we know the ad-hoc expression:

    inline void ChFieldElementTetrahedron4::ComputedNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) {
        dNdX.resize(3, 4);
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        double inv_det_J = 1.0 / Vdot(x14, Vcross(x24, x34));
        ChVector3d vai = inv_det_J * Vcross(x24, x34);
        ChVector3d vbi = inv_det_J * Vcross(x34, x14);
        ChVector3d vci = inv_det_J * Vcross(x14, x24);
        dNdX(0, 0) = vai.x();   dNdX(0, 1) = vbi.x();   dNdX(0, 2) = vci.x();   dNdX(0, 3) = -vai.x() - vbi.x() - vci.x();
        dNdX(1, 0) = vai.y();   dNdX(1, 1) = vbi.y();   dNdX(1, 2) = vci.y();   dNdX(1, 3) = -vai.y() - vbi.y() - vci.y();
        dNdX(2, 0) = vai.z();   dNdX(2, 1) = vbi.z();   dNdX(2, 2) = vci.z();   dNdX(2, 3) = -vai.z() - vbi.z() - vci.z();
        //***TEST***
        //ChMatrixDynamic<> test_dNdX(3, 4);
        //ChFieldElement::ComputedNdX(eta, test_dNdX);
        //**** 
    }

    // Compute Jacobian J, and returns its determinant. J is square 3x3

    double ChFieldElementTetrahedron4::ComputeJ(const ChVector3d eta, ChMatrix33d& J) {
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        J(0, 0) = x14.x();  J(0, 1) = x24.x();  J(0, 2) = x34.x();
        J(1, 0) = x14.y();  J(1, 1) = x24.y();  J(1, 2) = x34.y();
        J(2, 0) = x14.z();  J(2, 1) = x24.z();  J(2, 2) = x34.z();
        return Vdot(x14, Vcross(x24, x34));
    }

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3

    double ChFieldElementTetrahedron4::ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) {
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        double inv_det_J = 1.0 / Vdot(x14, Vcross(x24, x34));
        Jinv(0, Eigen::indexing::all) = inv_det_J * Vcross(x24, x34).eigen().transpose();
        Jinv(1, Eigen::indexing::all) = inv_det_J * Vcross(x34, x14).eigen().transpose();
        Jinv(2, Eigen::indexing::all) = inv_det_J * Vcross(x14, x24).eigen().transpose();
        return inv_det_J;
    }


    // Tell how many Gauss points are needed for integration

    inline int ChFieldElementTetrahedron4::GetNumQuadraturePointsForOrder(const int order) const {
        if (order == 1)
            return 1; // shortcut
        ChQuadratureTablesTetrahedron* mtables = ChQuadrature::GetStaticTablesTetrahedron();
        return (int)(mtables->Weight[order - 1].size());
    }

    inline void ChFieldElementTetrahedron4::GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords) const {
        if (order == 1) {
            coords.Set(0.25);
            weight = CH_1_6;
            return; // shortcut
        }
        ChQuadratureTablesTetrahedron* mtables = ChQuadrature::GetStaticTablesTetrahedron();
        coords.x() = mtables->LrootsU[order - 1][i];
        coords.y() = mtables->LrootsV[order - 1][i];
        coords.z() = mtables->LrootsW[order - 1][i];
        weight = mtables->Weight[order - 1][i] * CH_1_6; // the 1/6 factor is not in table
    }



}  // end namespace fea
}  // end namespace chrono
