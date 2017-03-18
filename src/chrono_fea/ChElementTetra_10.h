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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#ifndef CHELEMENTTETRA10_H
#define CHELEMENTTETRA10_H

#include <cmath>

#include "chrono_fea/ChElementTetrahedron.h"
#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Tetahedron FEA element with 10 nodes.
/// This is a quadratic element for displacementes; stress and strain
/// are interpolated depending on Gauss points.
class ChApiFea ChElementTetra_10 : public ChElementTetrahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    std::vector<ChMatrixDynamic<> >
        MatrB;  // matrices of shape function's partial derivatives (one for each integration point)
                // we use a vector to keep in memory all the four matrices (-> 4 integr. point)
    ChMatrixDynamic<> StiffnessMatrix;

    ChMatrixNM<double, 4, 4> mM;  // for speeding up corotational approach

  public:
    ChElementTetra_10();
    virtual ~ChElementTetra_10();

    virtual int GetNnodes() override { return 10; }
    virtual int GetNdofs() override { return 10 * 3; }
    virtual int GetNodeNdofs(int n) override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                          std::shared_ptr<ChNodeFEAxyz> nodeB,
                          std::shared_ptr<ChNodeFEAxyz> nodeC,
                          std::shared_ptr<ChNodeFEAxyz> nodeD,
                          std::shared_ptr<ChNodeFEAxyz> nodeE,
                          std::shared_ptr<ChNodeFEAxyz> nodeF,
                          std::shared_ptr<ChNodeFEAxyz> nodeG,
                          std::shared_ptr<ChNodeFEAxyz> nodeH,
                          std::shared_ptr<ChNodeFEAxyz> nodeI,
                          std::shared_ptr<ChNodeFEAxyz> nodeJ) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        nodes[2] = nodeC;
        nodes[3] = nodeD;
        nodes[4] = nodeE;
        nodes[5] = nodeF;
        nodes[6] = nodeG;
        nodes[7] = nodeH;
        nodes[8] = nodeI;
        nodes[9] = nodeJ;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        mvars.push_back(&nodes[2]->Variables());
        mvars.push_back(&nodes[3]->Variables());
        mvars.push_back(&nodes[4]->Variables());
        mvars.push_back(&nodes[5]->Variables());
        mvars.push_back(&nodes[6]->Variables());
        mvars.push_back(&nodes[7]->Variables());
        mvars.push_back(&nodes[8]->Variables());
        mvars.push_back(&nodes[9]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // FEM functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at zi parametric coordinates, where
    /// r=1 at 2nd vertex, s=1 at 3rd, t=1 at 4th. All ranging in [0...1].
    /// The last, u (=1 at 1st vertex) is computed form the first 3.
    /// It stores the Ni(r,s,t) values in a 1 row, 10 columns matrix.
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s, double t) {
        double u = 1.0 - r - s - t;

        // at corners
        N(0) = u * (2.0 * u - 1.0);
        N(1) = r * (2.0 * r - 1.0);
        N(2) = s * (2.0 * s - 1.0);
        N(3) = t * (2.0 * t - 1.0);
        // at mid edge
        N(4) = 4.0 * u * r;
        N(5) = 4.0 * r * s;
        N(6) = 4.0 * s * u;
        N(7) = 4.0 * u * t;
        N(8) = 4.0 * r * t;
        N(9) = 4.0 * s * t;
    };

    /// Fills the D vector (displacement) column matrix with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset(this->GetNdofs(), 1);

        for (int i = 0; i < GetNnodes(); i++)
            mD.PasteVector(A.MatrT_x_Vect(this->nodes[i]->GetPos()) - nodes[i]->GetX0(), i * 3, 0);
    }

    /// Approximation!! not the exact volume
    /// This returns an exact value only in case of Constant Metric Tetrahedron
    double ComputeVolume() {
        ChVector<> B1, C1, D1;
        B1.Sub(nodes[1]->pos, nodes[0]->pos);
        C1.Sub(nodes[2]->pos, nodes[0]->pos);
        D1.Sub(nodes[3]->pos, nodes[0]->pos);
        ChMatrixDynamic<> M(3, 3);
        M.PasteVector(B1, 0, 0);
        M.PasteVector(C1, 0, 1);
        M.PasteVector(D1, 0, 2);
        M.MatrTranspose();
        Volume = std::abs(M.Det() / 6);
        return Volume;
    }

    /// Puts inside 'Jacobian' the Jacobian matrix of the element
    /// zeta1,...,zeta4 are the four natural coordinates of the integration point
    /// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
    virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, double zeta1, double zeta2, double zeta3, double zeta4) {
        Jacobian.FillElem(1);

        Jacobian.SetElement(1, 0, 4 * (nodes[0]->pos.x() * (zeta1 - 1 / 4) + nodes[4]->pos.x() * zeta2 +
                                       nodes[6]->pos.x() * zeta3 + nodes[7]->pos.x() * zeta4));
        Jacobian.SetElement(2, 0, 4 * (nodes[0]->pos.y() * (zeta1 - 1 / 4) + nodes[4]->pos.y() * zeta2 +
                                       nodes[6]->pos.y() * zeta3 + nodes[7]->pos.y() * zeta4));
        Jacobian.SetElement(3, 0, 4 * (nodes[0]->pos.z() * (zeta1 - 1 / 4) + nodes[4]->pos.z() * zeta2 +
                                       nodes[6]->pos.z() * zeta3 + nodes[7]->pos.z() * zeta4));
        Jacobian.SetElement(1, 1, 4 * (nodes[4]->pos.x() * zeta1 + nodes[1]->pos.x() * (zeta2 - 1 / 4) +
                                       nodes[5]->pos.x() * zeta3 + nodes[8]->pos.x() * zeta4));
        Jacobian.SetElement(2, 1, 4 * (nodes[4]->pos.y() * zeta1 + nodes[1]->pos.y() * (zeta2 - 1 / 4) +
                                       nodes[5]->pos.y() * zeta3 + nodes[8]->pos.y() * zeta4));
        Jacobian.SetElement(3, 1, 4 * (nodes[4]->pos.z() * zeta1 + nodes[1]->pos.z() * (zeta2 - 1 / 4) +
                                       nodes[5]->pos.z() * zeta3 + nodes[8]->pos.z() * zeta4));
        Jacobian.SetElement(1, 2, 4 * (nodes[6]->pos.x() * zeta1 + nodes[5]->pos.x() * zeta2 +
                                       nodes[2]->pos.x() * (zeta3 - 1 / 4) + nodes[9]->pos.x() * zeta4));
        Jacobian.SetElement(2, 2, 4 * (nodes[6]->pos.y() * zeta1 + nodes[5]->pos.y() * zeta2 +
                                       nodes[2]->pos.y() * (zeta3 - 1 / 4) + nodes[9]->pos.y() * zeta4));
        Jacobian.SetElement(3, 2, 4 * (nodes[6]->pos.z() * zeta1 + nodes[5]->pos.z() * zeta2 +
                                       nodes[2]->pos.z() * (zeta3 - 1 / 4) + nodes[9]->pos.z() * zeta4));
        Jacobian.SetElement(1, 3, 4 * (nodes[7]->pos.x() * zeta1 + nodes[8]->pos.x() * zeta2 + nodes[9]->pos.x() * zeta3 +
                                       nodes[3]->pos.x() * (zeta4 - 1 / 4)));
        Jacobian.SetElement(2, 3, 4 * (nodes[7]->pos.y() * zeta1 + nodes[8]->pos.y() * zeta2 + nodes[9]->pos.y() * zeta3 +
                                       nodes[3]->pos.y() * (zeta4 - 1 / 4)));
        Jacobian.SetElement(3, 3, 4 * (nodes[7]->pos.z() * zeta1 + nodes[8]->pos.z() * zeta2 + nodes[9]->pos.z() * zeta3 +
                                       nodes[3]->pos.z() * (zeta4 - 1 / 4)));
    }

    /// Computes the matrix of partial derivatives and puts data in "mmatrB"
    ///	evaluated at natural coordinates zeta1,...,zeta4
    /// note: in case of tetrahedral elements natural coord. vary in the range 0 ... +1
    virtual void ComputeMatrB(ChMatrixDynamic<>& mmatrB,
                              double zeta1,
                              double zeta2,
                              double zeta3,
                              double zeta4,
                              double& JacobianDet) {
        ChMatrixDynamic<> Jacobian(4, 4);
        ComputeJacobian(Jacobian, zeta1, zeta2, zeta3, zeta4);

        double Jdet = Jacobian.Det();
        JacobianDet = Jdet;  // !!! store the Jacobian Determinant: needed for the integration

        mmatrB.SetElement(0, 0, (4 * zeta1 - 1) *
                                    ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                     (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) /
                                    Jdet);
        mmatrB.SetElement(0, 3, (4 * zeta2 - 1) *
                                    ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                     (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) /
                                    Jdet);
        mmatrB.SetElement(0, 6, (4 * zeta3 - 1) *
                                    ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                     (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) /
                                    Jdet);
        mmatrB.SetElement(0, 9, (4 * zeta4 - 1) *
                                    ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                     (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) /
                                    Jdet);
        mmatrB.SetElement(0, 12, 4 * (zeta1 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                               (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) +
                                      zeta2 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                               (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                                     Jdet);
        mmatrB.SetElement(0, 15, 4 * (zeta2 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                               (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) +
                                      zeta3 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                               (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                                     Jdet);
        mmatrB.SetElement(0, 18, 4 * (zeta3 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                               (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) +
                                      zeta1 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                               (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                                     Jdet);
        mmatrB.SetElement(0, 21, 4 * (zeta1 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                               (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                                      zeta4 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                               (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                                     Jdet);
        mmatrB.SetElement(0, 24, 4 * (zeta2 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                               (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                                      zeta4 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                               (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                                     Jdet);
        mmatrB.SetElement(0, 27, 4 * (zeta3 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                               (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                                      zeta4 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                               (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                                     Jdet);

        mmatrB.SetElement(1, 1, (4 * zeta1 - 1) *
                                    ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                     (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) /
                                    Jdet);
        mmatrB.SetElement(1, 4, (4 * zeta2 - 1) *
                                    ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                     (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) /
                                    Jdet);
        mmatrB.SetElement(1, 7, (4 * zeta3 - 1) *
                                    ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                     (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) /
                                    Jdet);
        mmatrB.SetElement(1, 10, (4 * zeta4 - 1) *
                                     ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                      (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) /
                                     Jdet);
        mmatrB.SetElement(1, 13, 4 * (zeta1 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) +
                                      zeta2 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                               (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                                     Jdet);
        mmatrB.SetElement(1, 16, 4 * (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                               (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) +
                                      zeta3 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                                     Jdet);
        mmatrB.SetElement(1, 19, 4 * (zeta3 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                               (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) +
                                      zeta1 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                               (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                                     Jdet);
        mmatrB.SetElement(1, 22, 4 * (zeta1 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                                      zeta4 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                               (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                                     Jdet);
        mmatrB.SetElement(1, 25, 4 * (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                                      zeta4 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                                     Jdet);
        mmatrB.SetElement(1, 28, 4 * (zeta3 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                                      zeta4 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                               (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                                     Jdet);

        mmatrB.SetElement(2, 2, (4 * zeta1 - 1) *
                                    ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                     (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) /
                                    Jdet);
        mmatrB.SetElement(2, 5, (4 * zeta2 - 1) *
                                    ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                     (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) /
                                    Jdet);
        mmatrB.SetElement(2, 8, (4 * zeta3 - 1) *
                                    ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                     (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) /
                                    Jdet);
        mmatrB.SetElement(2, 11, (4 * zeta4 - 1) *
                                     ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                      (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) /
                                     Jdet);
        mmatrB.SetElement(2, 14, 4 * (zeta1 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) +
                                      zeta2 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                               (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                                     Jdet);
        mmatrB.SetElement(2, 17, 4 * (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                               (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) +
                                      zeta3 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                                     Jdet);
        mmatrB.SetElement(2, 20, 4 * (zeta3 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                               (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) +
                                      zeta1 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                               (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3)))) /
                                     Jdet);
        mmatrB.SetElement(2, 23, 4 * (zeta1 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                                      zeta4 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                               (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                                     Jdet);
        mmatrB.SetElement(2, 26, 4 * (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                                      zeta4 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                               (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                                     Jdet);
        mmatrB.SetElement(2, 29, 4 * (zeta3 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                               (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                                      zeta4 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                               (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3)))) /
                                     Jdet);

        mmatrB.SetElement(3, 0, mmatrB(1, 1));
        mmatrB.SetElement(3, 1, mmatrB(0, 0));
        mmatrB.SetElement(3, 3, mmatrB(1, 4));
        mmatrB.SetElement(3, 4, mmatrB(0, 3));
        mmatrB.SetElement(3, 6, mmatrB(1, 7));
        mmatrB.SetElement(3, 7, mmatrB(0, 6));
        mmatrB.SetElement(3, 9, mmatrB(1, 10));
        mmatrB.SetElement(3, 10, mmatrB(0, 9));
        mmatrB.SetElement(3, 12, mmatrB(1, 13));
        mmatrB.SetElement(3, 13, mmatrB(0, 12));
        mmatrB.SetElement(3, 15, mmatrB(1, 16));
        mmatrB.SetElement(3, 16, mmatrB(0, 15));
        mmatrB.SetElement(3, 18, mmatrB(1, 19));
        mmatrB.SetElement(3, 19, mmatrB(0, 18));
        mmatrB.SetElement(3, 21, mmatrB(1, 22));
        mmatrB.SetElement(3, 22, mmatrB(0, 21));
        mmatrB.SetElement(3, 24, mmatrB(1, 25));
        mmatrB.SetElement(3, 25, mmatrB(0, 24));
        mmatrB.SetElement(3, 27, mmatrB(1, 28));
        mmatrB.SetElement(3, 28, mmatrB(0, 27));

        mmatrB.SetElement(4, 1, mmatrB(2, 2));
        mmatrB.SetElement(4, 2, mmatrB(1, 1));
        mmatrB.SetElement(4, 4, mmatrB(2, 5));
        mmatrB.SetElement(4, 5, mmatrB(1, 4));
        mmatrB.SetElement(4, 7, mmatrB(2, 8));
        mmatrB.SetElement(4, 8, mmatrB(1, 7));
        mmatrB.SetElement(4, 10, mmatrB(2, 11));
        mmatrB.SetElement(4, 11, mmatrB(1, 10));
        mmatrB.SetElement(4, 13, mmatrB(2, 14));
        mmatrB.SetElement(4, 14, mmatrB(1, 13));
        mmatrB.SetElement(4, 16, mmatrB(2, 17));
        mmatrB.SetElement(4, 17, mmatrB(1, 16));
        mmatrB.SetElement(4, 19, mmatrB(2, 20));
        mmatrB.SetElement(4, 20, mmatrB(1, 19));
        mmatrB.SetElement(4, 22, mmatrB(2, 23));
        mmatrB.SetElement(4, 23, mmatrB(1, 22));
        mmatrB.SetElement(4, 25, mmatrB(2, 26));
        mmatrB.SetElement(4, 26, mmatrB(1, 25));
        mmatrB.SetElement(4, 28, mmatrB(2, 29));
        mmatrB.SetElement(4, 29, mmatrB(1, 28));

        mmatrB.SetElement(5, 0, mmatrB(2, 2));
        mmatrB.SetElement(5, 2, mmatrB(0, 0));
        mmatrB.SetElement(5, 3, mmatrB(2, 5));
        mmatrB.SetElement(5, 5, mmatrB(0, 3));
        mmatrB.SetElement(5, 6, mmatrB(2, 8));
        mmatrB.SetElement(5, 8, mmatrB(0, 6));
        mmatrB.SetElement(5, 9, mmatrB(2, 11));
        mmatrB.SetElement(5, 11, mmatrB(0, 9));
        mmatrB.SetElement(5, 12, mmatrB(2, 14));
        mmatrB.SetElement(5, 14, mmatrB(0, 12));
        mmatrB.SetElement(5, 15, mmatrB(2, 17));
        mmatrB.SetElement(5, 17, mmatrB(0, 15));
        mmatrB.SetElement(5, 18, mmatrB(2, 20));
        mmatrB.SetElement(5, 20, mmatrB(0, 18));
        mmatrB.SetElement(5, 21, mmatrB(2, 23));
        mmatrB.SetElement(5, 23, mmatrB(0, 21));
        mmatrB.SetElement(5, 24, mmatrB(2, 26));
        mmatrB.SetElement(5, 26, mmatrB(0, 24));
        mmatrB.SetElement(5, 27, mmatrB(2, 29));
        mmatrB.SetElement(5, 29, mmatrB(0, 27));
        mmatrB.MatrScale(2);
    }

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = sum (w_i * [B]' * [D] * [B])
    ///
    virtual void ComputeStiffnessMatrix() {
        // for speeding up corotational, used later:
        // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
        //     [ 1    1    1    1    ]
        mM.PasteVector(nodes[0]->GetX0(), 0, 0);
        mM.PasteVector(nodes[1]->GetX0(), 0, 1);
        mM.PasteVector(nodes[2]->GetX0(), 0, 2);
        mM.PasteVector(nodes[3]->GetX0(), 0, 3);
        mM(3, 0) = 1.0;
        mM(3, 1) = 1.0;
        mM(3, 2) = 1.0;
        mM(3, 3) = 1.0;
        mM.MatrInverse();

        //========================
        // Exact Integration (4 Gp)
        //========================
        double zeta1, zeta2, zeta3, zeta4;
        double JacobianDet;
        ChMatrixDynamic<> temp;
        ChMatrixDynamic<> BT;

        zeta1 = 0.58541020;
        zeta2 = 0.1381966;
        zeta3 = 0.1381966;
        zeta4 = 0.1381966;

        ComputeMatrB(this->MatrB[0], zeta1, zeta2, zeta3, zeta4, JacobianDet);
        BT = MatrB[0];
        BT.MatrTranspose();
        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[0]);
        temp.MatrScale(JacobianDet / 6);
        // Gauss integration weight = 1*1/4*1/4*1/4
        temp.MatrDivScale(16);
        StiffnessMatrix = temp;

        zeta1 = 0.1381966;
        zeta2 = 0.58541020;
        zeta3 = 0.1381966;
        zeta4 = 0.1381966;

        ComputeMatrB(this->MatrB[1], zeta1, zeta2, zeta3, zeta4, JacobianDet);
        BT = MatrB[1];
        BT.MatrTranspose();
        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[1]);
        temp.MatrScale(JacobianDet / 6);
        // Gauss integration weight = 1*1/4*1/4*1/4
        temp.MatrDivScale(16);
        StiffnessMatrix.MatrAdd(StiffnessMatrix, temp);

        zeta1 = 0.1381966;
        zeta2 = 0.1381966;
        zeta3 = 0.58541020;
        zeta4 = 0.1381966;

        ComputeMatrB(this->MatrB[2], zeta1, zeta2, zeta3, zeta4, JacobianDet);
        BT = MatrB[2];
        BT.MatrTranspose();
        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[2]);
        temp.MatrScale(JacobianDet / 6);
        // Gauss integration weight = 1*1/4*1/4*1/4
        temp.MatrDivScale(16);
        StiffnessMatrix.MatrAdd(StiffnessMatrix, temp);

        zeta1 = 0.1381966;
        zeta2 = 0.1381966;
        zeta3 = 0.1381966;
        zeta4 = 0.58541020;

        ComputeMatrB(this->MatrB[3], zeta1, zeta2, zeta3, zeta4, JacobianDet);
        BT = MatrB[3];
        BT.MatrTranspose();
        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[3]);
        temp.MatrScale(JacobianDet / 6);
        // Gauss integration weight = 1*1/4*1/4*1/4
        temp.MatrDivScale(16);
        StiffnessMatrix.MatrAdd(StiffnessMatrix, temp);

        StiffnessMatrix.MatrDivScale(
            2);  //!!! => because the canonical interval is -1 ... +1,  but we want to integrate
                 //		 in 0 ... +1 -> we have to multiply by: b-a/2 ( = (1-0)/2 = 1/2)
    }

    /// Given the node ID, gets the 4 parameters of the shape function
    void GetParameterForNodeID(const int nodeID, double& z1, double& z2, double& z3, double& z4) {
        switch (nodeID) {
            case 0:
                z1 = 1;
                z2 = 0;
                z3 = 0;
                z4 = 0;
                break;
            case 1:
                z1 = 0;
                z2 = 1;
                z3 = 0;
                z4 = 0;
                break;
            case 2:
                z1 = 0;
                z2 = 0;
                z3 = 1;
                z4 = 0;
                break;
            case 3:
                z1 = 0;
                z2 = 0;
                z3 = 0;
                z4 = 1;
                break;
            case 4:
                z1 = 0.5;
                z2 = 0.5;
                z3 = 0;
                z4 = 0;
                break;
            case 5:
                z1 = 0;
                z2 = 0.5;
                z3 = 0.5;
                z4 = 0;
                break;
            case 6:
                z1 = 0.5;
                z2 = 0;
                z3 = 0.5;
                z4 = 0;
                break;
            case 7:
                z1 = 0.5;
                z2 = 0;
                z3 = 0;
                z4 = 0.5;
                break;
            case 8:
                z1 = 0;
                z2 = 0.5;
                z3 = 0;
                z4 = 0.5;
                break;
            case 9:
                z1 = 0;
                z2 = 0;
                z3 = 0.5;
                z4 = 0.5;
                break;
            default:
                break;
        }
    }

    /// Returns the strain tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain(double z1, double z2, double z3, double z4) {
        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(GetNdofs(), 1);
        this->GetStateBlock(displ);

        double JacobianDet;
        ChMatrixDynamic<> amatrB(6, GetNdofs());
        ComputeMatrB(amatrB, z1, z2, z3, z4, JacobianDet);

        ChStrainTensor<> mstrain;
        mstrain.MatrMultiply(amatrB, displ);
        return mstrain;
    }
    /// Returns the stress tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress(double z1, double z2, double z3, double z4) {
        ChStressTensor<> mstress;
        mstress.MatrMultiply(this->Material->Get_StressStrainMatrix(), this->GetStrain(z1, z2, z3, z4));
        return mstress;
    }

    virtual void SetupInitial(ChSystem* system) override {
        ComputeVolume();
        ComputeStiffnessMatrix();
    }

    // compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {
        // P = [ p_0  p_1  p_2  p_3 ]
        //     [ 1    1    1    1   ]
        ChMatrixNM<double, 4, 4> P;
        P.PasteVector(nodes[0]->pos, 0, 0);
        P.PasteVector(nodes[1]->pos, 0, 1);
        P.PasteVector(nodes[2]->pos, 0, 2);
        P.PasteVector(nodes[3]->pos, 0, 3);
        P(3, 0) = 1.0;
        P(3, 1) = 1.0;
        P(3, 2) = 1.0;
        P(3, 3) = 1.0;

        ChMatrix33<double> F;
        // F=P*mM (only upper-left 3x3 block!)
        double sum;
        for (int colres = 0; colres < 3; ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 4; ++col)
                    sum += (P(row, col)) * (mM(col, colres));
                F(row, colres) = sum;
            }
        ChMatrix33<> S;
        double det = ChPolarDecomposition<>::Compute(F, this->A, S, 1E-6);
        if (det < 0)
            this->A.MatrScale(-1.0);

        // GetLog() << "FEM rotation: \n" << A << "\n"
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == GetNdofs()) && (H.GetColumns() == GetNdofs()));

        // warp the local stiffness matrix K in order to obtain global
        // tangent stiffness CKCt:
        ChMatrixDynamic<> CK(GetNdofs(), GetNdofs());
        ChMatrixDynamic<> CKCt(GetNdofs(), GetNdofs());  // the global, corotated, K matrix
        ChMatrixCorotation<>::ComputeCK(StiffnessMatrix, this->A, 10, CK);
        ChMatrixCorotation<>::ComputeKCt(CK, this->A, 10, CKCt);

        // For K stiffness matrix and R damping matrix:

        double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

        CKCt.MatrScale(mkfactor);

        H.PasteMatrix(CKCt, 0, 0);

        // For M mass matrix:
        if (Mfactor) {
            double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / (double)this->GetNnodes();
            for (int id = 0; id < GetNdofs(); id++) {
                double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
                H(id, id) += amfactor * lumped_node_mass;
            }
        }
        //***TO DO*** better per-node lumping, or 30x30 consistent mass matrix.
    }

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        assert((Fi.GetRows() == GetNdofs()) && (Fi.GetColumns() == 1));

        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(GetNdofs(), 1);
        this->GetStateBlock(displ);

        // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
        ChMatrixDynamic<> FiK_local(GetNdofs(), 1);
        FiK_local.MatrMultiply(StiffnessMatrix, displ);

        displ.PasteVector(A.MatrT_x_Vect(nodes[0]->pos_dt), 0, 0);  // nodal speeds, local
        displ.PasteVector(A.MatrT_x_Vect(nodes[1]->pos_dt), 3, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[2]->pos_dt), 6, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[3]->pos_dt), 9, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[4]->pos_dt), 12, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[5]->pos_dt), 15, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[6]->pos_dt), 18, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[7]->pos_dt), 21, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[8]->pos_dt), 24, 0);
        displ.PasteVector(A.MatrT_x_Vect(nodes[9]->pos_dt), 27, 0);
        ChMatrixDynamic<> FiR_local(GetNdofs(), 1);
        FiR_local.MatrMultiply(StiffnessMatrix, displ);
        FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

        double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / (double)this->GetNnodes();
        displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM());  // reuse 'displ' for performance
        FiR_local.MatrInc(displ);
        //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

        FiK_local.MatrInc(FiR_local);

        FiK_local.MatrScale(-1.0);

        // Fi = C * Fi_local  with C block-diagonal rotations A
        ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 10, Fi);
    }

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() { return Material; }

    /// Get the partial derivatives matrix MatrB and the StiffnessMatrix
    ChMatrix<>& GetMatrB(int n) { return MatrB[n]; }
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 10 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 10 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos(), block_offset, 0);
        mD.PasteVector(this->nodes[1]->GetPos(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[2]->GetPos(), block_offset + 6, 0);
        mD.PasteVector(this->nodes[3]->GetPos(), block_offset + 9, 0);
        mD.PasteVector(this->nodes[4]->GetPos(), block_offset + 12, 0);
        mD.PasteVector(this->nodes[5]->GetPos(), block_offset + 15, 0);
        mD.PasteVector(this->nodes[6]->GetPos(), block_offset + 18, 0);
        mD.PasteVector(this->nodes[7]->GetPos(), block_offset + 21, 0);
        mD.PasteVector(this->nodes[8]->GetPos(), block_offset + 24, 0);
        mD.PasteVector(this->nodes[9]->GetPos(), block_offset + 27, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->nodes[1]->GetPos_dt(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[2]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->nodes[3]->GetPos_dt(), block_offset + 9, 0);
        mD.PasteVector(this->nodes[4]->GetPos_dt(), block_offset + 12, 0);
        mD.PasteVector(this->nodes[5]->GetPos_dt(), block_offset + 15, 0);
        mD.PasteVector(this->nodes[6]->GetPos_dt(), block_offset + 18, 0);
        mD.PasteVector(this->nodes[7]->GetPos_dt(), block_offset + 21, 0);
        mD.PasteVector(this->nodes[8]->GetPos_dt(), block_offset + 24, 0);
        mD.PasteVector(this->nodes[9]->GetPos_dt(), block_offset + 27, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<10; ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i*3  , x_new, x, off_v  + i*3  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 10; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < nodes.size(); ++i)
            mvars.push_back(&this->nodes[i]->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(
            N, U, V, W);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

        detJ = 6 * this->GetVolume();

        Qi(0) = N(0) * F(0);
        Qi(1) = N(0) * F(1);
        Qi(2) = N(0) * F(2);
        Qi(3) = N(1) * F(0);
        Qi(4) = N(1) * F(1);
        Qi(5) = N(1) * F(2);
        Qi(6) = N(2) * F(0);
        Qi(7) = N(2) * F(1);
        Qi(8) = N(2) * F(2);
        Qi(9) = N(3) * F(0);
        Qi(10) = N(3) * F(1);
        Qi(11) = N(3) * F(2);
        Qi(12) = N(4) * F(0);
        Qi(13) = N(4) * F(1);
        Qi(14) = N(4) * F(2);
        Qi(15) = N(5) * F(0);
        Qi(16) = N(5) * F(1);
        Qi(17) = N(5) * F(2);
        Qi(18) = N(6) * F(0);
        Qi(19) = N(6) * F(1);
        Qi(20) = N(6) * F(2);
        Qi(21) = N(7) * F(0);
        Qi(22) = N(7) * F(1);
        Qi(23) = N(7) * F(2);
        Qi(24) = N(8) * F(0);
        Qi(25) = N(8) * F(1);
        Qi(26) = N(8) * F(2);
        Qi(27) = N(9) * F(0);
        Qi(28) = N(9) * F(1);
        Qi(29) = N(9) * F(2);
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->Material->Get_density(); }

    /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
    /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
