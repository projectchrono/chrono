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
// Authors: Andrea Favali
// =============================================================================

#ifndef CHELEMENTHEXA20_H
#define CHELEMENTHEXA20_H

#include "chrono_fea/ChElementHexahedron.h"
#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Class for FEA elements of hexahedron type (isoparametric 3D bricks) with 20 nodes.
class ChApiFea ChElementHexa_20 : public ChElementHexahedron, public ChLoadableUVW {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyz> > nodes;
    std::shared_ptr<ChContinuumElastic> Material;
    // std::vector< ChMatrixDynamic<> > MatrB;		// matrices of shape function's partial derivatives (one for each
    // integration point)
    // we use a vector to keep in memory all the 27 matrices (-> 27 integr. point)
    // NO! each matrix is stored in the respective gauss point
    ChMatrixDynamic<> StiffnessMatrix;

  public:
    ChElementHexa_20();
    virtual ~ChElementHexa_20();

    virtual int GetNnodes() override { return 20; }
    virtual int GetNdofs() override { return 20 * 3; }
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
                          std::shared_ptr<ChNodeFEAxyz> nodeJ,
                          std::shared_ptr<ChNodeFEAxyz> nodeK,
                          std::shared_ptr<ChNodeFEAxyz> nodeL,
                          std::shared_ptr<ChNodeFEAxyz> nodeM,
                          std::shared_ptr<ChNodeFEAxyz> nodeN,
                          std::shared_ptr<ChNodeFEAxyz> nodeO,
                          std::shared_ptr<ChNodeFEAxyz> nodeP,
                          std::shared_ptr<ChNodeFEAxyz> nodeQ,
                          std::shared_ptr<ChNodeFEAxyz> nodeR,
                          std::shared_ptr<ChNodeFEAxyz> nodeS,
                          std::shared_ptr<ChNodeFEAxyz> nodeT) {
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
        nodes[10] = nodeK;
        nodes[11] = nodeL;
        nodes[12] = nodeM;
        nodes[13] = nodeN;
        nodes[14] = nodeO;
        nodes[15] = nodeP;
        nodes[16] = nodeQ;
        nodes[17] = nodeR;
        nodes[18] = nodeS;
        nodes[19] = nodeT;
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
        mvars.push_back(&nodes[10]->Variables());
        mvars.push_back(&nodes[11]->Variables());
        mvars.push_back(&nodes[12]->Variables());
        mvars.push_back(&nodes[13]->Variables());
        mvars.push_back(&nodes[14]->Variables());
        mvars.push_back(&nodes[15]->Variables());
        mvars.push_back(&nodes[16]->Variables());
        mvars.push_back(&nodes[17]->Variables());
        mvars.push_back(&nodes[18]->Variables());
        mvars.push_back(&nodes[19]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // QUADRATURE functions
    //

    virtual void SetDefaultIntegrationRule() { this->ir->SetIntOnCube(27, &this->GpVector); }

    virtual void SetReducedIntegrationRule() { this->ir->SetIntOnCube(8, &this->GpVector); }

    virtual void SetIntegrationRule(int nPoints) { this->ir->SetIntOnCube(nPoints, &this->GpVector); }

    //
    // FEA functions
    //

    /// Fills the N shape function matrix with the
    /// values of shape functions at r,s,t parametric coordinates, where
    /// each parameter is in [-1...+1] range.
    /// It stores the Ni(r,s,t) values in a 1 row, 20 columns matrix N.
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s, double t) {
        double rm = 1.0 - r;
        double rp = 1.0 + r;
        double sm = 1.0 - s;
        double sp = 1.0 + s;
        double tm = 1.0 - t;
        double tp = 1.0 + t;
        double r2 = 1.0 - r * r;
        double s2 = 1.0 - s * s;
        double t2 = 1.0 - t * t;

        // The eight corner points
        N(0) = 0.125 * rm * sm * tm * (-r - s - t - 2.0);
        N(1) = 0.125 * rp * sm * tm * (r - s - t - 2.0);
        N(2) = 0.125 * rp * sp * tm * (r + s - t - 2.0);
        N(3) = 0.125 * rm * sp * tm * (-r + s - t - 2.0);
        N(4) = 0.125 * rm * sm * tp * (-r - s + t - 2.0);
        N(5) = 0.125 * rp * sm * tp * (r - s + t - 2.0);
        N(6) = 0.125 * rp * sp * tp * (r + s + t - 2.0);
        N(7) = 0.125 * rm * sp * tp * (-r + s + t - 2.0);

        // The mid-edge nodes
        N(8) = 0.25 * r2 * sm * tm;
        N(9) = 0.25 * s2 * rp * tm;
        N(19) = 0.25 * r2 * sp * tm;
        N(11) = 0.25 * s2 * rm * tm;
        N(12) = 0.25 * r2 * sm * tp;
        N(13) = 0.25 * s2 * rp * tp;
        N(14) = 0.25 * r2 * sp * tp;
        N(15) = 0.25 * s2 * rm * tp;
        N(16) = 0.25 * t2 * rm * sm;
        N(17) = 0.25 * t2 * rp * sm;
        N(18) = 0.25 * t2 * rp * sp;
        N(19) = 0.25 * t2 * rm * sp;
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

    /// Puts inside 'Jacobian' and 'J1' the Jacobian matrix and the shape functions derivatives matrix of the element
    /// The vector "coord" contains the natural coordinates of the integration point
    /// in case of hexahedral elements natural coords vary in the classical range -1 ... +1
    virtual void ComputeJacobian(ChMatrixDynamic<>& Jacobian, ChMatrixDynamic<>& J1, ChVector<> coord) {
        ChMatrixDynamic<> J2(20, 3);

        J1.SetElement(0, 0, -(1 - coord.y()) * (1 - coord.z()) * (-1 - 2 * coord.x() - coord.y() - coord.z()) / 8);
        J1.SetElement(0, 1, +(1 - coord.y()) * (1 - coord.z()) * (-1 + 2 * coord.x() - coord.y() - coord.z()) / 8);
        J1.SetElement(0, 2, +(1 + coord.y()) * (1 - coord.z()) * (-1 + 2 * coord.x() + coord.y() - coord.z()) / 8);
        J1.SetElement(0, 3, -(1 + coord.y()) * (1 - coord.z()) * (-1 - 2 * coord.x() + coord.y() - coord.z()) / 8);
        J1.SetElement(0, 4, -(1 - coord.y()) * (1 + coord.z()) * (-1 - 2 * coord.x() - coord.y() + coord.z()) / 8);
        J1.SetElement(0, 5, +(1 - coord.y()) * (1 + coord.z()) * (-1 + 2 * coord.x() - coord.y() + coord.z()) / 8);
        J1.SetElement(0, 6, +(1 + coord.y()) * (1 + coord.z()) * (-1 + 2 * coord.x() + coord.y() + coord.z()) / 8);
        J1.SetElement(0, 7, -(1 + coord.y()) * (1 + coord.z()) * (-1 - 2 * coord.x() + coord.y() + coord.z()) / 8);
        J1.SetElement(0, 8, coord.x() * (1 - coord.y()) * (1 - coord.z()) / (-2));
        J1.SetElement(0, 9, +(1 - coord.y() * coord.y()) * (1 - coord.z()) / 4);
        J1.SetElement(0, 10, coord.x() * (1 + coord.y()) * (1 - coord.z()) / (-2));
        J1.SetElement(0, 11, -(1 - coord.y() * coord.y()) * (1 - coord.z()) / 4);
        J1.SetElement(0, 12, coord.x() * (1 - coord.y()) * (1 + coord.z()) / (-2));
        J1.SetElement(0, 13, +(1 - coord.y() * coord.y()) * (1 + coord.z()) / 4);
        J1.SetElement(0, 14, coord.x() * (1 + coord.y()) * (1 + coord.z()) / (-2));
        J1.SetElement(0, 15, -(1 - coord.y() * coord.y()) * (1 + coord.z()) / 4);
        J1.SetElement(0, 16, +(1 - coord.y()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(0, 17, +(1 + coord.y()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(0, 18, -(1 + coord.y()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(0, 19, -(1 - coord.y()) * (1 - coord.z() * coord.z()) / 4);

        J1.SetElement(1, 0, -(1 - coord.x()) * (1 - coord.z()) * (-1 - coord.x() - 2 * coord.y() - coord.z()) / 8);
        J1.SetElement(1, 1, -(1 + coord.x()) * (1 - coord.z()) * (-1 + coord.x() - 2 * coord.y() - coord.z()) / 8);
        J1.SetElement(1, 2, +(1 + coord.x()) * (1 - coord.z()) * (-1 + coord.x() + 2 * coord.y() - coord.z()) / 8);
        J1.SetElement(1, 3, +(1 - coord.x()) * (1 - coord.z()) * (-1 - coord.x() + 2 * coord.y() - coord.z()) / 8);
        J1.SetElement(1, 4, -(1 - coord.x()) * (1 + coord.z()) * (-1 - coord.x() - 2 * coord.y() + coord.z()) / 8);
        J1.SetElement(1, 5, -(1 + coord.x()) * (1 + coord.z()) * (-1 + coord.x() - 2 * coord.y() + coord.z()) / 8);
        J1.SetElement(1, 6, +(1 + coord.x()) * (1 + coord.z()) * (-1 + coord.x() + 2 * coord.y() + coord.z()) / 8);
        J1.SetElement(1, 7, +(1 - coord.x()) * (1 + coord.z()) * (-1 - coord.x() + 2 * coord.y() + coord.z()) / 8);
        J1.SetElement(1, 8, -(1 - coord.x() * coord.x()) * (1 - coord.z()) / 4);
        J1.SetElement(1, 9, coord.y() * (1 + coord.x()) * (1 - coord.z()) / (-2));
        J1.SetElement(1, 10, +(1 - coord.x() * coord.x()) * (1 - coord.z()) / 4);
        J1.SetElement(1, 11, coord.y() * (1 - coord.x()) * (1 - coord.z()) / (-2));
        J1.SetElement(1, 12, -(1 - coord.x() * coord.x()) * (1 + coord.z()) / 4);
        J1.SetElement(1, 13, coord.y() * (1 + coord.x()) * (1 + coord.z()) / (-2));
        J1.SetElement(1, 14, +(1 - coord.x() * coord.x()) * (1 + coord.z()) / 4);
        J1.SetElement(1, 15, coord.y() * (1 - coord.x()) * (1 + coord.z()) / (-2));
        J1.SetElement(1, 16, -(1 + coord.x()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(1, 17, +(1 + coord.x()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(1, 18, +(1 - coord.x()) * (1 - coord.z() * coord.z()) / 4);
        J1.SetElement(1, 19, -(1 - coord.x()) * (1 - coord.z() * coord.z()) / 4);

        J1.SetElement(2, 0, -(1 - coord.x()) * (1 - coord.y()) * (-1 - coord.x() - coord.y() - 2 * coord.z()) / 8);
        J1.SetElement(2, 1, -(1 + coord.x()) * (1 - coord.y()) * (-1 + coord.x() - coord.y() - 2 * coord.z()) / 8);
        J1.SetElement(2, 2, -(1 + coord.x()) * (1 + coord.y()) * (-1 + coord.x() + coord.y() - 2 * coord.z()) / 8);
        J1.SetElement(2, 3, -(1 - coord.x()) * (1 + coord.y()) * (-1 - coord.x() + coord.y() - 2 * coord.z()) / 8);
        J1.SetElement(2, 4, +(1 - coord.x()) * (1 - coord.y()) * (-1 - coord.x() - coord.y() + 2 * coord.z()) / 8);
        J1.SetElement(2, 5, +(1 + coord.x()) * (1 - coord.y()) * (-1 + coord.x() - coord.y() + 2 * coord.z()) / 8);
        J1.SetElement(2, 6, +(1 + coord.x()) * (1 + coord.y()) * (-1 + coord.x() + coord.y() + 2 * coord.z()) / 8);
        J1.SetElement(2, 7, +(1 - coord.x()) * (1 + coord.y()) * (-1 - coord.x() + coord.y() + 2 * coord.z()) / 8);
        J1.SetElement(2, 8, -(1 - coord.x() * coord.x()) * (1 - coord.y()) / 4);
        J1.SetElement(2, 9, -(1 + coord.x()) * (1 - coord.y() * coord.y()) / 4);
        J1.SetElement(2, 10, -(1 - coord.x() * coord.x()) * (1 + coord.y()) / 4);
        J1.SetElement(2, 11, -(1 - coord.x()) * (1 - coord.y() * coord.y()) / 4);
        J1.SetElement(2, 12, +(1 - coord.x() * coord.x()) * (1 - coord.y()) / 4);
        J1.SetElement(2, 13, +(1 + coord.x()) * (1 - coord.y() * coord.y()) / 4);
        J1.SetElement(2, 14, +(1 - coord.x() * coord.x()) * (1 + coord.y()) / 4);
        J1.SetElement(2, 15, +(1 - coord.x()) * (1 - coord.y() * coord.y()) / 4);
        J1.SetElement(2, 16, coord.z() * (1 + coord.x()) * (1 - coord.y()) / (-2));
        J1.SetElement(2, 17, coord.z() * (1 + coord.x()) * (1 + coord.y()) / (-2));
        J1.SetElement(2, 18, coord.z() * (1 - coord.x()) * (1 + coord.y()) / (-2));
        J1.SetElement(2, 19, coord.z() * (1 - coord.x()) * (1 - coord.y()) / (-2));

        J2.SetElement(0, 0, nodes[0]->GetX0().x());
        J2.SetElement(1, 0, nodes[1]->GetX0().x());
        J2.SetElement(2, 0, nodes[2]->GetX0().x());
        J2.SetElement(3, 0, nodes[3]->GetX0().x());
        J2.SetElement(4, 0, nodes[4]->GetX0().x());
        J2.SetElement(5, 0, nodes[5]->GetX0().x());
        J2.SetElement(6, 0, nodes[6]->GetX0().x());
        J2.SetElement(7, 0, nodes[7]->GetX0().x());
        J2.SetElement(8, 0, nodes[8]->GetX0().x());
        J2.SetElement(9, 0, nodes[9]->GetX0().x());
        J2.SetElement(10, 0, nodes[10]->GetX0().x());
        J2.SetElement(11, 0, nodes[11]->GetX0().x());
        J2.SetElement(12, 0, nodes[12]->GetX0().x());
        J2.SetElement(13, 0, nodes[13]->GetX0().x());
        J2.SetElement(14, 0, nodes[14]->GetX0().x());
        J2.SetElement(15, 0, nodes[15]->GetX0().x());
        J2.SetElement(16, 0, nodes[16]->GetX0().x());
        J2.SetElement(17, 0, nodes[17]->GetX0().x());
        J2.SetElement(18, 0, nodes[18]->GetX0().x());
        J2.SetElement(19, 0, nodes[19]->GetX0().x());

        J2.SetElement(0, 1, nodes[0]->GetX0().y());
        J2.SetElement(1, 1, nodes[1]->GetX0().y());
        J2.SetElement(2, 1, nodes[2]->GetX0().y());
        J2.SetElement(3, 1, nodes[3]->GetX0().y());
        J2.SetElement(4, 1, nodes[4]->GetX0().y());
        J2.SetElement(5, 1, nodes[5]->GetX0().y());
        J2.SetElement(6, 1, nodes[6]->GetX0().y());
        J2.SetElement(7, 1, nodes[7]->GetX0().y());
        J2.SetElement(8, 1, nodes[8]->GetX0().y());
        J2.SetElement(9, 1, nodes[9]->GetX0().y());
        J2.SetElement(10, 1, nodes[10]->GetX0().y());
        J2.SetElement(11, 1, nodes[11]->GetX0().y());
        J2.SetElement(12, 1, nodes[12]->GetX0().y());
        J2.SetElement(13, 1, nodes[13]->GetX0().y());
        J2.SetElement(14, 1, nodes[14]->GetX0().y());
        J2.SetElement(15, 1, nodes[15]->GetX0().y());
        J2.SetElement(16, 1, nodes[16]->GetX0().y());
        J2.SetElement(17, 1, nodes[17]->GetX0().y());
        J2.SetElement(18, 1, nodes[18]->GetX0().y());
        J2.SetElement(19, 1, nodes[19]->GetX0().y());

        J2.SetElement(0, 2, nodes[0]->GetX0().z());
        J2.SetElement(1, 2, nodes[1]->GetX0().z());
        J2.SetElement(2, 2, nodes[2]->GetX0().z());
        J2.SetElement(3, 2, nodes[3]->GetX0().z());
        J2.SetElement(4, 2, nodes[4]->GetX0().z());
        J2.SetElement(5, 2, nodes[5]->GetX0().z());
        J2.SetElement(6, 2, nodes[6]->GetX0().z());
        J2.SetElement(7, 2, nodes[7]->GetX0().z());
        J2.SetElement(8, 2, nodes[8]->GetX0().z());
        J2.SetElement(9, 2, nodes[9]->GetX0().z());
        J2.SetElement(10, 2, nodes[10]->GetX0().z());
        J2.SetElement(11, 2, nodes[11]->GetX0().z());
        J2.SetElement(12, 2, nodes[12]->GetX0().z());
        J2.SetElement(13, 2, nodes[13]->GetX0().z());
        J2.SetElement(14, 2, nodes[14]->GetX0().z());
        J2.SetElement(15, 2, nodes[15]->GetX0().z());
        J2.SetElement(16, 2, nodes[16]->GetX0().z());
        J2.SetElement(17, 2, nodes[17]->GetX0().z());
        J2.SetElement(18, 2, nodes[18]->GetX0().z());
        J2.SetElement(19, 2, nodes[19]->GetX0().z());

        Jacobian.MatrMultiply(J1, J2);
    }

    /// Computes the matrix of partial derivatives and puts data in "MatrB"
    ///	evaluated at natural coordinates zeta1,...,zeta4 . Also computes determinant of jacobian.
    /// note: in case of hexahedral elements natural coord. vary in the range -1 ... +1
    virtual void ComputeMatrB(ChMatrixDynamic<>& MatrB, double zeta1, double zeta2, double zeta3, double& JacobianDet) {
        ChMatrixDynamic<> Jacobian(3, 3);
        ChMatrixDynamic<> J1(3, 20);
        ComputeJacobian(Jacobian, J1, ChVector<>(zeta1, zeta2, zeta3));

        double Jdet = Jacobian.Det();
        JacobianDet = Jdet;  // !!! store the Jacobian Determinant: needed for the integration

        ChMatrixDynamic<> Jinv = Jacobian;
        Jinv.MatrInverse();
        ChMatrixDynamic<> Btemp(3, 20);
        Btemp.MatrMultiply(Jinv, J1);
        MatrB.Resize(6, 60);  // Remember to resize the matrix!

        MatrB.SetElement(0, 0, Btemp(0, 0));
        MatrB.SetElement(0, 3, Btemp(0, 1));
        MatrB.SetElement(0, 6, Btemp(0, 2));
        MatrB.SetElement(0, 9, Btemp(0, 3));
        MatrB.SetElement(0, 12, Btemp(0, 4));
        MatrB.SetElement(0, 15, Btemp(0, 5));
        MatrB.SetElement(0, 18, Btemp(0, 6));
        MatrB.SetElement(0, 21, Btemp(0, 7));
        MatrB.SetElement(0, 24, Btemp(0, 8));
        MatrB.SetElement(0, 27, Btemp(0, 9));
        MatrB.SetElement(0, 30, Btemp(0, 10));
        MatrB.SetElement(0, 33, Btemp(0, 11));
        MatrB.SetElement(0, 36, Btemp(0, 12));
        MatrB.SetElement(0, 39, Btemp(0, 13));
        MatrB.SetElement(0, 42, Btemp(0, 14));
        MatrB.SetElement(0, 45, Btemp(0, 15));
        MatrB.SetElement(0, 48, Btemp(0, 16));
        MatrB.SetElement(0, 51, Btemp(0, 17));
        MatrB.SetElement(0, 54, Btemp(0, 18));
        MatrB.SetElement(0, 57, Btemp(0, 19));

        MatrB.SetElement(1, 1, Btemp(1, 0));
        MatrB.SetElement(1, 4, Btemp(1, 1));
        MatrB.SetElement(1, 7, Btemp(1, 2));
        MatrB.SetElement(1, 10, Btemp(1, 3));
        MatrB.SetElement(1, 13, Btemp(1, 4));
        MatrB.SetElement(1, 16, Btemp(1, 5));
        MatrB.SetElement(1, 19, Btemp(1, 6));
        MatrB.SetElement(1, 22, Btemp(1, 7));
        MatrB.SetElement(1, 25, Btemp(1, 8));
        MatrB.SetElement(1, 28, Btemp(1, 9));
        MatrB.SetElement(1, 31, Btemp(1, 10));
        MatrB.SetElement(1, 34, Btemp(1, 11));
        MatrB.SetElement(1, 37, Btemp(1, 12));
        MatrB.SetElement(1, 40, Btemp(1, 13));
        MatrB.SetElement(1, 43, Btemp(1, 14));
        MatrB.SetElement(1, 46, Btemp(1, 15));
        MatrB.SetElement(1, 49, Btemp(1, 16));
        MatrB.SetElement(1, 52, Btemp(1, 17));
        MatrB.SetElement(1, 55, Btemp(1, 18));
        MatrB.SetElement(1, 58, Btemp(1, 19));

        MatrB.SetElement(2, 2, Btemp(2, 0));
        MatrB.SetElement(2, 5, Btemp(2, 1));
        MatrB.SetElement(2, 8, Btemp(2, 2));
        MatrB.SetElement(2, 11, Btemp(2, 3));
        MatrB.SetElement(2, 14, Btemp(2, 4));
        MatrB.SetElement(2, 17, Btemp(2, 5));
        MatrB.SetElement(2, 20, Btemp(2, 6));
        MatrB.SetElement(2, 23, Btemp(2, 7));
        MatrB.SetElement(2, 26, Btemp(2, 8));
        MatrB.SetElement(2, 29, Btemp(2, 9));
        MatrB.SetElement(2, 32, Btemp(2, 10));
        MatrB.SetElement(2, 35, Btemp(2, 11));
        MatrB.SetElement(2, 38, Btemp(2, 12));
        MatrB.SetElement(2, 41, Btemp(2, 13));
        MatrB.SetElement(2, 44, Btemp(2, 14));
        MatrB.SetElement(2, 47, Btemp(2, 15));
        MatrB.SetElement(2, 50, Btemp(2, 16));
        MatrB.SetElement(2, 53, Btemp(2, 17));
        MatrB.SetElement(2, 56, Btemp(2, 18));
        MatrB.SetElement(2, 59, Btemp(2, 19));

        MatrB.SetElement(3, 0, Btemp(1, 0));
        MatrB.SetElement(3, 1, Btemp(0, 0));
        MatrB.SetElement(3, 3, Btemp(1, 1));
        MatrB.SetElement(3, 4, Btemp(0, 1));
        MatrB.SetElement(3, 6, Btemp(1, 2));
        MatrB.SetElement(3, 7, Btemp(0, 2));
        MatrB.SetElement(3, 9, Btemp(1, 3));
        MatrB.SetElement(3, 10, Btemp(0, 3));
        MatrB.SetElement(3, 12, Btemp(1, 4));
        MatrB.SetElement(3, 13, Btemp(0, 4));
        MatrB.SetElement(3, 15, Btemp(1, 5));
        MatrB.SetElement(3, 16, Btemp(0, 5));
        MatrB.SetElement(3, 18, Btemp(1, 6));
        MatrB.SetElement(3, 19, Btemp(0, 6));
        MatrB.SetElement(3, 21, Btemp(1, 7));
        MatrB.SetElement(3, 22, Btemp(0, 7));
        MatrB.SetElement(3, 24, Btemp(1, 8));
        MatrB.SetElement(3, 25, Btemp(0, 8));
        MatrB.SetElement(3, 27, Btemp(1, 9));
        MatrB.SetElement(3, 28, Btemp(0, 9));
        MatrB.SetElement(3, 30, Btemp(1, 10));
        MatrB.SetElement(3, 31, Btemp(0, 10));
        MatrB.SetElement(3, 33, Btemp(1, 11));
        MatrB.SetElement(3, 34, Btemp(0, 11));
        MatrB.SetElement(3, 36, Btemp(1, 12));
        MatrB.SetElement(3, 37, Btemp(0, 12));
        MatrB.SetElement(3, 39, Btemp(1, 13));
        MatrB.SetElement(3, 40, Btemp(0, 13));
        MatrB.SetElement(3, 42, Btemp(1, 14));
        MatrB.SetElement(3, 43, Btemp(0, 14));
        MatrB.SetElement(3, 45, Btemp(1, 15));
        MatrB.SetElement(3, 46, Btemp(0, 15));
        MatrB.SetElement(3, 48, Btemp(1, 16));
        MatrB.SetElement(3, 49, Btemp(0, 16));
        MatrB.SetElement(3, 51, Btemp(1, 17));
        MatrB.SetElement(3, 52, Btemp(0, 17));
        MatrB.SetElement(3, 54, Btemp(1, 18));
        MatrB.SetElement(3, 55, Btemp(0, 18));
        MatrB.SetElement(3, 57, Btemp(1, 19));
        MatrB.SetElement(3, 58, Btemp(0, 19));

        MatrB.SetElement(4, 1, Btemp(2, 0));
        MatrB.SetElement(4, 2, Btemp(1, 0));
        MatrB.SetElement(4, 4, Btemp(2, 1));
        MatrB.SetElement(4, 5, Btemp(1, 1));
        MatrB.SetElement(4, 7, Btemp(2, 2));
        MatrB.SetElement(4, 8, Btemp(1, 2));
        MatrB.SetElement(4, 10, Btemp(2, 3));
        MatrB.SetElement(4, 11, Btemp(1, 3));
        MatrB.SetElement(4, 13, Btemp(2, 4));
        MatrB.SetElement(4, 14, Btemp(1, 4));
        MatrB.SetElement(4, 16, Btemp(2, 5));
        MatrB.SetElement(4, 17, Btemp(1, 5));
        MatrB.SetElement(4, 19, Btemp(2, 6));
        MatrB.SetElement(4, 20, Btemp(1, 6));
        MatrB.SetElement(4, 22, Btemp(2, 7));
        MatrB.SetElement(4, 23, Btemp(1, 7));
        MatrB.SetElement(4, 25, Btemp(2, 8));
        MatrB.SetElement(4, 26, Btemp(1, 8));
        MatrB.SetElement(4, 28, Btemp(2, 9));
        MatrB.SetElement(4, 29, Btemp(1, 9));
        MatrB.SetElement(4, 31, Btemp(2, 10));
        MatrB.SetElement(4, 32, Btemp(1, 10));
        MatrB.SetElement(4, 34, Btemp(2, 11));
        MatrB.SetElement(4, 35, Btemp(1, 11));
        MatrB.SetElement(4, 37, Btemp(2, 12));
        MatrB.SetElement(4, 38, Btemp(1, 12));
        MatrB.SetElement(4, 40, Btemp(2, 13));
        MatrB.SetElement(4, 41, Btemp(1, 13));
        MatrB.SetElement(4, 43, Btemp(2, 14));
        MatrB.SetElement(4, 44, Btemp(1, 14));
        MatrB.SetElement(4, 46, Btemp(2, 15));
        MatrB.SetElement(4, 47, Btemp(1, 15));
        MatrB.SetElement(4, 49, Btemp(2, 16));
        MatrB.SetElement(4, 50, Btemp(1, 16));
        MatrB.SetElement(4, 52, Btemp(2, 17));
        MatrB.SetElement(4, 53, Btemp(1, 17));
        MatrB.SetElement(4, 55, Btemp(2, 18));
        MatrB.SetElement(4, 56, Btemp(1, 18));
        MatrB.SetElement(4, 58, Btemp(2, 19));
        MatrB.SetElement(4, 59, Btemp(1, 19));

        MatrB.SetElement(5, 0, Btemp(2, 0));
        MatrB.SetElement(5, 2, Btemp(0, 0));
        MatrB.SetElement(5, 3, Btemp(2, 1));
        MatrB.SetElement(5, 5, Btemp(0, 1));
        MatrB.SetElement(5, 6, Btemp(2, 2));
        MatrB.SetElement(5, 8, Btemp(0, 2));
        MatrB.SetElement(5, 9, Btemp(2, 3));
        MatrB.SetElement(5, 11, Btemp(0, 3));
        MatrB.SetElement(5, 12, Btemp(2, 4));
        MatrB.SetElement(5, 14, Btemp(0, 4));
        MatrB.SetElement(5, 15, Btemp(2, 5));
        MatrB.SetElement(5, 17, Btemp(0, 5));
        MatrB.SetElement(5, 18, Btemp(2, 6));
        MatrB.SetElement(5, 20, Btemp(0, 6));
        MatrB.SetElement(5, 21, Btemp(2, 7));
        MatrB.SetElement(5, 23, Btemp(0, 7));
        MatrB.SetElement(5, 24, Btemp(2, 8));
        MatrB.SetElement(5, 26, Btemp(0, 8));
        MatrB.SetElement(5, 27, Btemp(2, 9));
        MatrB.SetElement(5, 29, Btemp(0, 9));
        MatrB.SetElement(5, 30, Btemp(2, 10));
        MatrB.SetElement(5, 32, Btemp(0, 10));
        MatrB.SetElement(5, 33, Btemp(2, 11));
        MatrB.SetElement(5, 35, Btemp(0, 11));
        MatrB.SetElement(5, 36, Btemp(2, 12));
        MatrB.SetElement(5, 38, Btemp(0, 12));
        MatrB.SetElement(5, 39, Btemp(2, 13));
        MatrB.SetElement(5, 41, Btemp(0, 13));
        MatrB.SetElement(5, 42, Btemp(2, 14));
        MatrB.SetElement(5, 44, Btemp(0, 14));
        MatrB.SetElement(5, 45, Btemp(2, 15));
        MatrB.SetElement(5, 47, Btemp(0, 15));
        MatrB.SetElement(5, 48, Btemp(2, 16));
        MatrB.SetElement(5, 50, Btemp(0, 16));
        MatrB.SetElement(5, 51, Btemp(2, 17));
        MatrB.SetElement(5, 53, Btemp(0, 17));
        MatrB.SetElement(5, 54, Btemp(2, 18));
        MatrB.SetElement(5, 56, Btemp(0, 18));
        MatrB.SetElement(5, 57, Btemp(2, 19));
        MatrB.SetElement(5, 59, Btemp(0, 19));
    }

    /// Computes the matrix of partial derivatives and puts data in "GaussPt"
    ///	Stores the determinant of the jacobian in "JacobianDet"
    virtual void ComputeMatrB(ChGaussPoint* GaussPt, double& JacobianDet) {
        this->ComputeMatrB(*(GaussPt->MatrB), GaussPt->GetLocalCoordinates().x(), GaussPt->GetLocalCoordinates().y(),
                           GaussPt->GetLocalCoordinates().z(), JacobianDet);
    }

    /// Computes the global STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    /// The number of Gauss Point is defined by SetIntegrationRule function (default: 27 Gp)
    virtual void ComputeStiffnessMatrix() {
        double Jdet;
        ChMatrixDynamic<>* temp = new ChMatrixDynamic<>;
        ChMatrixDynamic<> BT;
        this->Volume = 0;

        for (unsigned int i = 0; i < GpVector.size(); i++) {
            ComputeMatrB(GpVector[i], Jdet);
            BT = *GpVector[i]->MatrB;
            BT.MatrTranspose();
            *temp = (BT * Material->Get_StressStrainMatrix() * *(GpVector[i]->MatrB));
            temp->MatrScale(GpVector[i]->GetWeight());
            temp->MatrScale(Jdet);
            StiffnessMatrix.MatrAdd(StiffnessMatrix, *temp);

            // by the way also computes volume:
            this->Volume += GpVector[i]->GetWeight() * Jdet;
        }
        delete temp;
    }

    //////////////////// *** OLD METHOD (before GaussIntegrationRule) *** //////////////////////
    /// Computes the global STIFFNESS MATRIX of the element:
    /// K = Volume * [B]' * [D] * [B]
    ///
    /*	virtual void ComputeStiffnessMatrix()
                {
                    //========================
                    //Exact Integration (27 Gp)
                    //========================
                        double zeta1, zeta2, zeta3;
                        double Jdet;
                        ChMatrixDynamic<> temp;
                        ChMatrixDynamic<> BT;

                    //////////////////////////////////
                    /// Reduced Integration (8 Gp) ///
                    //////////////////////////////////

                        zeta1=0.577350269189626;
                        zeta2=0.577350269189626;
                        zeta3=0.577350269189626;

                        ComputeMatrB(0, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[0];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[0]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        this->StiffnessMatrix = temp;

                        zeta1=-0.577350269189626;
                        zeta2=0.577350269189626;
                        zeta3=0.577350269189626;

                        ComputeMatrB(1, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[1];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[1]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=0.577350269189626;
                        zeta2=-0.577350269189626;
                        zeta3=0.577350269189626;

                        ComputeMatrB(2, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[2];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[2]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=0.577350269189626;
                        zeta2=0.577350269189626;
                        zeta3=-0.577350269189626;

                        ComputeMatrB(3, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[3];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[3]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=-0.577350269189626;
                        zeta2=-0.577350269189626;
                        zeta3=0.577350269189626;

                        ComputeMatrB(4, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[4];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[4]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=-0.577350269189626;
                        zeta2=0.577350269189626;
                        zeta3=-0.577350269189626;

                        ComputeMatrB(5, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[5];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[5]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=0.577350269189626;
                        zeta2=-0.577350269189626;
                        zeta3=-0.577350269189626;

                        ComputeMatrB(6, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[6];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[6]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                        zeta1=-0.577350269189626;
                        zeta2=-0.577350269189626;
                        zeta3=-0.577350269189626;

                        ComputeMatrB(7, zeta1, zeta2, zeta3, Jdet);
                        BT=MatrB[7];
                        BT.MatrTranspose();
                        temp = (BT * Material->Get_StressStrainMatrix() * MatrB[7]);
                        temp.MatrScale(Jdet);
                        //Gauss integration weight = 1
                        StiffnessMatrix.MatrAdd(StiffnessMatrix,temp);

                }*/

    virtual void SetupInitial(ChSystem* system) override { ComputeStiffnessMatrix(); }

    // compute large rotation of element for corotational approach
    virtual void UpdateRotation() override {
        ChVector<> avgX1;
        avgX1 = this->nodes[0]->GetX0() + this->nodes[1]->GetX0() + this->nodes[2]->GetX0() + this->nodes[3]->GetX0();
        ChVector<> avgX2;
        avgX2 = this->nodes[4]->GetX0() + this->nodes[5]->GetX0() + this->nodes[6]->GetX0() + this->nodes[7]->GetX0();
        ChVector<> Xdir = avgX2 - avgX1;

        ChVector<> avgY1;
        avgY1 = this->nodes[0]->GetX0() + this->nodes[1]->GetX0() + this->nodes[4]->GetX0() + this->nodes[5]->GetX0();
        ChVector<> avgY2;
        avgY2 = this->nodes[2]->GetX0() + this->nodes[3]->GetX0() + this->nodes[6]->GetX0() + this->nodes[7]->GetX0();
        ChVector<> Ydir = avgY2 - avgY1;
        ChMatrix33<> rotX0;
        rotX0.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

        avgX1 = this->nodes[0]->pos + this->nodes[1]->pos + this->nodes[2]->pos + this->nodes[3]->pos;
        avgX2 = this->nodes[4]->pos + this->nodes[5]->pos + this->nodes[6]->pos + this->nodes[7]->pos;
        Xdir = avgX2 - avgX1;

        avgY1 = this->nodes[0]->pos + this->nodes[1]->pos + this->nodes[4]->pos + this->nodes[5]->pos;
        avgY2 = this->nodes[2]->pos + this->nodes[3]->pos + this->nodes[6]->pos + this->nodes[7]->pos;
        Ydir = avgY2 - avgY1;
        ChMatrix33<> rotXcurrent;
        rotXcurrent.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

        this->A.MatrMultiplyT(rotXcurrent, rotX0);
    }

    /// Returns the strain tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStrainTensor<> GetStrain(double z1, double z2, double z3) {
        // set up vector of nodal displacements (in local element system) u_l = R*p - p0
        ChMatrixDynamic<> displ(GetNdofs(), 1);
        this->GetStateBlock(displ);

        double JacobianDet;
        ChMatrixDynamic<> amatrB(6, GetNdofs());
        ComputeMatrB(amatrB, z1, z2, z3, JacobianDet);

        ChStrainTensor<> mstrain;
        mstrain.MatrMultiply(amatrB, displ);
        return mstrain;
    }

    /// Returns the stress tensor at given parameters.
    /// The tensor is in the original undeformed unrotated reference.
    ChStressTensor<> GetStress(double z1, double z2, double z3) {
        ChStressTensor<> mstress;
        mstress.MatrMultiply(this->Material->Get_StressStrainMatrix(), this->GetStrain(z1, z2, z3));
        return mstress;
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == GetNdofs()) && (H.GetColumns() == GetNdofs()));

        // warp the local stiffness matrix K in order to obtain global
        // tangent stiffness CKCt:
        ChMatrixDynamic<> CK(GetNdofs(), GetNdofs());
        ChMatrixDynamic<> CKCt(GetNdofs(), GetNdofs());  // the global, corotated, K matrix, for 20 nodes
        ChMatrixCorotation<>::ComputeCK(StiffnessMatrix, this->A, 20, CK);
        ChMatrixCorotation<>::ComputeKCt(CK, this->A, 20, CKCt);

        // For K stiffness matrix and R damping matrix:

        double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();

        CKCt.MatrScale(mkfactor);

        H.PasteMatrix(CKCt, 0, 0);

        // For M mass matrix:
        if (Mfactor) {
            double lumped_node_mass = (this->Volume * this->Material->Get_density()) / 20.0;
            for (int id = 0; id < GetNdofs(); id++) {
                double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
                H(id, id) += amfactor * lumped_node_mass;
            }
        }
        //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
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

        for (int in = 0; in < 20; ++in) {
            displ.PasteVector(A.MatrT_x_Vect(nodes[in]->pos_dt), in * 3, 0);  // nodal speeds, local
        }
        ChMatrixDynamic<> FiR_local(GetNdofs(), 1);
        FiR_local.MatrMultiply(StiffnessMatrix, displ);
        FiR_local.MatrScale(this->Material->Get_RayleighDampingK());

        double lumped_node_mass = (this->Volume * this->Material->Get_density()) / 20.0;
        displ.MatrScale(lumped_node_mass * this->Material->Get_RayleighDampingM());  // reuse 'displ' for performance
        FiR_local.MatrInc(displ);
        //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

        FiK_local.MatrInc(FiR_local);

        FiK_local.MatrScale(-1.0);

        // Fi = C * Fi_local  with C block-diagonal rotations A
        ChMatrixCorotation<>::ComputeCK(FiK_local, this->A, 20, Fi);
    }

    //
    // Custom properties functions
    //

    /// Set the material of the element
    void SetMaterial(std::shared_ptr<ChContinuumElastic> my_material) { Material = my_material; }
    std::shared_ptr<ChContinuumElastic> GetMaterial() { return Material; }

    /// Get the StiffnessMatrix
    ChMatrix<>& GetStiffnessMatrix() { return StiffnessMatrix; }
    /// Get the Nth gauss point
    ChGaussPoint* GetGaussPoint(int N) { return GpVector[N]; }

    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 20 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 20 * 3; }

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
        mD.PasteVector(this->nodes[10]->GetPos(), block_offset + 30, 0);
        mD.PasteVector(this->nodes[11]->GetPos(), block_offset + 33, 0);
        mD.PasteVector(this->nodes[12]->GetPos(), block_offset + 36, 0);
        mD.PasteVector(this->nodes[13]->GetPos(), block_offset + 39, 0);
        mD.PasteVector(this->nodes[14]->GetPos(), block_offset + 42, 0);
        mD.PasteVector(this->nodes[15]->GetPos(), block_offset + 45, 0);
        mD.PasteVector(this->nodes[16]->GetPos(), block_offset + 48, 0);
        mD.PasteVector(this->nodes[17]->GetPos(), block_offset + 51, 0);
        mD.PasteVector(this->nodes[18]->GetPos(), block_offset + 54, 0);
        mD.PasteVector(this->nodes[19]->GetPos(), block_offset + 57, 0);
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
        mD.PasteVector(this->nodes[10]->GetPos_dt(), block_offset + 30, 0);
        mD.PasteVector(this->nodes[11]->GetPos_dt(), block_offset + 33, 0);
        mD.PasteVector(this->nodes[12]->GetPos_dt(), block_offset + 36, 0);
        mD.PasteVector(this->nodes[13]->GetPos_dt(), block_offset + 39, 0);
        mD.PasteVector(this->nodes[14]->GetPos_dt(), block_offset + 42, 0);
        mD.PasteVector(this->nodes[15]->GetPos_dt(), block_offset + 45, 0);
        mD.PasteVector(this->nodes[16]->GetPos_dt(), block_offset + 48, 0);
        mD.PasteVector(this->nodes[17]->GetPos_dt(), block_offset + 51, 0);
        mD.PasteVector(this->nodes[18]->GetPos_dt(), block_offset + 54, 0);
        mD.PasteVector(this->nodes[19]->GetPos_dt(), block_offset + 57, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<20; ++i) {
            nodes[i]->NodeIntStateIncrement(off_x + i*3  , x_new, x, off_v  + i*3  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 20; }

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
        ChMatrixNM<double, 1, 20> N;
        this->ShapeFunctions(N, U, V, W);  // note: U,V,W in -1..1 range

        detJ = this->GetVolume() / 8.0;

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
        Qi(30) = N(10) * F(0);
        Qi(31) = N(10) * F(1);
        Qi(32) = N(10) * F(2);
        Qi(33) = N(11) * F(0);
        Qi(34) = N(11) * F(1);
        Qi(35) = N(11) * F(2);
        Qi(36) = N(12) * F(0);
        Qi(37) = N(12) * F(1);
        Qi(38) = N(12) * F(2);
        Qi(39) = N(13) * F(0);
        Qi(40) = N(13) * F(1);
        Qi(41) = N(13) * F(2);
        Qi(42) = N(14) * F(0);
        Qi(43) = N(14) * F(1);
        Qi(44) = N(14) * F(2);
        Qi(45) = N(15) * F(0);
        Qi(46) = N(15) * F(1);
        Qi(47) = N(15) * F(2);
        Qi(48) = N(16) * F(0);
        Qi(49) = N(16) * F(1);
        Qi(50) = N(16) * F(2);
        Qi(51) = N(17) * F(0);
        Qi(52) = N(17) * F(1);
        Qi(53) = N(17) * F(2);
        Qi(54) = N(18) * F(0);
        Qi(55) = N(18) * F(1);
        Qi(56) = N(18) * F(2);
        Qi(57) = N(19) * F(0);
        Qi(58) = N(19) * F(1);
        Qi(59) = N(19) * F(2);
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->Material->Get_density(); }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
