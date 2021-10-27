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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChElementTetraCorot_10.h"

namespace chrono {
namespace fea {

ChElementTetraCorot_10::ChElementTetraCorot_10() : Volume(0) {
    nodes.resize(10);
    MatrB.resize(4);  // standard: 4 integration points
    MatrB[0].setZero(6, 30);
    MatrB[1].setZero(6, 30);
    MatrB[2].setZero(6, 30);
    MatrB[3].setZero(6, 30);
    this->StiffnessMatrix.setZero(30, 30);
}

ChElementTetraCorot_10::~ChElementTetraCorot_10() {}

void ChElementTetraCorot_10::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
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

void ChElementTetraCorot_10::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementTetraCorot_10::ShapeFunctions(ShapeVector& N, double r, double s, double t) {
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
}

void ChElementTetraCorot_10::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNdofs());

    for (int i = 0; i < GetNnodes(); i++)
        mD.segment(i * 3, 3) = (A.transpose() * this->nodes[i]->GetPos() - nodes[i]->GetX0()).eigen();
}

double ChElementTetraCorot_10::ComputeVolume() {
    ChVector<> B1, C1, D1;
    B1.Sub(nodes[1]->pos, nodes[0]->pos);
    C1.Sub(nodes[2]->pos, nodes[0]->pos);
    D1.Sub(nodes[3]->pos, nodes[0]->pos);
    ChMatrixDynamic<> M(3, 3);
    M.col(0) = B1.eigen();
    M.col(1) = C1.eigen();
    M.col(2) = D1.eigen();
    M.transposeInPlace();
    Volume = std::abs(M.determinant() / 6);
    return Volume;
}

void ChElementTetraCorot_10::ComputeJacobian(ChMatrixDynamic<>& Jacobian,
                                             double zeta1,
                                             double zeta2,
                                             double zeta3,
                                             double zeta4) {
    Jacobian.setConstant(1);

    Jacobian(1, 0) = 4 * (nodes[0]->pos.x() * (zeta1 - 1 / 4) + nodes[4]->pos.x() * zeta2 + nodes[6]->pos.x() * zeta3 +
                          nodes[7]->pos.x() * zeta4);
    Jacobian(2, 0) = 4 * (nodes[0]->pos.y() * (zeta1 - 1 / 4) + nodes[4]->pos.y() * zeta2 + nodes[6]->pos.y() * zeta3 +
                          nodes[7]->pos.y() * zeta4);
    Jacobian(3, 0) = 4 * (nodes[0]->pos.z() * (zeta1 - 1 / 4) + nodes[4]->pos.z() * zeta2 + nodes[6]->pos.z() * zeta3 +
                          nodes[7]->pos.z() * zeta4);
    Jacobian(1, 1) = 4 * (nodes[4]->pos.x() * zeta1 + nodes[1]->pos.x() * (zeta2 - 1 / 4) + nodes[5]->pos.x() * zeta3 +
                          nodes[8]->pos.x() * zeta4);
    Jacobian(2, 1) = 4 * (nodes[4]->pos.y() * zeta1 + nodes[1]->pos.y() * (zeta2 - 1 / 4) + nodes[5]->pos.y() * zeta3 +
                          nodes[8]->pos.y() * zeta4);
    Jacobian(3, 1) = 4 * (nodes[4]->pos.z() * zeta1 + nodes[1]->pos.z() * (zeta2 - 1 / 4) + nodes[5]->pos.z() * zeta3 +
                          nodes[8]->pos.z() * zeta4);
    Jacobian(1, 2) = 4 * (nodes[6]->pos.x() * zeta1 + nodes[5]->pos.x() * zeta2 + nodes[2]->pos.x() * (zeta3 - 1 / 4) +
                          nodes[9]->pos.x() * zeta4);
    Jacobian(2, 2) = 4 * (nodes[6]->pos.y() * zeta1 + nodes[5]->pos.y() * zeta2 + nodes[2]->pos.y() * (zeta3 - 1 / 4) +
                          nodes[9]->pos.y() * zeta4);
    Jacobian(3, 2) = 4 * (nodes[6]->pos.z() * zeta1 + nodes[5]->pos.z() * zeta2 + nodes[2]->pos.z() * (zeta3 - 1 / 4) +
                          nodes[9]->pos.z() * zeta4);
    Jacobian(1, 3) = 4 * (nodes[7]->pos.x() * zeta1 + nodes[8]->pos.x() * zeta2 + nodes[9]->pos.x() * zeta3 +
                          nodes[3]->pos.x() * (zeta4 - 1 / 4));
    Jacobian(2, 3) = 4 * (nodes[7]->pos.y() * zeta1 + nodes[8]->pos.y() * zeta2 + nodes[9]->pos.y() * zeta3 +
                          nodes[3]->pos.y() * (zeta4 - 1 / 4));
    Jacobian(3, 3) = 4 * (nodes[7]->pos.z() * zeta1 + nodes[8]->pos.z() * zeta2 + nodes[9]->pos.z() * zeta3 +
                          nodes[3]->pos.z() * (zeta4 - 1 / 4));
}

void ChElementTetraCorot_10::ComputeMatrB(ChMatrixDynamic<>& mmatrB,
                                          double zeta1,
                                          double zeta2,
                                          double zeta3,
                                          double zeta4,
                                          double& JacobianDet) {
    ChMatrixDynamic<> Jacobian(4, 4);
    ComputeJacobian(Jacobian, zeta1, zeta2, zeta3, zeta4);

    double Jdet = Jacobian.determinant();
    JacobianDet = Jdet;  // !!! store the Jacobian Determinant: needed for the integration

    mmatrB(0, 0) = (4 * zeta1 - 1) *
                   ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                    (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) /
                   Jdet;
    mmatrB(0, 3) = (4 * zeta2 - 1) *
                   ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                    (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) /
                   Jdet;
    mmatrB(0, 6) = (4 * zeta3 - 1) *
                   ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                    (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) /
                   Jdet;
    mmatrB(0, 9) = (4 * zeta4 - 1) *
                   ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                    (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) /
                   Jdet;
    mmatrB(0, 12) = 4 *
                    (zeta1 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                              (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) +
                     zeta2 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                              (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                    Jdet;
    mmatrB(0, 15) = 4 *
                    (zeta2 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                              (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) +
                     zeta3 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                              (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                    Jdet;
    mmatrB(0, 18) = 4 *
                    (zeta3 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                              (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) +
                     zeta1 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                              (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                    Jdet;
    mmatrB(0, 21) = 4 *
                    (zeta1 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                              (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                     zeta4 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                              (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                    Jdet;
    mmatrB(0, 24) = 4 *
                    (zeta2 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                              (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                     zeta4 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                              (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                    Jdet;
    mmatrB(0, 27) = 4 *
                    (zeta3 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                              (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                     zeta4 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                              (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                    Jdet;

    mmatrB(1, 1) = (4 * zeta1 - 1) *
                   ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                    (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) /
                   Jdet;
    mmatrB(1, 4) = (4 * zeta2 - 1) *
                   ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                    (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) /
                   Jdet;
    mmatrB(1, 7) = (4 * zeta3 - 1) *
                   ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                    (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) /
                   Jdet;
    mmatrB(1, 10) = (4 * zeta4 - 1) *
                    ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                     (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) /
                    Jdet;
    mmatrB(1, 13) = 4 *
                    (zeta1 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) +
                     zeta2 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                              (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                    Jdet;
    mmatrB(1, 16) = 4 *
                    (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                              (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) +
                     zeta3 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                    Jdet;
    mmatrB(1, 19) = 4 *
                    (zeta3 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                              (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) +
                     zeta1 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                              (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                    Jdet;
    mmatrB(1, 22) = 4 *
                    (zeta1 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                     zeta4 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                              (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                    Jdet;
    mmatrB(1, 25) = 4 *
                    (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                     zeta4 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                    Jdet;
    mmatrB(1, 28) = 4 *
                    (zeta3 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                     zeta4 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                              (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                    Jdet;

    mmatrB(2, 2) = (4 * zeta1 - 1) *
                   ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                    (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) /
                   Jdet;
    mmatrB(2, 5) = (4 * zeta2 - 1) *
                   ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                    (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) /
                   Jdet;
    mmatrB(2, 8) = (4 * zeta3 - 1) *
                   ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                    (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) /
                   Jdet;
    mmatrB(2, 11) = (4 * zeta4 - 1) *
                    ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                     (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) /
                    Jdet;
    mmatrB(2, 14) = 4 *
                    (zeta1 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) +
                     zeta2 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                              (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                    Jdet;
    mmatrB(2, 17) = 4 *
                    (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                              (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) +
                     zeta3 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                    Jdet;
    mmatrB(2, 20) = 4 *
                    (zeta3 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                              (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) +
                     zeta1 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                              (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3)))) /
                    Jdet;
    mmatrB(2, 23) = 4 *
                    (zeta1 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                     zeta4 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                              (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                    Jdet;
    mmatrB(2, 26) = 4 *
                    (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                     zeta4 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                              (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                    Jdet;
    mmatrB(2, 29) = 4 *
                    (zeta3 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                              (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                     zeta4 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                              (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3)))) /
                    Jdet;

    mmatrB(3, 0) = mmatrB(1, 1);
    mmatrB(3, 1) = mmatrB(0, 0);
    mmatrB(3, 3) = mmatrB(1, 4);
    mmatrB(3, 4) = mmatrB(0, 3);
    mmatrB(3, 6) = mmatrB(1, 7);
    mmatrB(3, 7) = mmatrB(0, 6);
    mmatrB(3, 9) = mmatrB(1, 10);
    mmatrB(3, 10) = mmatrB(0, 9);
    mmatrB(3, 12) = mmatrB(1, 13);
    mmatrB(3, 13) = mmatrB(0, 12);
    mmatrB(3, 15) = mmatrB(1, 16);
    mmatrB(3, 16) = mmatrB(0, 15);
    mmatrB(3, 18) = mmatrB(1, 19);
    mmatrB(3, 19) = mmatrB(0, 18);
    mmatrB(3, 21) = mmatrB(1, 22);
    mmatrB(3, 22) = mmatrB(0, 21);
    mmatrB(3, 24) = mmatrB(1, 25);
    mmatrB(3, 25) = mmatrB(0, 24);
    mmatrB(3, 27) = mmatrB(1, 28);
    mmatrB(3, 28) = mmatrB(0, 27);

    mmatrB(4, 1) = mmatrB(2, 2);
    mmatrB(4, 2) = mmatrB(1, 1);
    mmatrB(4, 4) = mmatrB(2, 5);
    mmatrB(4, 5) = mmatrB(1, 4);
    mmatrB(4, 7) = mmatrB(2, 8);
    mmatrB(4, 8) = mmatrB(1, 7);
    mmatrB(4, 10) = mmatrB(2, 11);
    mmatrB(4, 11) = mmatrB(1, 10);
    mmatrB(4, 13) = mmatrB(2, 14);
    mmatrB(4, 14) = mmatrB(1, 13);
    mmatrB(4, 16) = mmatrB(2, 17);
    mmatrB(4, 17) = mmatrB(1, 16);
    mmatrB(4, 19) = mmatrB(2, 20);
    mmatrB(4, 20) = mmatrB(1, 19);
    mmatrB(4, 22) = mmatrB(2, 23);
    mmatrB(4, 23) = mmatrB(1, 22);
    mmatrB(4, 25) = mmatrB(2, 26);
    mmatrB(4, 26) = mmatrB(1, 25);
    mmatrB(4, 28) = mmatrB(2, 29);
    mmatrB(4, 29) = mmatrB(1, 28);

    mmatrB(5, 0) = mmatrB(2, 2);
    mmatrB(5, 2) = mmatrB(0, 0);
    mmatrB(5, 3) = mmatrB(2, 5);
    mmatrB(5, 5) = mmatrB(0, 3);
    mmatrB(5, 6) = mmatrB(2, 8);
    mmatrB(5, 8) = mmatrB(0, 6);
    mmatrB(5, 9) = mmatrB(2, 11);
    mmatrB(5, 11) = mmatrB(0, 9);
    mmatrB(5, 12) = mmatrB(2, 14);
    mmatrB(5, 14) = mmatrB(0, 12);
    mmatrB(5, 15) = mmatrB(2, 17);
    mmatrB(5, 17) = mmatrB(0, 15);
    mmatrB(5, 18) = mmatrB(2, 20);
    mmatrB(5, 20) = mmatrB(0, 18);
    mmatrB(5, 21) = mmatrB(2, 23);
    mmatrB(5, 23) = mmatrB(0, 21);
    mmatrB(5, 24) = mmatrB(2, 26);
    mmatrB(5, 26) = mmatrB(0, 24);
    mmatrB(5, 27) = mmatrB(2, 29);
    mmatrB(5, 29) = mmatrB(0, 27);

    mmatrB *= 2;
}

void ChElementTetraCorot_10::ComputeStiffnessMatrix() {
    // for speeding up corotational, used later:
    // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
    //     [ 1    1    1    1    ]
    ChMatrixNM<double, 4, 4> tmp;
    tmp.block(0, 0, 3, 1) = nodes[0]->GetX0().eigen();
    tmp.block(0, 1, 3, 1) = nodes[1]->GetX0().eigen();
    tmp.block(0, 2, 3, 1) = nodes[2]->GetX0().eigen();
    tmp.block(0, 3, 3, 1) = nodes[3]->GetX0().eigen();
    tmp.row(3).setConstant(1.0);
    mM = tmp.inverse();

    //========================
    // Exact Integration (4 Gp)
    //========================
    ChMatrixDynamic<> temp;

    double zeta1 = 0.58541020;
    double zeta2 = 0.1381966;
    double zeta3 = 0.1381966;
    double zeta4 = 0.1381966;
    double JacobianDet;
    ComputeMatrB(this->MatrB[0], zeta1, zeta2, zeta3, zeta4, JacobianDet);
    temp = (JacobianDet / 6.0 / 16.0) * (MatrB[0].transpose() * Material->Get_StressStrainMatrix() * MatrB[0]);
    // Gauss integration weight = 1*1/4*1/4*1/4
    StiffnessMatrix = temp;

    zeta1 = 0.1381966;
    zeta2 = 0.58541020;
    zeta3 = 0.1381966;
    zeta4 = 0.1381966;
    ComputeMatrB(this->MatrB[1], zeta1, zeta2, zeta3, zeta4, JacobianDet);
    temp = (JacobianDet / 6.0 / 16.0) * (MatrB[1].transpose() * Material->Get_StressStrainMatrix() * MatrB[1]);
    // Gauss integration weight = 1*1/4*1/4*1/4
    StiffnessMatrix += temp;

    zeta1 = 0.1381966;
    zeta2 = 0.1381966;
    zeta3 = 0.58541020;
    zeta4 = 0.1381966;
    ComputeMatrB(this->MatrB[2], zeta1, zeta2, zeta3, zeta4, JacobianDet);
    temp = (JacobianDet / 6.0 / 16.0) * (MatrB[2].transpose() * Material->Get_StressStrainMatrix() * MatrB[2]);
    // Gauss integration weight = 1*1/4*1/4*1/4
    StiffnessMatrix += temp;

    zeta1 = 0.1381966;
    zeta2 = 0.1381966;
    zeta3 = 0.1381966;
    zeta4 = 0.58541020;
    ComputeMatrB(this->MatrB[3], zeta1, zeta2, zeta3, zeta4, JacobianDet);
    temp = (JacobianDet / 6.0 / 16.0) * (MatrB[3].transpose() * Material->Get_StressStrainMatrix() * MatrB[3]);
    // Gauss integration weight = 1*1/4*1/4*1/4
    StiffnessMatrix += temp;

    StiffnessMatrix /= 2;  //!!! => because the canonical interval is -1 ... +1,  but we want to integrate
                           //		 in 0 ... +1 -> we have to multiply by: b-a/2 ( = (1-0)/2 = 1/2)
}

void ChElementTetraCorot_10::GetParameterForNodeID(const int nodeID, double& z1, double& z2, double& z3, double& z4) {
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

ChStrainTensor<> ChElementTetraCorot_10::GetStrain(double z1, double z2, double z3, double z4) {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(GetNdofs());
    this->GetStateBlock(displ);

    double JacobianDet;
    ChMatrixDynamic<> amatrB(6, GetNdofs());
    amatrB.setZero();
    ComputeMatrB(amatrB, z1, z2, z3, z4, JacobianDet);

    ChStrainTensor<> mstrain = amatrB * displ;
    return mstrain;
}

ChStressTensor<> ChElementTetraCorot_10::GetStress(double z1, double z2, double z3, double z4) {
    ChStressTensor<> mstress = this->Material->Get_StressStrainMatrix() * this->GetStrain(z1, z2, z3, z4);
    return mstress;
}

void ChElementTetraCorot_10::SetupInitial(ChSystem* system) {
    ComputeVolume();
    ComputeStiffnessMatrix();
}

void ChElementTetraCorot_10::UpdateRotation() {
    // P = [ p_0  p_1  p_2  p_3 ]
    //     [ 1    1    1    1   ]
    ChMatrixNM<double, 4, 4> P;
    P.block(0, 0, 3, 1) = nodes[0]->pos.eigen();
    P.block(0, 1, 3, 1) = nodes[1]->pos.eigen();
    P.block(0, 2, 3, 1) = nodes[2]->pos.eigen();
    P.block(0, 3, 3, 1) = nodes[3]->pos.eigen();
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
        this->A *= -1.0;

    // GetLog() << "FEM rotation: \n" << A << "\n"
}

void ChElementTetraCorot_10::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == GetNdofs()) && (H.cols() == GetNdofs()));

    // warp the local stiffness matrix K in order to obtain global tangent stiffness CKCt:
    ChMatrixDynamic<> CK(GetNdofs(), GetNdofs());
    ChMatrixDynamic<> CKCt(GetNdofs(), GetNdofs());  // the global, corotated, K matrix
    ChMatrixCorotation::ComputeCK(StiffnessMatrix, this->A, 10, CK);
    ChMatrixCorotation::ComputeKCt(CK, this->A, 10, CKCt);

    // For K stiffness matrix and R damping matrix:
    double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();
    H.block(0, 0, GetNdofs(), GetNdofs()) = mkfactor * CKCt;

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

void ChElementTetraCorot_10::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == GetNdofs());

    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(GetNdofs());
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    // nodal speeds, local
    displ.segment(0, 3) = A.transpose() * nodes[0]->pos_dt.eigen();
    displ.segment(3, 3) = A.transpose() * nodes[1]->pos_dt.eigen();
    displ.segment(6, 3) = A.transpose() * nodes[2]->pos_dt.eigen();
    displ.segment(9, 3) = A.transpose() * nodes[3]->pos_dt.eigen();
    displ.segment(12, 3) = A.transpose() * nodes[4]->pos_dt.eigen();
    displ.segment(15, 3) = A.transpose() * nodes[5]->pos_dt.eigen();
    displ.segment(18, 3) = A.transpose() * nodes[6]->pos_dt.eigen();
    displ.segment(21, 3) = A.transpose() * nodes[7]->pos_dt.eigen();
    displ.segment(24, 3) = A.transpose() * nodes[8]->pos_dt.eigen();
    displ.segment(27, 3) = A.transpose() * nodes[9]->pos_dt.eigen();

    double lumped_node_mass = (GetVolume() * Material->Get_density()) / GetNnodes();
    ChVectorDynamic<> FiR_local =
        Material->Get_RayleighDampingK() * (StiffnessMatrix * displ + lumped_node_mass * displ);
    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

    FiK_local += FiR_local;
    FiK_local *= -1.0;

    // Fi = C * Fi_local  with C block-diagonal rotations A
    ChMatrixCorotation::ComputeCK(FiK_local, this->A, 10, Fi);
}

void ChElementTetraCorot_10::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = this->nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = this->nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 12, 3) = this->nodes[4]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = this->nodes[5]->GetPos().eigen();
    mD.segment(block_offset + 18, 3) = this->nodes[6]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = this->nodes[7]->GetPos().eigen();
    mD.segment(block_offset + 24, 3) = this->nodes[8]->GetPos().eigen();
    mD.segment(block_offset + 27, 3) = this->nodes[9]->GetPos().eigen();
}

void ChElementTetraCorot_10::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 6, 3) = this->nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = this->nodes[3]->GetPos_dt().eigen();
    mD.segment(block_offset + 12, 3) = this->nodes[4]->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = this->nodes[5]->GetPos_dt().eigen();
    mD.segment(block_offset + 18, 3) = this->nodes[6]->GetPos_dt().eigen();
    mD.segment(block_offset + 21, 3) = this->nodes[7]->GetPos_dt().eigen();
    mD.segment(block_offset + 24, 3) = this->nodes[8]->GetPos_dt().eigen();
    mD.segment(block_offset + 27, 3) = this->nodes[9]->GetPos_dt().eigen();
}

void ChElementTetraCorot_10::LoadableStateIncrement(const unsigned int off_x,
                                                    ChState& x_new,
                                                    const ChState& x,
                                                    const unsigned int off_v,
                                                    const ChStateDelta& Dv) {
    for (int i = 0; i < 10; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementTetraCorot_10::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementTetraCorot_10::ComputeNF(const double U,
                                       const double V,
                                       const double W,
                                       ChVectorDynamic<>& Qi,
                                       double& detJ,
                                       const ChVectorDynamic<>& F,
                                       ChVectorDynamic<>* state_x,
                                       ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependant on state
    // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}
    ShapeVector N;
    ShapeFunctions(N, U, V, W);

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

}  // end namespace fea
}  // end namespace chrono
