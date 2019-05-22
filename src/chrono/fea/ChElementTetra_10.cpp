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

#include "chrono/fea/ChElementTetra_10.h"

namespace chrono {
namespace fea {

ChElementTetra_10::ChElementTetra_10() {
    nodes.resize(10);
    MatrB.resize(4);  // standard: 4 integration points
    MatrB[0].Reset(6, 30);
    MatrB[1].Reset(6, 30);
    MatrB[2].Reset(6, 30);
    MatrB[3].Reset(6, 30);
    this->StiffnessMatrix.Resize(30, 30);
}

ChElementTetra_10::~ChElementTetra_10() {}

void ChElementTetra_10::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
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

void ChElementTetra_10::ShapeFunctions(ChMatrix<>& N, double r, double s, double t) {
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

void ChElementTetra_10::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(this->GetNdofs(), 1);

    for (int i = 0; i < GetNnodes(); i++)
        mD.PasteVector(A.MatrT_x_Vect(this->nodes[i]->GetPos()) - nodes[i]->GetX0(), i * 3, 0);
}

double ChElementTetra_10::ComputeVolume() {
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

void ChElementTetra_10::ComputeJacobian(ChMatrixDynamic<>& Jacobian,
                                        double zeta1,
                                        double zeta2,
                                        double zeta3,
                                        double zeta4) {
    Jacobian.FillElem(1);

    Jacobian.SetElement(1, 0,
                        4 * (nodes[0]->pos.x() * (zeta1 - 1 / 4) + nodes[4]->pos.x() * zeta2 +
                             nodes[6]->pos.x() * zeta3 + nodes[7]->pos.x() * zeta4));
    Jacobian.SetElement(2, 0,
                        4 * (nodes[0]->pos.y() * (zeta1 - 1 / 4) + nodes[4]->pos.y() * zeta2 +
                             nodes[6]->pos.y() * zeta3 + nodes[7]->pos.y() * zeta4));
    Jacobian.SetElement(3, 0,
                        4 * (nodes[0]->pos.z() * (zeta1 - 1 / 4) + nodes[4]->pos.z() * zeta2 +
                             nodes[6]->pos.z() * zeta3 + nodes[7]->pos.z() * zeta4));
    Jacobian.SetElement(1, 1,
                        4 * (nodes[4]->pos.x() * zeta1 + nodes[1]->pos.x() * (zeta2 - 1 / 4) +
                             nodes[5]->pos.x() * zeta3 + nodes[8]->pos.x() * zeta4));
    Jacobian.SetElement(2, 1,
                        4 * (nodes[4]->pos.y() * zeta1 + nodes[1]->pos.y() * (zeta2 - 1 / 4) +
                             nodes[5]->pos.y() * zeta3 + nodes[8]->pos.y() * zeta4));
    Jacobian.SetElement(3, 1,
                        4 * (nodes[4]->pos.z() * zeta1 + nodes[1]->pos.z() * (zeta2 - 1 / 4) +
                             nodes[5]->pos.z() * zeta3 + nodes[8]->pos.z() * zeta4));
    Jacobian.SetElement(1, 2,
                        4 * (nodes[6]->pos.x() * zeta1 + nodes[5]->pos.x() * zeta2 +
                             nodes[2]->pos.x() * (zeta3 - 1 / 4) + nodes[9]->pos.x() * zeta4));
    Jacobian.SetElement(2, 2,
                        4 * (nodes[6]->pos.y() * zeta1 + nodes[5]->pos.y() * zeta2 +
                             nodes[2]->pos.y() * (zeta3 - 1 / 4) + nodes[9]->pos.y() * zeta4));
    Jacobian.SetElement(3, 2,
                        4 * (nodes[6]->pos.z() * zeta1 + nodes[5]->pos.z() * zeta2 +
                             nodes[2]->pos.z() * (zeta3 - 1 / 4) + nodes[9]->pos.z() * zeta4));
    Jacobian.SetElement(1, 3,
                        4 * (nodes[7]->pos.x() * zeta1 + nodes[8]->pos.x() * zeta2 + nodes[9]->pos.x() * zeta3 +
                             nodes[3]->pos.x() * (zeta4 - 1 / 4)));
    Jacobian.SetElement(2, 3,
                        4 * (nodes[7]->pos.y() * zeta1 + nodes[8]->pos.y() * zeta2 + nodes[9]->pos.y() * zeta3 +
                             nodes[3]->pos.y() * (zeta4 - 1 / 4)));
    Jacobian.SetElement(3, 3,
                        4 * (nodes[7]->pos.z() * zeta1 + nodes[8]->pos.z() * zeta2 + nodes[9]->pos.z() * zeta3 +
                             nodes[3]->pos.z() * (zeta4 - 1 / 4)));
}

void ChElementTetra_10::ComputeMatrB(ChMatrixDynamic<>& mmatrB,
                                     double zeta1,
                                     double zeta2,
                                     double zeta3,
                                     double zeta4,
                                     double& JacobianDet) {
    ChMatrixDynamic<> Jacobian(4, 4);
    ComputeJacobian(Jacobian, zeta1, zeta2, zeta3, zeta4);

    double Jdet = Jacobian.Det();
    JacobianDet = Jdet;  // !!! store the Jacobian Determinant: needed for the integration

    mmatrB.SetElement(0, 0,
                      (4 * zeta1 - 1) *
                          ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                           (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) /
                          Jdet);
    mmatrB.SetElement(0, 3,
                      (4 * zeta2 - 1) *
                          ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                           (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) /
                          Jdet);
    mmatrB.SetElement(0, 6,
                      (4 * zeta3 - 1) *
                          ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                           (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) /
                          Jdet);
    mmatrB.SetElement(0, 9,
                      (4 * zeta4 - 1) *
                          ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                           (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) /
                          Jdet);
    mmatrB.SetElement(0, 12,
                      4 *
                          (zeta1 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                    (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2))) +
                           zeta2 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                    (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                          Jdet);
    mmatrB.SetElement(0, 15,
                      4 *
                          (zeta2 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                    (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3))) +
                           zeta3 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                    (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                          Jdet);
    mmatrB.SetElement(0, 18,
                      4 *
                          (zeta3 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                    (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1))) +
                           zeta1 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                    (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                          Jdet);
    mmatrB.SetElement(0, 21,
                      4 *
                          (zeta1 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                    (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                           zeta4 * ((Jacobian(2, 3) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)) -
                                    (Jacobian(2, 2) - Jacobian(2, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)))) /
                          Jdet);
    mmatrB.SetElement(0, 24,
                      4 *
                          (zeta2 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                    (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                           zeta4 * ((Jacobian(2, 2) - Jacobian(2, 0)) * (Jacobian(3, 3) - Jacobian(3, 2)) -
                                    (Jacobian(2, 2) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 2)))) /
                          Jdet);
    mmatrB.SetElement(0, 27,
                      4 *
                          (zeta3 * ((Jacobian(2, 0) - Jacobian(2, 2)) * (Jacobian(3, 1) - Jacobian(3, 0)) -
                                    (Jacobian(2, 0) - Jacobian(2, 1)) * (Jacobian(3, 2) - Jacobian(3, 0))) +
                           zeta4 * ((Jacobian(2, 1) - Jacobian(2, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)) -
                                    (Jacobian(2, 0) - Jacobian(2, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)))) /
                          Jdet);

    mmatrB.SetElement(1, 1,
                      (4 * zeta1 - 1) *
                          ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                           (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) /
                          Jdet);
    mmatrB.SetElement(1, 4,
                      (4 * zeta2 - 1) *
                          ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                           (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) /
                          Jdet);
    mmatrB.SetElement(1, 7,
                      (4 * zeta3 - 1) *
                          ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                           (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) /
                          Jdet);
    mmatrB.SetElement(1, 10,
                      (4 * zeta4 - 1) *
                          ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                           (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) /
                          Jdet);
    mmatrB.SetElement(1, 13,
                      4 *
                          (zeta1 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                    (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3))) +
                           zeta2 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                    (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                          Jdet);
    mmatrB.SetElement(1, 16,
                      4 *
                          (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                    (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3))) +
                           zeta3 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                    (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                          Jdet);
    mmatrB.SetElement(1, 19,
                      4 *
                          (zeta3 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                    (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1))) +
                           zeta1 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                    (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                          Jdet);
    mmatrB.SetElement(1, 22,
                      4 *
                          (zeta1 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                           zeta4 * ((Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(3, 3) - Jacobian(3, 1)) -
                                    (Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(3, 2) - Jacobian(3, 1)))) /
                          Jdet);
    mmatrB.SetElement(1, 25,
                      4 *
                          (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                           zeta4 * ((Jacobian(1, 3) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 0)) -
                                    (Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(3, 2) - Jacobian(3, 3)))) /
                          Jdet);
    mmatrB.SetElement(1, 28,
                      4 *
                          (zeta3 * ((Jacobian(1, 1) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(3, 0) - Jacobian(3, 1))) +
                           zeta4 * ((Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(3, 1) - Jacobian(3, 3)) -
                                    (Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(3, 0) - Jacobian(3, 3)))) /
                          Jdet);

    mmatrB.SetElement(2, 2,
                      (4 * zeta1 - 1) *
                          ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                           (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) /
                          Jdet);
    mmatrB.SetElement(2, 5,
                      (4 * zeta2 - 1) *
                          ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                           (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) /
                          Jdet);
    mmatrB.SetElement(2, 8,
                      (4 * zeta3 - 1) *
                          ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                           (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) /
                          Jdet);
    mmatrB.SetElement(2, 11,
                      (4 * zeta4 - 1) *
                          ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                           (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) /
                          Jdet);
    mmatrB.SetElement(2, 14,
                      4 *
                          (zeta1 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2))) +
                           zeta2 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                    (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                          Jdet);
    mmatrB.SetElement(2, 17,
                      4 *
                          (zeta2 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                    (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3))) +
                           zeta3 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                          Jdet);
    mmatrB.SetElement(2, 20,
                      4 *
                          (zeta3 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                    (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1))) +
                           zeta1 * ((Jacobian(1, 1) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 3)) -
                                    (Jacobian(1, 0) - Jacobian(1, 3)) * (Jacobian(2, 1) - Jacobian(2, 3)))) /
                          Jdet);
    mmatrB.SetElement(2, 23,
                      4 *
                          (zeta1 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                    (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                           zeta4 * ((Jacobian(1, 3) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 1)) -
                                    (Jacobian(1, 2) - Jacobian(1, 1)) * (Jacobian(2, 3) - Jacobian(2, 1)))) /
                          Jdet);
    mmatrB.SetElement(2, 26,
                      4 *
                          (zeta2 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
                                    (Jacobian(1, 0) - Jacobian(1, 1)) * (Jacobian(2, 2) - Jacobian(2, 0))) +
                           zeta4 * ((Jacobian(1, 2) - Jacobian(1, 0)) * (Jacobian(2, 3) - Jacobian(2, 2)) -
                                    (Jacobian(1, 2) - Jacobian(1, 3)) * (Jacobian(2, 0) - Jacobian(2, 2)))) /
                          Jdet);
    mmatrB.SetElement(2, 29,
                      4 *
                          (zeta3 * ((Jacobian(1, 0) - Jacobian(1, 2)) * (Jacobian(2, 1) - Jacobian(2, 0)) -
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

void ChElementTetra_10::ComputeStiffnessMatrix() {
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

    StiffnessMatrix.MatrDivScale(2);  //!!! => because the canonical interval is -1 ... +1,  but we want to integrate
                                      //		 in 0 ... +1 -> we have to multiply by: b-a/2 ( = (1-0)/2 = 1/2)
}

void ChElementTetra_10::GetParameterForNodeID(const int nodeID, double& z1, double& z2, double& z3, double& z4) {
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

ChStrainTensor<> ChElementTetra_10::GetStrain(double z1, double z2, double z3, double z4) {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChMatrixDynamic<> displ(GetNdofs(), 1);
    this->GetStateBlock(displ);

    double JacobianDet;
    ChMatrixDynamic<> amatrB(6, GetNdofs());
    amatrB.Reset();
    ComputeMatrB(amatrB, z1, z2, z3, z4, JacobianDet);

    ChStrainTensor<> mstrain;
    mstrain.MatrMultiply(amatrB, displ);
    return mstrain;
}

ChStressTensor<> ChElementTetra_10::GetStress(double z1, double z2, double z3, double z4) {
    ChStressTensor<> mstress;
    mstress.MatrMultiply(this->Material->Get_StressStrainMatrix(), this->GetStrain(z1, z2, z3, z4));
    return mstress;
}

void ChElementTetra_10::SetupInitial(ChSystem* system) {
    ComputeVolume();
    ComputeStiffnessMatrix();
}

void ChElementTetra_10::UpdateRotation() {
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

void ChElementTetra_10::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
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

void ChElementTetra_10::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
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

void ChElementTetra_10::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
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

void ChElementTetra_10::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
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

void ChElementTetra_10::LoadableStateIncrement(const unsigned int off_x,
                                               ChState& x_new,
                                               const ChState& x,
                                               const unsigned int off_v,
                                               const ChStateDelta& Dv) {
    for (int i = 0; i < 10; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementTetra_10::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementTetra_10::ComputeNF(const double U,
                                  const double V,
                                  const double W,
                                  ChVectorDynamic<>& Qi,
                                  double& detJ,
                                  const ChVectorDynamic<>& F,
                                  ChVectorDynamic<>* state_x,
                                  ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependant on state
    ChMatrixNM<double, 1, 10> N;
    this->ShapeFunctions(N, U, V,
                         W);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

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
