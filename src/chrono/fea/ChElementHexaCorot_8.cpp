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
// Authors: Andrea Favali, Radu Serban
// =============================================================================

#include "chrono/fea/ChElementHexaCorot_8.h"

namespace chrono {
namespace fea {

ChElementHexaCorot_8::ChElementHexaCorot_8() : ir(nullptr), Volume(0) {
    nodes.resize(8);
    StiffnessMatrix.setZero(24, 24);
    this->ir = new ChGaussIntegrationRule;
    this->SetDefaultIntegrationRule();
}

ChElementHexaCorot_8::~ChElementHexaCorot_8() {
    delete ir;
    for (auto gpoint : GpVector)
        delete gpoint;
    GpVector.clear();
}

void ChElementHexaCorot_8::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                    std::shared_ptr<ChNodeFEAxyz> nodeB,
                                    std::shared_ptr<ChNodeFEAxyz> nodeC,
                                    std::shared_ptr<ChNodeFEAxyz> nodeD,
                                    std::shared_ptr<ChNodeFEAxyz> nodeE,
                                    std::shared_ptr<ChNodeFEAxyz> nodeF,
                                    std::shared_ptr<ChNodeFEAxyz> nodeG,
                                    std::shared_ptr<ChNodeFEAxyz> nodeH) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    nodes[4] = nodeE;
    nodes[5] = nodeF;
    nodes[6] = nodeG;
    nodes[7] = nodeH;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    mvars.push_back(&nodes[4]->Variables());
    mvars.push_back(&nodes[5]->Variables());
    mvars.push_back(&nodes[6]->Variables());
    mvars.push_back(&nodes[7]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementHexaCorot_8::ShapeFunctions(ShapeVector& N, double z0, double z1, double z2) {
    double sc = 1. / 8.;
    N(0) = sc * (1 - z0) * (1 - z1) * (1 - z2);
    N(1) = sc * (1 + z0) * (1 - z1) * (1 - z2);
    N(2) = sc * (1 + z0) * (1 + z1) * (1 - z2);
    N(3) = sc * (1 - z0) * (1 + z1) * (1 - z2);
    N(4) = sc * (1 - z0) * (1 - z1) * (1 + z2);
    N(5) = sc * (1 + z0) * (1 - z1) * (1 + z2);
    N(6) = sc * (1 + z0) * (1 + z1) * (1 + z2);
    N(7) = sc * (1 - z0) * (1 + z1) * (1 + z2);
}

void ChElementHexaCorot_8::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNdofs());

    for (int i = 0; i < GetNnodes(); i++)
        mD.segment(i * 3, 3) = (A.transpose() * this->nodes[i]->GetPos() - nodes[i]->GetX0()).eigen();
}

void ChElementHexaCorot_8::ComputeJacobian(ChMatrixDynamic<>& Jacobian, ChMatrixDynamic<>& J1, ChVector<> coord) {
    ChMatrixDynamic<> J2(8, 3);

    J1(0, 0) = -(1 - coord.y()) * (1 - coord.z()) / 8;
    J1(0, 1) = +(1 - coord.y()) * (1 - coord.z()) / 8;
    J1(0, 2) = +(1 + coord.y()) * (1 - coord.z()) / 8;
    J1(0, 3) = -(1 + coord.y()) * (1 - coord.z()) / 8;
    J1(0, 4) = -(1 - coord.y()) * (1 + coord.z()) / 8;
    J1(0, 5) = +(1 - coord.y()) * (1 + coord.z()) / 8;
    J1(0, 6) = +(1 + coord.y()) * (1 + coord.z()) / 8;
    J1(0, 7) = -(1 + coord.y()) * (1 + coord.z()) / 8;

    J1(1, 0) = -(1 - coord.x()) * (1 - coord.z()) / 8;
    J1(1, 1) = -(1 + coord.x()) * (1 - coord.z()) / 8;
    J1(1, 2) = +(1 + coord.x()) * (1 - coord.z()) / 8;
    J1(1, 3) = +(1 - coord.x()) * (1 - coord.z()) / 8;
    J1(1, 4) = -(1 - coord.x()) * (1 + coord.z()) / 8;
    J1(1, 5) = -(1 + coord.x()) * (1 + coord.z()) / 8;
    J1(1, 6) = +(1 + coord.x()) * (1 + coord.z()) / 8;
    J1(1, 7) = +(1 - coord.x()) * (1 + coord.z()) / 8;

    J1(2, 0) = -(1 - coord.x()) * (1 - coord.y()) / 8;
    J1(2, 1) = -(1 + coord.x()) * (1 - coord.y()) / 8;
    J1(2, 2) = -(1 + coord.x()) * (1 + coord.y()) / 8;
    J1(2, 3) = -(1 - coord.x()) * (1 + coord.y()) / 8;
    J1(2, 4) = +(1 - coord.x()) * (1 - coord.y()) / 8;
    J1(2, 5) = +(1 + coord.x()) * (1 - coord.y()) / 8;
    J1(2, 6) = +(1 + coord.x()) * (1 + coord.y()) / 8;
    J1(2, 7) = +(1 - coord.x()) * (1 + coord.y()) / 8;

    J2(0, 0) = nodes[0]->GetX0().x();
    J2(1, 0) = nodes[1]->GetX0().x();
    J2(2, 0) = nodes[2]->GetX0().x();
    J2(3, 0) = nodes[3]->GetX0().x();
    J2(4, 0) = nodes[4]->GetX0().x();
    J2(5, 0) = nodes[5]->GetX0().x();
    J2(6, 0) = nodes[6]->GetX0().x();
    J2(7, 0) = nodes[7]->GetX0().x();

    J2(0, 1) = nodes[0]->GetX0().y();
    J2(1, 1) = nodes[1]->GetX0().y();
    J2(2, 1) = nodes[2]->GetX0().y();
    J2(3, 1) = nodes[3]->GetX0().y();
    J2(4, 1) = nodes[4]->GetX0().y();
    J2(5, 1) = nodes[5]->GetX0().y();
    J2(6, 1) = nodes[6]->GetX0().y();
    J2(7, 1) = nodes[7]->GetX0().y();

    J2(0, 2) = nodes[0]->GetX0().z();
    J2(1, 2) = nodes[1]->GetX0().z();
    J2(2, 2) = nodes[2]->GetX0().z();
    J2(3, 2) = nodes[3]->GetX0().z();
    J2(4, 2) = nodes[4]->GetX0().z();
    J2(5, 2) = nodes[5]->GetX0().z();
    J2(6, 2) = nodes[6]->GetX0().z();
    J2(7, 2) = nodes[7]->GetX0().z();

    Jacobian = J1 * J2;
}

void ChElementHexaCorot_8::ComputeMatrB(ChMatrixDynamic<>& MatrB,
                                        double zeta1,
                                        double zeta2,
                                        double zeta3,
                                        double& JacobianDet) {
    ChMatrixDynamic<> Jacobian(3, 3);
    ChMatrixDynamic<> J1(3, 8);
    ComputeJacobian(Jacobian, J1, ChVector<>(zeta1, zeta2, zeta3));

    // !!! store the Jacobian Determinant: needed for the integration
    JacobianDet = Jacobian.determinant();

    ChMatrixDynamic<> Jinv = Jacobian.inverse();

    ChMatrixDynamic<> Btemp = Jinv * J1;

    MatrB.setZero(6, 24);  // Remember to resize the matrix!

    MatrB(0, 0) = Btemp(0, 0);
    MatrB(0, 3) = Btemp(0, 1);
    MatrB(0, 6) = Btemp(0, 2);
    MatrB(0, 9) = Btemp(0, 3);
    MatrB(0, 12) = Btemp(0, 4);
    MatrB(0, 15) = Btemp(0, 5);
    MatrB(0, 18) = Btemp(0, 6);
    MatrB(0, 21) = Btemp(0, 7);

    MatrB(1, 1) = Btemp(1, 0);
    MatrB(1, 4) = Btemp(1, 1);
    MatrB(1, 7) = Btemp(1, 2);
    MatrB(1, 10) = Btemp(1, 3);
    MatrB(1, 13) = Btemp(1, 4);
    MatrB(1, 16) = Btemp(1, 5);
    MatrB(1, 19) = Btemp(1, 6);
    MatrB(1, 22) = Btemp(1, 7);

    MatrB(2, 2) = Btemp(2, 0);
    MatrB(2, 5) = Btemp(2, 1);
    MatrB(2, 8) = Btemp(2, 2);
    MatrB(2, 11) = Btemp(2, 3);
    MatrB(2, 14) = Btemp(2, 4);
    MatrB(2, 17) = Btemp(2, 5);
    MatrB(2, 20) = Btemp(2, 6);
    MatrB(2, 23) = Btemp(2, 7);

    MatrB(3, 0) = Btemp(1, 0);
    MatrB(3, 1) = Btemp(0, 0);
    MatrB(3, 3) = Btemp(1, 1);
    MatrB(3, 4) = Btemp(0, 1);
    MatrB(3, 6) = Btemp(1, 2);
    MatrB(3, 7) = Btemp(0, 2);
    MatrB(3, 9) = Btemp(1, 3);
    MatrB(3, 10) = Btemp(0, 3);
    MatrB(3, 12) = Btemp(1, 4);
    MatrB(3, 13) = Btemp(0, 4);
    MatrB(3, 15) = Btemp(1, 5);
    MatrB(3, 16) = Btemp(0, 5);
    MatrB(3, 18) = Btemp(1, 6);
    MatrB(3, 19) = Btemp(0, 6);
    MatrB(3, 21) = Btemp(1, 7);
    MatrB(3, 22) = Btemp(0, 7);

    MatrB(4, 1) = Btemp(2, 0);
    MatrB(4, 2) = Btemp(1, 0);
    MatrB(4, 4) = Btemp(2, 1);
    MatrB(4, 5) = Btemp(1, 1);
    MatrB(4, 7) = Btemp(2, 2);
    MatrB(4, 8) = Btemp(1, 2);
    MatrB(4, 10) = Btemp(2, 3);
    MatrB(4, 11) = Btemp(1, 3);
    MatrB(4, 13) = Btemp(2, 4);
    MatrB(4, 14) = Btemp(1, 4);
    MatrB(4, 16) = Btemp(2, 5);
    MatrB(4, 17) = Btemp(1, 5);
    MatrB(4, 19) = Btemp(2, 6);
    MatrB(4, 20) = Btemp(1, 6);
    MatrB(4, 22) = Btemp(2, 7);
    MatrB(4, 23) = Btemp(1, 7);

    MatrB(5, 0) = Btemp(2, 0);
    MatrB(5, 2) = Btemp(0, 0);
    MatrB(5, 3) = Btemp(2, 1);
    MatrB(5, 5) = Btemp(0, 1);
    MatrB(5, 6) = Btemp(2, 2);
    MatrB(5, 8) = Btemp(0, 2);
    MatrB(5, 9) = Btemp(2, 3);
    MatrB(5, 11) = Btemp(0, 3);
    MatrB(5, 12) = Btemp(2, 4);
    MatrB(5, 14) = Btemp(0, 4);
    MatrB(5, 15) = Btemp(2, 5);
    MatrB(5, 17) = Btemp(0, 5);
    MatrB(5, 18) = Btemp(2, 6);
    MatrB(5, 20) = Btemp(0, 6);
    MatrB(5, 21) = Btemp(2, 7);
    MatrB(5, 23) = Btemp(0, 7);
}

void ChElementHexaCorot_8::ComputeMatrB(ChGaussPoint* GaussPt, double& JacobianDet) {
    this->ComputeMatrB(*(GaussPt->MatrB), (*GaussPt).GetLocalCoordinates().x(), (*GaussPt).GetLocalCoordinates().y(),
                       (*GaussPt).GetLocalCoordinates().z(), JacobianDet);
}

void ChElementHexaCorot_8::ComputeStiffnessMatrix() {
    double Jdet;
    ChMatrixDynamic<>* temp = new ChMatrixDynamic<>;
    ChMatrixDynamic<> BT;
    this->Volume = 0;

    for (unsigned int i = 0; i < GpVector.size(); i++) {
        ComputeMatrB(GpVector[i], Jdet);
        BT = GpVector[i]->MatrB->transpose();
        *temp = (Jdet * GpVector[i]->GetWeight()) * (BT * Material->Get_StressStrainMatrix() * *(GpVector[i]->MatrB));
        StiffnessMatrix += *temp;

        // by the way also computes volume:
        this->Volume += GpVector[i]->GetWeight() * Jdet;
    }

    delete temp;
}

void ChElementHexaCorot_8::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementHexaCorot_8::UpdateRotation() {
    ChVector<> avgX1;
    avgX1 = nodes[0]->GetX0() + nodes[1]->GetX0() + nodes[2]->GetX0() + nodes[3]->GetX0();
    ChVector<> avgX2;
    avgX2 = nodes[4]->GetX0() + nodes[5]->GetX0() + nodes[6]->GetX0() + nodes[7]->GetX0();
    ChVector<> Xdir = avgX2 - avgX1;

    ChVector<> avgY1;
    avgY1 = nodes[0]->GetX0() + nodes[1]->GetX0() + nodes[4]->GetX0() + nodes[5]->GetX0();
    ChVector<> avgY2;
    avgY2 = nodes[2]->GetX0() + nodes[3]->GetX0() + nodes[6]->GetX0() + nodes[7]->GetX0();
    ChVector<> Ydir = avgY2 - avgY1;
    ChMatrix33<> rotX0;
    rotX0.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

    avgX1 = nodes[0]->pos + nodes[1]->pos + nodes[2]->pos + nodes[3]->pos;
    avgX2 = nodes[4]->pos + nodes[5]->pos + nodes[6]->pos + nodes[7]->pos;
    Xdir = avgX2 - avgX1;

    avgY1 = nodes[0]->pos + nodes[1]->pos + nodes[4]->pos + nodes[5]->pos;
    avgY2 = nodes[2]->pos + nodes[3]->pos + nodes[6]->pos + nodes[7]->pos;
    Ydir = avgY2 - avgY1;
    ChMatrix33<> rotXcurrent;
    rotXcurrent.Set_A_Xdir(Xdir.GetNormalized(), Ydir.GetNormalized());

    this->A = rotXcurrent * rotX0.transpose();
}

ChStrainTensor<> ChElementHexaCorot_8::GetStrain(double z1, double z2, double z3) {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(GetNdofs());
    this->GetStateBlock(displ);

    double JacobianDet;
    ChMatrixDynamic<> amatrB(6, GetNdofs());
    ComputeMatrB(amatrB, z1, z2, z3, JacobianDet);

    ChStrainTensor<> mstrain = amatrB * displ;
    return mstrain;
}

ChStressTensor<> ChElementHexaCorot_8::GetStress(double z1, double z2, double z3) {
    ChStressTensor<> mstress = this->Material->Get_StressStrainMatrix() * this->GetStrain(z1, z2, z3);
    return mstress;
}

void ChElementHexaCorot_8::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == GetNdofs()) && (H.cols() == GetNdofs()));

    // warp the local stiffness matrix K in order to obtain global
    // tangent stiffness CKCt:
    ChMatrixDynamic<> CK(GetNdofs(), GetNdofs());
    ChMatrixDynamic<> CKCt(GetNdofs(), GetNdofs());  // the global, corotated, K matrix, for 8 nodes
    ChMatrixCorotation::ComputeCK(StiffnessMatrix, this->A, 8, CK);
    ChMatrixCorotation::ComputeKCt(CK, this->A, 8, CKCt);

    // For K stiffness matrix and R damping matrix:

    double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();
    H = mkfactor * CKCt;

    // For M mass matrix:
    if (Mfactor) {
        double lumped_node_mass = (this->Volume * this->Material->Get_density()) / 8.0;
        for (int id = 0; id < GetNdofs(); id++) {
            double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
            H(id, id) += amfactor * lumped_node_mass;
        }
    }
    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
}

void ChElementHexaCorot_8::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == GetNdofs());

    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(GetNdofs());
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    for (int in = 0; in < 8; ++in) {
        displ.segment(in * 3, 3) = (A.transpose() * nodes[in]->pos_dt).eigen();  // nodal speeds, local
    }
    ChMatrixDynamic<> FiR_local = Material->Get_RayleighDampingK() * StiffnessMatrix * displ;

    double lumped_node_mass = (this->Volume * this->Material->Get_density()) / 8.0;
    displ *= (lumped_node_mass * Material->Get_RayleighDampingM());
    FiR_local += displ;

    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

    FiK_local += FiR_local;
    FiK_local *= -1.0;

    // Fi = C * Fi_local  with C block-diagonal rotations A
    ChMatrixCorotation::ComputeCK(FiK_local, this->A, 8, Fi);
}

void ChElementHexaCorot_8::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 12, 3) = nodes[4]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = nodes[5]->GetPos().eigen();
    mD.segment(block_offset + 18, 3) = nodes[6]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = nodes[7]->GetPos().eigen();
}

void ChElementHexaCorot_8::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPos_dt().eigen();
    mD.segment(block_offset + 12, 3) = nodes[4]->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = nodes[5]->GetPos_dt().eigen();
    mD.segment(block_offset + 18, 3) = nodes[6]->GetPos_dt().eigen();
    mD.segment(block_offset + 21, 3) = nodes[7]->GetPos_dt().eigen();
}

void ChElementHexaCorot_8::LoadableStateIncrement(const unsigned int off_x,
                                                  ChState& x_new,
                                                  const ChState& x,
                                                  const unsigned int off_v,
                                                  const ChStateDelta& Dv) {
    for (int i = 0; i < 8; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementHexaCorot_8::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementHexaCorot_8::ComputeNF(const double U,
                                     const double V,
                                     const double W,
                                     ChVectorDynamic<>& Qi,
                                     double& detJ,
                                     const ChVectorDynamic<>& F,
                                     ChVectorDynamic<>* state_x,
                                     ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependant on state
    ShapeVector N;
    ShapeFunctions(N, U, V, W);  // note: U,V,W in -1..1 range

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
}

}  // end namespace fea
}  // end namespace chrono
