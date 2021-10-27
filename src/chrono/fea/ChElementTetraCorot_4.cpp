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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChElementTetraCorot_4.h"

namespace chrono {
namespace fea {

ChElementTetraCorot_4::ChElementTetraCorot_4() : Volume(0) {
    nodes.resize(4);
    this->MatrB.setZero(6, 12);
    this->StiffnessMatrix.setZero(12, 12);
}

ChElementTetraCorot_4::~ChElementTetraCorot_4() {}

void ChElementTetraCorot_4::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                     std::shared_ptr<ChNodeFEAxyz> nodeB,
                                     std::shared_ptr<ChNodeFEAxyz> nodeC,
                                     std::shared_ptr<ChNodeFEAxyz> nodeD) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementTetraCorot_4::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementTetraCorot_4::ShapeFunctions(ShapeVector& N, double r, double s, double t) {
    N(0) = 1.0 - r - s - t;
    N(1) = r;
    N(2) = s;
    N(3) = t;
}

void ChElementTetraCorot_4::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNdofs());
    mD.segment(0, 3) = (A.transpose() * nodes[0]->pos - nodes[0]->GetX0()).eigen();
    mD.segment(3, 3) = (A.transpose() * nodes[1]->pos - nodes[1]->GetX0()).eigen();
    mD.segment(6, 3) = (A.transpose() * nodes[2]->pos - nodes[2]->GetX0()).eigen();
    mD.segment(9, 3) = (A.transpose() * nodes[3]->pos - nodes[3]->GetX0()).eigen();
}

double ChElementTetraCorot_4::ComputeVolume() {
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

void ChElementTetraCorot_4::ComputeStiffnessMatrix() {
    // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
    //     [ 1    1    1    1    ]
    ChMatrixNM<double, 4, 4> tmp;
    tmp.block(0, 0, 3, 1) = nodes[0]->GetX0().eigen();
    tmp.block(0, 1, 3, 1) = nodes[1]->GetX0().eigen();
    tmp.block(0, 2, 3, 1) = nodes[2]->GetX0().eigen();
    tmp.block(0, 3, 3, 1) = nodes[3]->GetX0().eigen();
    tmp.row(3).setConstant(1.0);
    mM = tmp.inverse();

    ////MatrB.Reset(6, 12);
    MatrB(0) = mM(0);
    MatrB(3) = mM(4);
    MatrB(6) = mM(8);
    MatrB(9) = mM(12);
    MatrB(13) = mM(1);
    MatrB(16) = mM(5);
    MatrB(19) = mM(9);
    MatrB(22) = mM(13);
    MatrB(26) = mM(2);
    MatrB(29) = mM(6);
    MatrB(32) = mM(10);
    MatrB(35) = mM(14);
    MatrB(36) = mM(1);
    MatrB(37) = mM(0);
    MatrB(39) = mM(5);
    MatrB(40) = mM(4);
    MatrB(42) = mM(9);
    MatrB(43) = mM(8);
    MatrB(45) = mM(13);
    MatrB(46) = mM(12);
    MatrB(49) = mM(2);
    MatrB(50) = mM(1);
    MatrB(52) = mM(6);
    MatrB(53) = mM(5);
    MatrB(55) = mM(10);
    MatrB(56) = mM(9);
    MatrB(58) = mM(14);
    MatrB(59) = mM(13);
    MatrB(60) = mM(2);
    MatrB(62) = mM(0);
    MatrB(63) = mM(6);
    MatrB(65) = mM(4);
    MatrB(66) = mM(10);
    MatrB(68) = mM(8);
    MatrB(69) = mM(14);
    MatrB(71) = mM(12);

    StiffnessMatrix = Volume * MatrB.transpose() * Material->Get_StressStrainMatrix() * MatrB;

    // ***TEST*** SYMMETRIZE TO AVOID ROUNDOFF ASYMMETRY
    for (int row = 0; row < StiffnessMatrix.rows() - 1; ++row)
        for (int col = row + 1; col < StiffnessMatrix.cols(); ++col)
            StiffnessMatrix(row, col) = StiffnessMatrix(col, row);

    double max_err = 0;
    int err_r = -1;
    int err_c = -1;
    for (int row = 0; row < StiffnessMatrix.rows(); ++row)
        for (int col = 0; col < StiffnessMatrix.cols(); ++col) {
            double diff = fabs(StiffnessMatrix(row, col) - StiffnessMatrix(col, row));
            if (diff > max_err) {
                max_err = diff;
                err_r = row;
                err_c = col;
            }
        }
    if (max_err > 1e-10)
        GetLog() << "NONSYMMETRIC local stiffness matrix! err " << max_err << " at " << err_r << "," << err_c << "\n";
}

void ChElementTetraCorot_4::SetupInitial(ChSystem* system) {
    ComputeVolume();
    ComputeStiffnessMatrix();
}

void ChElementTetraCorot_4::UpdateRotation() {
    // P = [ p_0  p_1  p_2  p_3 ]
    //     [ 1    1    1    1   ]
    ChMatrixNM<double, 4, 4> P;
    P.block(0, 0, 3, 1) = nodes[0]->pos.eigen();
    P.block(0, 1, 3, 1) = nodes[1]->pos.eigen();
    P.block(0, 2, 3, 1) = nodes[2]->pos.eigen();
    P.block(0, 3, 3, 1) = nodes[3]->pos.eigen();
    P.row(3).setConstant(1.0);

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

    // GetLog() << "FEM rotation: \n" << A << "\n" ;
}

void ChElementTetraCorot_4::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));

    // warp the local stiffness matrix K in order to obtain global
    // tangent stiffness CKCt:
    ChMatrixDynamic<> CK(12, 12);
    ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix
    ChMatrixCorotation::ComputeCK(StiffnessMatrix, this->A, 4, CK);
    ChMatrixCorotation::ComputeKCt(CK, this->A, 4, CKCt);

    // ***TEST*** SYMMETRIZE TO AVOID ROUNDOFF ASYMMETRY
    for (int row = 0; row < CKCt.rows() - 1; ++row)
        for (int col = row + 1; col < CKCt.cols(); ++col)
            CKCt(row, col) = CKCt(col, row);

    //***DEBUG***
    double max_err = 0;
    int err_r = -1;
    int err_c = -1;
    for (int row = 0; row < StiffnessMatrix.rows(); ++row)
        for (int col = 0; col < StiffnessMatrix.cols(); ++col) {
            double diff = fabs(StiffnessMatrix(row, col) - StiffnessMatrix(col, row));
            if (diff > max_err) {
                max_err = diff;
                err_r = row;
                err_c = col;
            }
        }
    if (max_err > 1e-10)
        GetLog() << "NONSYMMETRIC local stiffness matrix! err " << max_err << " at " << err_r << "," << err_c << "\n";
    max_err = 0;
    err_r = -1;
    err_c = -1;
    double maxval = 0;
    for (int row = 0; row < CKCt.rows(); ++row)
        for (int col = 0; col < CKCt.cols(); ++col) {
            double diff = fabs(CKCt(row, col) - CKCt(col, row));
            if (diff > max_err) {
                max_err = diff;
                err_r = row;
                err_c = col;
            }
            if (CKCt(row, col) > maxval)
                maxval = CKCt(row, col);
        }
    if (max_err > 1e-10)
        GetLog() << "NONSYMMETRIC corotated matrix! err " << max_err << " at " << err_r << "," << err_c
                 << ",   maxval=" << maxval << "\n";

    // For K stiffness matrix and R damping matrix:
    double mkfactor = Kfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingK();
    H = mkfactor * CKCt;

    // For M mass matrix:
    if (Mfactor) {
        double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / 4.0;
        for (int id = 0; id < 12; id++) {
            double amfactor = Mfactor + Rfactor * this->GetMaterial()->Get_RayleighDampingM();
            H(id, id) += amfactor * lumped_node_mass;
        }
    }
    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.
}

void ChElementTetraCorot_4::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 12);

    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);  // nodal displacements, local

    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    displ.segment(0, 3) = (A.transpose() * nodes[0]->pos_dt).eigen();  // nodal speeds, local
    displ.segment(3, 3) = (A.transpose() * nodes[1]->pos_dt).eigen();
    displ.segment(6, 3) = (A.transpose() * nodes[2]->pos_dt).eigen();
    displ.segment(9, 3) = (A.transpose() * nodes[3]->pos_dt).eigen();
    ChMatrixDynamic<> FiR_local = Material->Get_RayleighDampingK() * StiffnessMatrix * displ;

    double lumped_node_mass = (this->GetVolume() * this->Material->Get_density()) / 4.0;
    displ *= lumped_node_mass * this->Material->Get_RayleighDampingM();
    FiR_local += displ;

    //***TO DO*** better per-node lumping, or 12x12 consistent mass matrix.

    FiK_local += FiR_local;
    FiK_local *= -1.0;

    // Fi = C * Fi_local  with C block-diagonal rotations A
    ChMatrixCorotation::ComputeCK(FiK_local, this->A, 4, Fi);
}

ChStrainTensor<> ChElementTetraCorot_4::GetStrain() {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);  // nodal displacements, local

    ChStrainTensor<> mstrain = MatrB * displ;
    return mstrain;
}

ChStressTensor<> ChElementTetraCorot_4::GetStress() {
    ChStressTensor<> mstress = this->Material->Get_StressStrainMatrix() * this->GetStrain();
    return mstress;
}

void ChElementTetraCorot_4::ComputeNodalMass() {
    nodes[0]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[1]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[2]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
    nodes[3]->m_TotalMass += this->GetVolume() * this->Material->Get_density() / 4.0;
}

void ChElementTetraCorot_4::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPos().eigen();
}

void ChElementTetraCorot_4::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 6, 3) = nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[3]->GetPos_dt().eigen();
}

void ChElementTetraCorot_4::LoadableStateIncrement(const unsigned int off_x,
                                                   ChState& x_new,
                                                   const ChState& x,
                                                   const unsigned int off_v,
                                                   const ChStateDelta& Dv) {
    for (int i = 0; i < 4; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementTetraCorot_4::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementTetraCorot_4::ComputeNF(const double U,
                                      const double V,
                                      const double W,
                                      ChVectorDynamic<>& Qi,
                                      double& detJ,
                                      const ChVectorDynamic<>& F,
                                      ChVectorDynamic<>* state_x,
                                      ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependent on state
    // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}
    ShapeVector N;
    this->ShapeFunctions(N, U, V, W);

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
}

// -----------------------------------------------------------------------------

ChElementTetraCorot_4_P::ChElementTetraCorot_4_P() : Volume(0) {
    nodes.resize(4);
    this->MatrB.setZero(3, 4);
    this->StiffnessMatrix.setZero(4, 4);
}

void ChElementTetraCorot_4_P::SetNodes(std::shared_ptr<ChNodeFEAxyzP> nodeA,
                                       std::shared_ptr<ChNodeFEAxyzP> nodeB,
                                       std::shared_ptr<ChNodeFEAxyzP> nodeC,
                                       std::shared_ptr<ChNodeFEAxyzP> nodeD) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementTetraCorot_4_P::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementTetraCorot_4_P::ShapeFunctions(ShapeVector& N, double z0, double z1, double z2) {
    N(0) = z0;
    N(1) = z1;
    N(2) = z2;
    N(3) = 1.0 - z0 - z1 - z2;
}

void ChElementTetraCorot_4_P::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNdofs());
    mD(0) = nodes[0]->GetP();
    mD(1) = nodes[1]->GetP();
    mD(2) = nodes[2]->GetP();
    mD(3) = nodes[3]->GetP();
}

double ChElementTetraCorot_4_P::ComputeVolume() {
    ChVector<> B1, C1, D1;
    B1.Sub(nodes[1]->GetPos(), nodes[0]->GetPos());
    C1.Sub(nodes[2]->GetPos(), nodes[0]->GetPos());
    D1.Sub(nodes[3]->GetPos(), nodes[0]->GetPos());
    ChMatrixDynamic<> M(3, 3);
    M.col(0) = B1.eigen();
    M.col(1) = C1.eigen();
    M.col(2) = D1.eigen();
    M.transposeInPlace();
    Volume = std::abs(M.determinant() / 6);
    return Volume;
}

void ChElementTetraCorot_4_P::ComputeStiffnessMatrix() {
    // M = [ X0_0 X0_1 X0_2 X0_3 ] ^-1
    //     [ 1    1    1    1    ]
    ChMatrixNM<double, 4, 4> tmp;
    tmp.block(0, 0, 3, 1) = nodes[0]->GetPos().eigen();
    tmp.block(0, 1, 3, 1) = nodes[1]->GetPos().eigen();
    tmp.block(0, 2, 3, 1) = nodes[2]->GetPos().eigen();
    tmp.block(0, 3, 3, 1) = nodes[3]->GetPos().eigen();
    tmp.row(3).setConstant(1.0);
    mM = tmp.inverse();

    ////MatrB.Reset(3, 4);
    MatrB(0, 0) = mM(0);
    MatrB(0, 1) = mM(4);
    MatrB(0, 2) = mM(8);
    MatrB(0, 3) = mM(12);
    MatrB(1, 0) = mM(1);
    MatrB(1, 1) = mM(5);
    MatrB(1, 2) = mM(9);
    MatrB(1, 3) = mM(13);
    MatrB(2, 0) = mM(2);
    MatrB(2, 1) = mM(6);
    MatrB(2, 2) = mM(10);
    MatrB(2, 3) = mM(14);

    StiffnessMatrix = Volume * MatrB.transpose() * Material->Get_ConstitutiveMatrix() * MatrB;
}

void ChElementTetraCorot_4_P::SetupInitial(ChSystem* system) {
    ComputeVolume();
    ComputeStiffnessMatrix();
}

void ChElementTetraCorot_4_P::UpdateRotation() {
    // P = [ p_0  p_1  p_2  p_3 ]
    //     [ 1    1    1    1   ]
    ChMatrixNM<double, 4, 4> P;
    P.block(0, 0, 3, 1) = nodes[0]->GetPos().eigen();
    P.block(0, 1, 3, 1) = nodes[1]->GetPos().eigen();
    P.block(0, 2, 3, 1) = nodes[2]->GetPos().eigen();
    P.block(0, 3, 3, 1) = nodes[3]->GetPos().eigen();
    P.row(3).setConstant(1.0);

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
}

void ChElementTetraCorot_4_P::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 4) && (H.cols() == 4));

    // For K  matrix (jacobian d/dT of  c dT/dt + div [C] grad T = f )
    H = Kfactor * StiffnessMatrix;

    // For R  matrix: (jacobian d/d\dot(T) of  c dT/dt + div [C] grad T = f )
    if (Rfactor)
        if (this->GetMaterial()->Get_DtMultiplier()) {
            // lumped approx. integration of c
            double lumped_node_c = (this->GetVolume() * this->GetMaterial()->Get_DtMultiplier()) / 4.0;
            for (int id = 0; id < 4; id++) {
                H(id, id) += Rfactor * lumped_node_c;
            }
        }
    //***TO DO*** better per-node lumping, or 4x4 consistent c integration as per mass matrices.

    // For M mass matrix: NONE in Poisson equation c dT/dt + div [C] grad T = f
}

void ChElementTetraCorot_4_P::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 4);

    // set up vector of nodal fields
    ChVectorDynamic<> displ(4);
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * P
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    //***TO DO*** derivative terms? + [Rlocal] * P_dt ???? ***NO because Poisson  rho dP/dt + div [C] grad P = 0

    FiK_local *= -1.0;

    // ChMatrixCorotation::ComputeCK(FiK_local, this->A, 4, Fi);  ***corotation NOT NEEDED

    Fi = FiK_local;
}

ChVectorN<double, 3> ChElementTetraCorot_4_P::GetPgradient() {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(4);
    this->GetStateBlock(displ);

    return MatrB * displ;
}

void ChElementTetraCorot_4_P::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD(block_offset) = this->nodes[0]->GetP();
    mD(block_offset + 1) = this->nodes[1]->GetP();
    mD(block_offset + 2) = this->nodes[2]->GetP();
    mD(block_offset + 3) = this->nodes[3]->GetP();
}

void ChElementTetraCorot_4_P::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD(block_offset) = this->nodes[0]->GetP_dt();
    mD(block_offset + 1) = this->nodes[1]->GetP_dt();
    mD(block_offset + 2) = this->nodes[2]->GetP_dt();
    mD(block_offset + 3) = this->nodes[3]->GetP_dt();
}

void ChElementTetraCorot_4_P::LoadableStateIncrement(const unsigned int off_x,
                                                     ChState& x_new,
                                                     const ChState& x,
                                                     const unsigned int off_v,
                                                     const ChStateDelta& Dv) {
    for (int i = 0; i < 4; ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 1, x_new, x, off_v + i * 1, Dv);
    }
}

void ChElementTetraCorot_4_P::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementTetraCorot_4_P::ComputeNF(const double U,
                                        const double V,
                                        const double W,
                                        ChVectorDynamic<>& Qi,
                                        double& detJ,
                                        const ChVectorDynamic<>& F,
                                        ChVectorDynamic<>* state_x,
                                        ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependant on state
    ShapeVector N;
    this->ShapeFunctions(N, U, V,
                         W);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

    detJ = 6 * this->GetVolume();

    Qi(0) = N(0) * F(0);
    Qi(1) = N(1) * F(0);
    Qi(2) = N(2) * F(0);
    Qi(3) = N(3) * F(0);
}

}  // end namespace fea
}  // end namespace chrono
