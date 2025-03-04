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

#include "chrono_wood/ChElementSpringP.h"


namespace chrono {
namespace wood {

ChElementSpringP::ChElementSpringP() : Volume(0) {
    nodes.resize(2);
    this->MatrB.setZero(3, 6);
    this->StiffnessMatrix.setZero(6, 6);
}

ChElementSpringP::~ChElementSpringP() {}

void ChElementSpringP::SetNodes(std::shared_ptr<ChNodeFEAxyzPP> nodeA, 
                                std::shared_ptr<ChNodeFEAxyzPP> nodeB) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementSpringP::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementSpringP::ShapeFunctions(ShapeVector& N, double z0) {
    N(0) = 1 - z0;
    N(1) = z0;
}
             
void ChElementSpringP::GetStateBlock(ChVectorDynamic<>& D) {
    D.setZero(this->GetNumCoordsPosLevel());
    D.segment(0, 3) = nodes[0]->GetFieldVal().eigen();
    D.segment(3, 3) = nodes[1]->GetFieldVal().eigen();
}

void ChElementSpringP::ComputeStiffnessMatrix() {
    MatrB(0, 0) =  1;
    MatrB(0, 3) = -1;
    MatrB(1, 1) = -1;
    MatrB(1, 4) =  1;
    MatrB(2, 2) =  1;
    MatrB(2, 5) = -1;

    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();

    ChMatrixDynamic<> temp;
    temp = this->GetMaterial()->GetConstitutiveMatrix();
    //std::cout << "Elasticity Tensor being printed!" << std::endl;
    //std::cout << temp(0,0) << " " << temp(0,1) << " " << temp(0,2) << " " << temp(0,3) << " " << temp(0,4) << " " << temp(0,5) << std::endl;
    //std::cout << temp(1,0) << " " << temp(1,1) << " " << temp(1,2) << " " << temp(1,3) << " " << temp(1,4) << " " << temp(1,5) << std::endl;
    //std::cout << temp(2,0) << " " << temp(2,1) << " " << temp(2,2) << " " << temp(2,3) << " " << temp(2,4) << " " << temp(2,5) << std::endl;
    //std::cout << temp(3,0) << " " << temp(3,1) << " " << temp(3,2) << " " << temp(3,3) << " " << temp(3,4) << " " << temp(3,5) << std::endl;
    //std::cout << temp(4,0) << " " << temp(4,1) << " " << temp(4,2) << " " << temp(4,3) << " " << temp(4,4) << " " << temp(4,5) << std::endl;
    //std::cout << temp(5,0) << " " << temp(5,1) << " " << temp(5,2) << " " << temp(5,3) << " " << temp(5,4) << " " << temp(5,5) << std::endl;
    
    StiffnessMatrix = (1/L) * MatrB.transpose() * Material->GetConstitutiveMatrix() * MatrB;

    //std::cout << "Stiffness Matrix is being computed!" << std::endl;
    //temp = StiffnessMatrix;
    //std::cout << temp(0,0) << " " << temp(0,1) << " " << temp(0,2) << " " << temp(0,3) << " " << temp(0,4) << " " << temp(0,5) << std::endl;
    //std::cout << temp(1,0) << " " << temp(1,1) << " " << temp(1,2) << " " << temp(1,3) << " " << temp(1,4) << " " << temp(1,5) << std::endl;
    //std::cout << temp(2,0) << " " << temp(2,1) << " " << temp(2,2) << " " << temp(2,3) << " " << temp(2,4) << " " << temp(2,5) << std::endl;
    //std::cout << temp(3,0) << " " << temp(3,1) << " " << temp(3,2) << " " << temp(3,3) << " " << temp(3,4) << " " << temp(3,5) << std::endl;
    //std::cout << temp(4,0) << " " << temp(4,1) << " " << temp(4,2) << " " << temp(4,3) << " " << temp(4,4) << " " << temp(4,5) << std::endl;
    //std::cout << temp(5,0) << " " << temp(5,1) << " " << temp(5,2) << " " << temp(5,3) << " " << temp(5,4) << " " << temp(5,5) << std::endl;
}

void ChElementSpringP::SetupInitial(ChSystem* system) {
    ComputeStiffnessMatrix();
}

void ChElementSpringP::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6) && (H.cols() == 6));

    // For K  matrix (jacobian d/dT of  c dT/dt + div [C] grad T = f )
    H = Kfactor * StiffnessMatrix;

    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();

    // For R  matrix: (jacobian d/d\dot(T) of  c dT/dt + div [C] grad T = f )
    if (Rfactor)
        if (this->GetMaterial()->Get_DtMultiplier()) {
            double lumped_node_c = (L * this->GetMaterial()->Get_DtMultiplier()) / 2.0;
            for (int id = 0; id < 6; id++) {
                H(id, id) += Rfactor * lumped_node_c;
            }
        }
    //// TODO  better per-node lumping, or 4x4 consistent c integration as per mass matrices.

    // For M mass matrix: NONE in Poisson equation c dT/dt + div [C] grad T = f

    // finally, do nothing about mass matrix because this element is mass-less
}

void ChElementSpringP::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 6);

    //std::cout << "Compute internal forces!" << std::endl;

    // set up vector of nodal fields
    ChVectorDynamic<> displ(6);
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * P
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    //// TODO  derivative terms? + [Rlocal] * P_dt ???? ***NO because Poisson  rho dP/dt + div [C] grad P = 0
    FiK_local *= -1.0;

    // ChMatrixCorotation::ComputeCK(FiK_local, this->A, 4, Fi);  ***corotation NOT NEEDED
    Fi = FiK_local;
}

ChVectorN<double, 1> ChElementSpringP::GetPgradient() {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(6);
    this->GetStateBlock(displ);

    return MatrB * displ;
}

void ChElementSpringP::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetFieldVal().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetFieldVal().eigen();
}

void ChElementSpringP::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetFieldValDt().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetFieldValDt().eigen();
}

void ChElementSpringP::LoadableStateIncrement(const unsigned int off_x,
                                              ChState& x_new,
                                              const ChState& x,
                                              const unsigned int off_v,
                                              const ChStateDelta& Dv) {
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementSpringP::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementSpringP::ComputeNF(const double U,
                                ChVectorDynamic<>& Qi,
                                double& detJ,
                                const ChVectorDynamic<>& F,
                                ChVectorDynamic<>* state_x,
                                ChVectorDynamic<>* state_w) {
    // evaluate shape functions (in compressed vector), btw. not dependant on state
    ShapeVector N;
    this->ShapeFunctions(N, U);  // note: U,V,W in 0..1 range, thanks to IsTetrahedronIntegrationNeeded() {return true;}

    detJ = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();

    //Qi(0) = N(0) * F(0);
    //Qi(1) = N(1) * F(0);
    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(3, 3);
}

void ChElementSpringP::ComputeNF(const double U,
                                   const double V,
                                   const double W,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    //detJ /= 4.0;  // because volume
}

}  // end namespace wood
}  // end namespace chrono
