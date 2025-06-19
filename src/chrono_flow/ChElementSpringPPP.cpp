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

#include "chrono_flow/ChElementSpringPPP.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

ChElementSpringPPP::ChElementSpringPPP() : Volume(0) {
    nodes.resize(2);
    this->MatrB.setZero(3, 6);
    this->StiffnessMatrix.setZero(6, 6);
    this->MassMatrix.setZero(6, 6);
    this->ElementSourceTerm.setZero(6);
}

ChElementSpringPPP::~ChElementSpringPPP() {}

void ChElementSpringPPP::SetNodes(std::shared_ptr<ChNodeFEAxyzPPP> nodeA, 
                                std::shared_ptr<ChNodeFEAxyzPPP> nodeB) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementSpringPPP::SetupInitial(ChSystem* system) {
    ElementState.resize(4);      // 4 is the number of element internal variables 
    ElementSourceTerm.resize(6); // 6 is the size of the source term vector
    ComputeStiffnessMatrix();
    ComputeMassMatrix();
    ComputeSourceTerm();
}

void ChElementSpringPPP::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // compute stiffness matrix
    ComputeStiffnessMatrix();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementSpringPPP::ShapeFunctions(ShapeVector& N, double z0) {
    N(0) = 1 - z0;
    N(1) = z0;
}
             
void ChElementSpringPPP::GetStateBlock(ChVectorDynamic<>& D) {
    D.setZero(this->GetNumCoordsPosLevel());
    D.segment(0, 3) = nodes[0]->GetFieldVal().eigen();
    D.segment(3, 3) = nodes[1]->GetFieldVal().eigen();
}

void ChElementSpringPPP::GetStateBlockDt(ChVectorDynamic<>& D) {
    D.setZero(this->GetNumCoordsPosLevel());
    D.segment(0, 3) = nodes[0]->GetFieldValDt().eigen();
    D.segment(3, 3) = nodes[1]->GetFieldValDt().eigen();
}

void ChElementSpringPPP::ComputeStiffnessMatrix() {
    MatrB(0, 0) =  1;
    MatrB(0, 3) = -1;
    MatrB(1, 1) = -1;
    MatrB(1, 4) =  1;
    MatrB(2, 2) =  1;
    MatrB(2, 5) = -1;

    // RR: here are the element geometry parameters created by FreeCAD
    //std::cout << elL1 << " " << elL2 << " " << elA << " " << elVol << " " << elType << std::endl;

    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    // RR: L1 and L2 needs to be read from mesh file
    double L1 = 0.5 * L; 
    double L2 = 0.5 * L;
    double g1 = L1/L;
    double g2 = L2/L;
    
    // Retrievce Current Nodal Variables (RR: needs to be checked)
    ChVectorDynamic<> NodeVal(6);
    this->GetStateBlock(NodeVal);
    ChVectorDynamic<> NodeValP(3);
    NodeValP(0) = g2*NodeVal(0) + g1*NodeVal(3);
    NodeValP(1) = g2*NodeVal(1) + g1*NodeVal(4);
    NodeValP(2) = g2*NodeVal(2) + g1*NodeVal(5);

    // Retrievce Current Rate of Nodal Variables (RR: needs to be checked)
    ChVectorDynamic<> NodeValDt(6);
    this->GetStateBlockDt(NodeValDt);
    ChVectorDynamic<> NodeValDtP(3);
    NodeValDtP(0) = g2*NodeValDt(0) + g1*NodeValDt(3);
    NodeValDtP(1) = g2*NodeValDt(1) + g1*NodeValDt(4);
    NodeValDtP(2) = g2*NodeValDt(2) + g1*NodeValDt(5);

    ChMatrixDynamic<> temp;
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveMatrixK(NodeValP, NodeValDtP);
    UpdatedConstitutiveMatrix = temp;

    // RR: A needs to be read from mesh file
    double A = 1.0;
    StiffnessMatrix = (A/L) * MatrB.transpose() * UpdatedConstitutiveMatrix * MatrB;
}

void ChElementSpringPPP::ComputeMassMatrix() {
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    // RR: L1 and L2 needs to be read from mesh file
    double L1 = 0.5 * L; 
    double L2 = 0.5 * L;
    double g1 = L1/L;
    double g2 = L2/L;
    // RR nodal volume should be read from mesh file
    double vol = 1.0; 

    // Compute mass matrix for node 2 and assemble
    // Retrievce Current Nodal Variables for Node 1 (RR: needs to be checked) 
    ChVectorDynamic<> NodeVal(6);
    this->GetStateBlock(NodeVal);
    ChVectorDynamic<> NodeValtemp(3);
    NodeValtemp(0) = NodeVal(0);
    NodeValtemp(1) = NodeVal(1);
    NodeValtemp(2) = NodeVal(2);

    // Retrievce Current Rate of Nodal Variables for Node 1 (RR: needs to be checked)
    ChVectorDynamic<> NodeValDt(6);
    this->GetStateBlockDt(NodeValDt);
    ChVectorDynamic<> NodeValDttemp(3);
    NodeValDttemp(0) = NodeValDt(0);
    NodeValDttemp(1) = NodeValDt(1);
    NodeValDttemp(2) = NodeValDt(2);

    // compute mass matrix of the element node 1
    ChMatrixDynamic<> temp;
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveMatrixM(NodeValtemp, NodeValDttemp);
    
    // assemble node 1 mass matrix to element mass matrix
    MassMatrix(0, 0) = g1 * temp(0,0);
    MassMatrix(0, 1) = g1 * temp(0,1);
    MassMatrix(0, 2) = g1 * temp(0,2);
    MassMatrix(1, 0) = g1 * temp(1,0);
    MassMatrix(1, 1) = g1 * temp(1,1);
    MassMatrix(1, 2) = g1 * temp(1,2);
    MassMatrix(2, 0) = g1 * temp(2,0);
    MassMatrix(2, 1) = g1 * temp(2,1);
    MassMatrix(2, 2) = g1 * temp(2,2);

    // Compute mass matrix for node 2 and assemble
    // nodal values of node 2
    NodeValtemp(0) = NodeVal(3);
    NodeValtemp(1) = NodeVal(4);
    NodeValtemp(2) = NodeVal(5);

    // rate of nodal values node 2
    NodeValDttemp(0) = NodeValDt(3);
    NodeValDttemp(1) = NodeValDt(4);
    NodeValDttemp(2) = NodeValDt(5);

    // Compute mass matrix of the element node 2
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveMatrixM(NodeValtemp, NodeValDttemp);

    // Assemble second node mass matrix to element mass matrix
    MassMatrix(0+3, 0+3) = g2 * temp(0,0);
    MassMatrix(0+3, 1+3) = g2 * temp(0,1);
    MassMatrix(0+3, 2+3) = g2 * temp(0,2);
    MassMatrix(1+3, 0+3) = g2 * temp(1,0);
    MassMatrix(1+3, 1+3) = g2 * temp(1,1);
    MassMatrix(1+3, 2+3) = g2 * temp(1,2);
    MassMatrix(2+3, 0+3) = g2 * temp(2,0);
    MassMatrix(2+3, 1+3) = g2 * temp(2,1);
    MassMatrix(2+3, 2+3) = g2 * temp(2,2);

    // Take element volume into account
    MassMatrix *= vol;

    //temp = MassMatrix;
    //std::cout << temp(0,0) << " " << temp(0,1) << " " << temp(0,2) << " " << temp(0,3) << " " << temp(0,4) << " " << temp(0,5) << std::endl;
    //std::cout << temp(1,0) << " " << temp(1,1) << " " << temp(1,2) << " " << temp(1,3) << " " << temp(1,4) << " " << temp(1,5) << std::endl;
    //std::cout << temp(2,0) << " " << temp(2,1) << " " << temp(2,2) << " " << temp(2,3) << " " << temp(2,4) << " " << temp(2,5) << std::endl;
    //std::cout << temp(3,0) << " " << temp(3,1) << " " << temp(3,2) << " " << temp(3,3) << " " << temp(3,4) << " " << temp(3,5) << std::endl;
    //std::cout << temp(4,0) << " " << temp(4,1) << " " << temp(4,2) << " " << temp(4,3) << " " << temp(4,4) << " " << temp(4,5) << std::endl;
    //std::cout << temp(5,0) << " " << temp(5,1) << " " << temp(5,2) << " " << temp(5,3) << " " << temp(5,4) << " " << temp(5,5) << std::endl;
}

void ChElementSpringPPP::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6) && (H.cols() == 6));

    // For K  matrix (jacobian d/dT of  c dT/dt + div [C] grad T = f )
    H = Kfactor * StiffnessMatrix;

    //double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();

    if (Rfactor)
        for (int id = 0; id < 6; id++) {
            H(id, id) += Rfactor * MassMatrix(id, id);
        }       

}

void ChElementSpringPPP::ComputeSourceTerm() {
    // Get element state variables
    ElementState = this->GetElementStateVariable();

    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    // RR: L1 and L2 needs to be read from mesh file
    double L1 = 0.5 * L; 
    double L2 = 0.5 * L;
    double g1 = L1/L;
    double g2 = L2/L;
    // RR nodal volume should be read from mesh file
    double vol = 1.0; 

    ChVectorDynamic<> NodeVal(6);
    this->GetStateBlock(NodeVal);
    ChVectorDynamic<> NodeValtemp(3);
    NodeValtemp(0) = NodeVal(0);
    NodeValtemp(1) = NodeVal(1);
    NodeValtemp(2) = NodeVal(2);

    ChVectorDynamic<> NodeValDt(6);
    this->GetStateBlockDt(NodeValDt);
    ChVectorDynamic<> NodeValDttemp(3);
    NodeValDttemp(0) = NodeValDt(0);
    NodeValDttemp(1) = NodeValDt(1);
    NodeValDttemp(2) = NodeValDt(2);

    // compute source vector of the first element node 
    ChMatrixDynamic<> temp;
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveVectorS(NodeValtemp, NodeValDttemp);
    
    // Assemble first node source vector to element source vector
    ElementSourceTerm(0) = temp(0);
    ElementSourceTerm(1) = temp(1);
    ElementSourceTerm(2) = temp(2);

    NodeValtemp(0) = NodeVal(3);
    NodeValtemp(1) = NodeVal(4);
    NodeValtemp(2) = NodeVal(5);

    NodeValDttemp(0) = NodeValDt(3);
    NodeValDttemp(1) = NodeValDt(4);
    NodeValDttemp(2) = NodeValDt(5);

    // Compute source vector of the second element node 
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveVectorS(NodeValtemp, NodeValDttemp);

    // Assemble second node source vector to element source vector
    ElementSourceTerm(3) = temp(0);
    ElementSourceTerm(4) = temp(1);
    ElementSourceTerm(5) = temp(2);

    //ChVectorDynamic<> tempS;
    //tempS = ElementSourceTerm;
    //std::cout << tempS(0) << " " << tempS(1) << " " << tempS(2) << " " << tempS(3) << " " << tempS(4) << " " << tempS(5) << std::endl;
}

void ChElementSpringPPP::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 6);

    //std::cout << "Compute source term!" << std::endl;
    ComputeSourceTerm();

    // set up vector of nodal fields
    ChVectorDynamic<> NodeVal(6);
    this->GetStateBlock(NodeVal);

    // set up vector of nodal velocity fields
    ChVectorDynamic<> NodeValDt(6);
    this->GetStateBlockDt(NodeValDt);

    // [local Internal Forces] = [Mlocal] * dPdt + [Klocal] * P + src_term
    ChVectorDynamic<> FiK_local = MassMatrix * NodeValDt + StiffnessMatrix * NodeVal + ElementSourceTerm;
    FiK_local *= -1.0;

    // ChMatrixCorotation::ComputeCK(FiK_local, this->A, 4, Fi);  ***corotation NOT NEEDED
    Fi = FiK_local;
}
 
ChVectorN<double, 1> ChElementSpringPPP::GetPgradient() {
    // set up vector of nodal displacements (in local element system) u_l = R*p - p0
    ChVectorDynamic<> displ(6);
    this->GetStateBlock(displ);

    return MatrB * displ;
}

void ChElementSpringPPP::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetFieldVal().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetFieldVal().eigen();
}

void ChElementSpringPPP::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = this->nodes[0]->GetFieldValDt().eigen();
    mD.segment(block_offset + 3, 3) = this->nodes[1]->GetFieldValDt().eigen();
}

void ChElementSpringPPP::LoadableStateIncrement(const unsigned int off_x,
                                              ChState& x_new,
                                              const ChState& x,
                                              const unsigned int off_v,
                                              const ChStateDelta& Dv) {
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
    }
}

void ChElementSpringPPP::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < nodes.size(); ++i)
        mvars.push_back(&this->nodes[i]->Variables());
}

void ChElementSpringPPP::ComputeNF(const double U,
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

void ChElementSpringPPP::ComputeNF(const double U,
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

}  // end namespace fea
}  // end namespace chrono
