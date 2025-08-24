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
#include "chrono_flow/ChFlow3D.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

ChElementSpringPPP::ChElementSpringPPP() : Volume(0) {
    nodes.resize(2);
    this->MatrB.setZero(3, 6);
    this->StiffnessMatrix.setZero(6, 6);
    this->MassMatrix.setZero(6, 6);
    this->ElementSourceTerm.setZero(6);
    //this->ElementState.setZero(4);

    
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
    ElementState.resize(8);      // 4 is the number of element internal variables 
    ElementSourceTerm.resize(6); // 6 is the size of the source term vector
    ComputeStiffnessMatrix();
    ComputeMassMatrix();
    ComputeSourceTerm();

    /*double alpha_inf = Material->GetAlphaInfinity();
    Alpha(0) = alpha_inf;
    Alpha(1) = alpha_inf;
    Alpha0 = Alpha;*/
}   

void ChElementSpringPPP::Update() {
    // parent class update:
    ChElementGeneric::Update();
    // compute stiffness matrix
    ComputeStiffnessMatrix();
    ComputeMassMatrix();
    // always keep updated the rotation matrix A:
    this->UpdateRotation();

    //double T0 = 293.0;  // [K] reference temperature
    //double dt = 1.0;    // can be got dynamically
    //this->UpdateHydration(dt, T0);
    //Alpha0 = Alpha;
    this->ComputeElementVar(dt_current);
}

// void ChElementSpringPPP::UpdateHydration(double dt, double T0) {
//     double patm = 0.1;        // [MPa]
//     double rhol0 = 1.0e-6;    // [kg/mm^3]
//     double Mw = 0.018015;     // [kg/mol]
//     double R = 8.314;         // [J/(mol·K)]
//     double Gl = 2.2e3;        // [N/mm^2]
//     double alphavT = 2.07e-4; // [-]

//     double Wmct = Material->Getwmct();
//     double A1c = Material->GetA1c();
//     double A2c = Material->GetA2c();
//     double etac = Material->GetEtac();
//     double af = Material->GetAf();
//     double bf = Material->GetBf();
//     double Eac_over_R = Material->GetEacOverR();
//     double alpha_inf = Material->GetAlphaInfinity();

//     for (int i = 0; i < 2; ++i) {
//         double T = nodes[i]->GetFieldVal()[0]; // Temperature
//         double h = nodes[i]->GetFieldVal()[1]; // Humidity
//         //double p = 
//         double alpha_old = Alpha0(i);

//         double psat = patm * std::exp((11.9515 * (T - 373.15)) / (T - 39.724));
//         double rhol = rhol0 * (1.0 - alphavT * (T - T0) + (p - patm) / Gl);
//         double Hmed = std::exp((p - psat) * Mw / (R * T * rhol));
//         Hmed = std::min(1.0, Hmed);

//         double affinity = A1c * (A2c / alpha_inf + alpha_old) * (alpha_inf - alpha_old) * std::exp(-alpha_old * etac / alpha_inf);
//         double beta_f = 1.0 / std::pow((1.0 + std::pow((af - af * Hmed), bf)), 1.0);
//         double delta_alpha = dt * affinity * beta_f * std::exp(-Eac_over_R / T);
//         Alpha(i) = std::min(alpha_inf, alpha_old + delta_alpha);
//     }
// }

void ChElementSpringPPP::ComputeElementVar(double dt) {

    // double patm = 0.1;        // [MPa]
    // double rhol0 = 1.0e-6;    // [kg/mm^3]
    // double Mw = 0.018015;     // [kg/mol]
    // double R = 8.314;         // [J/(mol·K)]
    // double Gl = 2.2e3;        // [N/mm^2]
    // double alphavT = 2.07e-4; // [-]
    

    double Wmct = Material->Getwmct();
    double A1c = Material->GetA1c();
    double A2c = Material->GetA2c();
    double etac = Material->GetEtac();
    double af = Material->GetAf();
    double bf = Material->GetBf();
    double Eac_over_R = Material->GetEacOverR();

    double Eas_over_R = Material->GetEasOverR();
    double B1_S = Material->GetB1();
    double B2_S = Material->GetB2();
    double B3_S = Material->GetB3();
    double B5_S = Material->GetB5();

    double ct = Material->Getct_1();
    double Lampda= Material->GetLampda_1();
    double Q_hydr_inf = Material->GetQ_hydr_inf_1();
    double Q_S_inf = Material->GetQ_S_inf_1();
    
    double rho= Material->Getrho_1();
    double CEMENT = Material->GetCEMENT_1();
    double SILICA = Material->GetSILICA_1();
    double AGGREGATE = Material->GetAGGREGATE_1();
    double k_c = Material->Getk_c_1();
    double g_1 = Material->Getg_1_1();
    double k_vg_c = Material->Getk_vg_c_1();
    double k_vg_s = Material->Getk_vg_s_1();
    double Q_over_R = Material->GetQ_over_R_1();
    
    double T0 = 293.0;
    double W0 = Wmct*CEMENT;

    // double alpha_inf = 1.032 * Wmct / (0.194 + Wmct);
    double alpha_S_inf = 0.0;
    double Silica_to_Cement = CEMENT > 0.0 ? SILICA / CEMENT : 0.0;
    if (Wmct > 0.4 && Silica_to_Cement <= 0.16) {
        alpha_S_inf = B5_S;
    } else if (Wmct <= 0.4 && Silica_to_Cement <= 0.4 * Wmct) {
        alpha_S_inf = B5_S;
    } else if (Wmct > 0.4 && Silica_to_Cement > 0.16) {
        alpha_S_inf = B5_S * 0.16 / Silica_to_Cement;
    } else if (Wmct <= 0.4 && Silica_to_Cement > 0.4 * Wmct) {
        alpha_S_inf = B5_S * 0.4 * Wmct / Silica_to_Cement;
    }
    double alpha_inf = (1.032 * Wmct - 0.279 * Silica_to_Cement * alpha_S_inf) / (0.194 + Wmct);

    // Obtain the element state variable of last step
    ChVectorDynamic<> ElStateTemp(9);
    ElStateTemp = this->GetElementStateVariable();

    // Current T, h of the element
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L1 = 0.5 * L;
    double L2 = 0.5 * L;
    double g1 = L1 / L;
    double g2 = L2 / L;
    
    ChVectorDynamic<> NodeVal(6);
    this->GetStateBlock(NodeVal);
    double tp = g2 * NodeVal(0) + g1 * NodeVal(3); //293;  // T
    double hp = g2 * NodeVal(1) + g1 * NodeVal(4); //0.9999; // P or RH

    // std::cout << "tp=" << tp         
    //           << "hp=" << hp << std::endl;
    
    // calculate hydration degree
    double ALPHA = ElStateTemp(0);
    double Affinity = A1c * (A2c / alpha_inf + ALPHA)
                            * (alpha_inf - ALPHA)
                            * std::exp(-ALPHA * etac / alpha_inf);
    // double xx = std::pow(std::min(hp, 1.0), bf);
    // double beta_f = std::pow(1.0 + (af - af * xx), -1.0);
    double beta_f = 1.0/(1.0 + std::pow(af-af*std::min(hp, 1.0), bf));
    double Arrhenius = std::exp(-Eac_over_R / tp);
    // double Arrhenius = std::exp(-Eac_over_R / 293.0);
    double dalpha_dt = Affinity * beta_f * Arrhenius;
    //double dalpha_dt = Affinity * beta_f;
    double ALPHA_new = ALPHA + dt * dalpha_dt;

    // calculate silica fume reaction degree
    double ALPHA_S = ElStateTemp(2);
    double dalphas_dt = 0.0;
    double ALPHAS_new = 0.0;
    if (SILICA > 0.0) {
        double Affinity_S = B1_S * (B2_S / alpha_S_inf + ALPHA_S) * (alpha_S_inf - ALPHA_S) * std::exp(-ALPHA_S * B3_S / alpha_S_inf);
        dalphas_dt = Affinity_S * std::exp(-Eas_over_R / tp);
        ALPHAS_new = ALPHA_S + dalphas_dt * dt;
    } 

    /* std::cout << "ALPHA=" << ALPHA 
               << "  " 
               << "dalpha_dt=" << dalpha_dt << std::endl;*/
    /*std::cout << "SILICA=" << SILICA << "  "
              << "k_vg_s=" << k_vg_s << "  "
              << "ALPHAS_new=" << ALPHAS_new << std::endl;*/

    ElStateTemp(0) = ALPHA_new;
    ElStateTemp(1) = dalpha_dt;
    ElStateTemp(2) = ALPHAS_new;
    ElStateTemp(3) = dalphas_dt;

    // calculate sorption isotherm
    double b_11 = g_1*10.0*alpha_inf; 
    double gixs = std::exp(b_11-10.0*ALPHA_new);
    double psi = 1.0; // std::exp(Q_over_R*(1/tp-1/T0));
    double gixsh = std::exp(hp * (b_11 - 10.0 * ALPHA_new));
    double G_11 = k_vg_c*CEMENT*ALPHA_new + k_vg_s*SILICA*ALPHAS_new;
    double gK_1 = (W0 - 0.188 * CEMENT * ALPHA_new + 0.22 * SILICA * ALPHAS_new - G_11 * (1.0 - 1.0 / gixs)) / (gixs - 1.0);
    double m_cap = (G_11 / gixsh * psi + gK_1 * gixsh) * (b_11 - 10.0 * ALPHA_new);  //% dev_we_h
    // double dev_we_T = (-Q_over_R/tp/tp)*G_11*((1-std::exp(-(b_11-10.0*ALPHA_new)*hp)))*psi; // % - (1.0-1.0/gixs)/(gixs-1.0)*(exp((b_11-10.0*ALPHA_med)*Hmed)-1));
    double we_sat = G_11*(1-std::exp(-(b_11-10.0*ALPHA_new)))*psi+gK_1*(std::exp((b_11-10.0*ALPHA_new))-1);
    double we = G_11*(1-std::exp(-(b_11-10.0*ALPHA_new)*hp))*psi+gK_1*(std::exp((b_11-10.0*ALPHA_new)*hp)-1);
    double Sl = we/we_sat;
    double np = we_sat/rho;

    //m_cap = 2.50e-7; 
    Material->SetCurrentCap(m_cap);

    double A11 = (gixsh - 1.0)*(-0.188*CEMENT+10.0*G_11/gixs+k_vg_c*CEMENT*(1.0/gixs-1.0))/(gixs-1.0);
    double A12 = -10.0*hp/gixsh*G_11*psi;
    double A13 = -k_vg_c*CEMENT*(1.0/gixsh - 1.0)*psi;
    double A14 = (10.0 * gixs * (gixsh - 1.0)) * (W0 - 0.188 * CEMENT * ALPHA_new + 0.22 * SILICA * ALPHAS_new + G_11 * (1.0 / gixs - 1.0))/(gixs - 1.0) / (gixs - 1.0);
    double A15 = -(10.0 * hp * gixsh) * (W0 - 0.188 * CEMENT * ALPHA_new + 0.22 * SILICA * ALPHAS_new + G_11 * (1.0 / gixs - 1.0))/(gixs - 1.0);
    double A21 = (gixsh - 1.0) * (0.22 * SILICA + k_vg_s * SILICA * (1.0 / gixs - 1.0)) / (gixs - 1.0); 
    double A22 = -k_vg_s * SILICA * (1.0 / gixsh - 1.0);
    double AA = A11 + A12 + A13 + A14 + A15; 
    double AAS = A21 + A22;
    //std::cout << "AA = " << AA;
    // std::cout << "AA=" << AA
    //           << "AAS=" << AAS << std::endl;

    ElStateTemp(4) = AA;
    ElStateTemp(5) = AAS;
    ElStateTemp(6) = we;
    ElStateTemp(7) = Sl;
    ElStateTemp(8) = np;
    //std::cout << "m_cap = " << ElStateTemp(4);

    this->SetElementStateVariable(ElStateTemp);
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
    //ElementState = this->GetElementStateVariable();
    //double m_cap = ElementState(4);
    //std::cout << "m_cap = " << m_cap;
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
    double dalpha_dt = ElementState(1);
    double AA = ElementState(4);
    double dalphas_dt = ElementState(3);
    double AAS = ElementState(5);
    //double dalpha_dt = 1.0; 
    //std::cout << " dalpha_dt=" << dalpha_dt;

    ChMatrixDynamic<> temp;
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveVectorS(NodeValtemp, NodeValDttemp, dalpha_dt, AA, dalphas_dt, AAS);
    
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
    //double dalpha_dt_2 = ElementState(1); 
    temp = this->GetMaterial()->ComputeUpdatedConstitutiveVectorS(NodeValtemp, NodeValDttemp, dalpha_dt, AA, dalphas_dt, AAS);

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
