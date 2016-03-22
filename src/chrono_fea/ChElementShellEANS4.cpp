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
// Authors: Alessandro Tasora
// =============================================================================
// Four node shell with geom.exact kinematics
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChElementShellEANS4.h"
#include "chrono_fea/ChUtilsFEA.h"
#include <cmath>

namespace chrono {
namespace fea {


ChMaterialShellEANS::ChMaterialShellEANS(
                        double thickness, ///< thickness
                        double rho,  ///< material density
                        double E,    ///< Young's modulus
                        double nu,   ///< Poisson ratio
                        double alpha,///< shear factor
                        double beta  ///< torque factor
                        ) {
    m_thickness = thickness;
    m_rho = rho;
    m_E = E;
    m_nu = nu;
    m_alpha = alpha;
    m_beta = beta;
}


void ChMaterialShellEANS::ComputeStress(ChVector<>& n_u, 
                               ChVector<>& n_v,
                               ChVector<>& m_u, 
                               ChVector<>& m_v,
                               const ChVector<>& eps_u, 
                               const ChVector<>& eps_v,
                               const ChVector<>& kur_u, 
                               const ChVector<>& kur_v){
    double h = m_thickness;
    double G = m_E / (2.*(1.+m_nu));
    double C = m_E*h / (1. - m_nu*m_nu);
    double D = C*h*h / 12.;
    double F = G*h*h*h / 12.;

    n_u.x = eps_u.x * C  + eps_v.y * m_nu*C;
    n_u.y = eps_u.y * 2*G*h;
    n_u.z = eps_u.z * m_alpha * G *h;
    n_v.x = eps_v.x * 2*G*h;
    n_v.y = eps_v.y * C  + eps_u.x * m_nu*C;
    n_v.z = eps_v.z * m_alpha * G *h;
    
    m_u.x = kur_u.x * 2* F;
    m_u.y = kur_u.y * D  +  kur_v.x * (- m_nu * D);
    m_u.z = kur_u.z * m_beta * F;
    m_v.x = kur_v.x * D  +  kur_u.y * (- m_nu * D);
    m_v.x = kur_v.x * 2* F;
    m_v.z = kur_v.z * m_beta * F;
}



// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellEANS4::ChElementShellEANS4() :  m_numLayers(0), m_thickness(0) {
    m_nodes.resize(4);
    m_Alpha = 0;

    q_refrotA = QUNIT;
    q_refrotB = QUNIT;
    q_refrotC = QUNIT;
    q_refrotD = QUNIT;
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementShellEANS4::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeD) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);
    assert(nodeD);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;
    std::vector<ChLcpVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellEANS4::AddLayer(double thickness, double theta, std::shared_ptr<ChMaterialShellEANS> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellEANS4::SetupInitial(ChSystem* system) {
    // Perform layer initialization and accumulate element thickness.
    m_numLayers = m_layers.size();
    m_thickness = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        m_layers[kl].SetupInitial();
        m_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the range for Gauss integration in the
    // z direction (values in [-1,1]).
    m_GaussZ.push_back(-1);
    double z = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        z += m_layers[kl].Get_thickness();
        m_GaussZ.push_back(2 * z / m_thickness - 1);
    }

    // compute initial sizes
    m_lenX = (0.5*(GetNodeA()->coord.pos+GetNodeC()->coord.pos) - 0.5*(GetNodeB()->coord.pos+GetNodeD()->coord.pos) ).Length();
    m_lenY = (0.5*(GetNodeA()->coord.pos+GetNodeB()->coord.pos) - 0.5*(GetNodeC()->coord.pos+GetNodeD()->coord.pos) ).Length();

    // Compute mass matrix and gravitational forces (constant)
    ComputeMassMatrix();
}

// State update.
void ChElementShellEANS4::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector with the current field values at the element nodes.
void ChElementShellEANS4::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(4*7, 1);
    mD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
    mD.PasteQuaternion(m_nodes[0]->GetRot(), 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), 7, 0);
    mD.PasteQuaternion(m_nodes[1]->GetRot(), 10, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), 14, 0);
    mD.PasteQuaternion(m_nodes[2]->GetRot(), 17, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), 21, 0);
    mD.PasteQuaternion(m_nodes[3]->GetRot(), 24, 0);
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementShellEANS4::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 24) && (H.GetColumns() == 24));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 24; i++)
        for (int j = 0; j < 24; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellEANS4::ComputeMmatrixGlobal(ChMatrix<>& M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class MyMassEANS : public ChIntegrable2D<ChMatrixNM<double, 24, 24> > {
public:
  MyMassEANS(ChElementShellEANS4* element) : m_element(element) {}
  ~MyMassEANS() {}

private:
    ChElementShellEANS4* m_element;

    virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y) override;
};

void MyMassEANS::Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y) {
    ChMatrixNM<double, 1, 4> N;
    m_element->ShapeFunctions(N, x, y, 0);
    double N00 = N(0)*N(0);
    double N11 = N(1)*N(1);
    double N22 = N(2)*N(2);
    double N33 = N(3)*N(3);
    double rotfactor = 0.01; //***TODO*** this just applies a bit of mass to rotational DOFs - not needed if implicit integration btw.
    result(0,0)   = N00;
    result(1,1)   = N00;
    result(2,2)   = N00;
    result(3,3)   = N00*rotfactor;
    result(4,4)   = N00*rotfactor;
    result(5,5)   = N00*rotfactor;

    result(6,6)   = N11;
    result(7,7)   = N11;
    result(8,8)   = N11;
    result(9,9)   = N11*rotfactor;
    result(10,10) = N11*rotfactor;
    result(11,11) = N11*rotfactor;

    result(12,12) = N22;
    result(13,13) = N22;
    result(14,14) = N22;
    result(15,15) = N22*rotfactor;
    result(16,16) = N22*rotfactor;
    result(17,17) = N22*rotfactor;

    result(18,18) = N33;
    result(19,19) = N33;
    result(20,20) = N33;
    result(21,21) = N33*rotfactor;
    result(22,22) = N33*rotfactor;
    result(23,23) = N33*rotfactor;
};

void ChElementShellEANS4::ComputeMassMatrix() {
    m_MassMatrix.Reset();
    double jacobian = this->m_lenY * this->m_lenX / 4.0;

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double thickness = m_layers[kl].GetMaterial()->Get_thickness();
        MyMassEANS myformula(this);
        ChMatrixNM<double, 24, 24> TempMassMatrix;

        ChQuadrature::Integrate2D<ChMatrixNM<double, 24, 24> >(TempMassMatrix,  // result of integration will go there
                                                               myformula,       // formula to integrate
                                                               -1, 1,           // x limits
                                                               -1, 1,           // y limits
                                                               2                                // order of integration
                                                               );
        TempMassMatrix *= (rho * jacobian * thickness);
        m_MassMatrix += TempMassMatrix;
    }
}


// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------


class MyForceEANS : public ChIntegrable2D<ChMatrixNM<double, 24, 1> > {
  public:
    MyForceEANS(ChElementShellEANS4* element,             // Containing element
            size_t m_layer_i                              // Current layer index
            )
        : m_element(element),
          layer_i(m_layer_i) {}
    ~MyForceEANS() {}

    ChVector<> pA;
    ChVector<> pB;
    ChVector<> pC;
    ChVector<> pD;
    ChQuaternion<> rA;
    ChQuaternion<> rB;
    ChQuaternion<> rC;
    ChQuaternion<> rD;
    size_t layer_i;

  private:
    ChElementShellEANS4* m_element;

    /// Evaluate internal force
    virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y) override;
};

// eq. 101 from Felippa,Haugen: "A unified formulation of small-strain corotational
// finite elements"
void ComputeGammaMatrix(ChMatrix33<>& H, const ChVector<> phi) {
    
    H.Set33Identity();

    ChMatrix33<> Phi;
    Phi.Set_X_matrix(phi);
    H -= (Phi * 0.5);

    double ang = phi.Length();
    double eta;
    if (fabs(ang)>0.01) {
        eta = (1. - 0.5*ang*(1./tan(0.5*ang)))/(ang*ang);
    } 
    else {
        eta = (1./12.) + (1./720.)*ang*ang + (1./30240.0)*pow(ang,4);
    }
    H += Phi*Phi*eta;
}
// eq. 100 from Felippa,Haugen: "A unified formulation of small-strain corotational
// finite elements"
void ComputeGammaMatrixInverse(ChMatrix33<>& H, const ChVector<> phi) {
    H.Set33Identity();
    double ang = phi.Length();
    if (fabs(ang)>1e-3) {
        ChMatrix33<> Phi;
        Phi.Set_X_matrix(phi);
        H += (Phi * ((1.-cos(ang))/(ang*ang)));
        H += (Phi*Phi * ((ang-sin(ang))/(ang*ang*ang)));
    }
}

void MyForceEANS::Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y) {
    // Element shape function
    ChMatrixNM<double, 1, 4> N;
    m_element->ShapeFunctions(N, x, y, 0);
    ChMatrixNM<double, 1, 4> N_ANS;
    m_element->ShapeFunctionANSbilinearShell(N_ANS, x, y);

    ChMatrixNM<double, 1, 4> Nu;
    ChMatrixNM<double, 1, 4> Nv;
    m_element->ShapeFunctionsDerivativeX(Nu, x, y, 0);
    m_element->ShapeFunctionsDerivativeY(Nv, x, y, 0);



    const ChVector<>& pA0 = m_element->GetNodeA()->GetX0().GetPos();
    const ChVector<>& pB0 = m_element->GetNodeB()->GetX0().GetPos();
    const ChVector<>& pC0 = m_element->GetNodeC()->GetX0().GetPos();
    const ChVector<>& pD0 = m_element->GetNodeD()->GetX0().GetPos();
    ChVector<> yA = pA - pA0;
    ChVector<> yB = pB - pB0;
    ChVector<> yC = pC - pC0;
    ChVector<> yD = pD - pD0;

    // Tn = Rn*Rn0'
    ChQuaternion<> Ta = rA * m_element->GetNodeAreferenceRot().GetConjugate();
    ChQuaternion<> Tb = rB * m_element->GetNodeBreferenceRot().GetConjugate();
    ChQuaternion<> Tc = rC * m_element->GetNodeCreferenceRot().GetConjugate();
    ChQuaternion<> Td = rD * m_element->GetNodeDreferenceRot().GetConjugate();
    
    // Tavg = exp(log(1/4(Ta+Tb+Tc+Td)), also approx as:
//    ChQuaternion<> Tavg = (Ta+Tb+Tc+Td).GetNormalized();
    ChQuaternion<> Tavg =(m_element->GetNodeA()->GetRot() * m_element->GetNodeAreferenceRot().GetConjugate() +
                          m_element->GetNodeB()->GetRot() * m_element->GetNodeBreferenceRot().GetConjugate() +
                          m_element->GetNodeC()->GetRot() * m_element->GetNodeCreferenceRot().GetConjugate() +
                          m_element->GetNodeD()->GetRot() * m_element->GetNodeDreferenceRot().GetConjugate() ).GetNormalized();
    // Tavg' 
    ChQuaternion<> TavgT = Tavg.GetConjugate();

    // R_rel_n = Tavg'* T_n
    // F_rel_n = log(R_rel_n)    i.e. to rot vector
    ChVector<> F_relA = (TavgT * Ta).Q_to_Rotv();
    ChVector<> F_relB = (TavgT * Tb).Q_to_Rotv();
    ChVector<> F_relC = (TavgT * Tc).Q_to_Rotv();
    ChVector<> F_relD = (TavgT * Td).Q_to_Rotv();
    
    // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
    ChVector<> F_rel_i = N(0)*F_relA + 
                         N(1)*F_relB + 
                         N(2)*F_relC +
                         N(3)*F_relD;
    
    // T_i = Tavg*exp(phi_i)
    ChQuaternion<> expPhi; 
    expPhi.Q_from_Rotv(F_rel_i);
    ChQuaternion<> T_i = Tavg * expPhi;

    // STRAIN  in local frame
    // eps_u = T_i'* Yi,u = T_i' * sum_n(Nin,u Yn)
    // eps_v = T_i'* Yi,v = T_i' * sum_n(Nin,v Yn)
    ChVector<> yi_u =  Nu(0) * yA +   Nu(1) * yB +  Nu(2) * yC +  Nu(3) * yD ;
    ChVector<> yi_v =  Nv(0) * yA +   Nv(1) * yB +  Nv(2) * yC +  Nv(3) * yD ;
    ChVector<> eps_u = T_i.RotateBack ( yi_u );
    ChVector<> eps_v = T_i.RotateBack ( yi_v );


    // CURVATURES in local frame
    // kur_u = T_i'* abs_kur,u = Hi' * sum( Nin,u F_rel_n )
    // kur_v = T_i'* abs_kur,v = Hi' * sum( Nin,u F_rel_n )
    ChVector<> F_rel_u =  Nu(0) * F_relA +   Nu(1) * F_relB +  Nu(2) * F_relC +  Nu(3) * F_relD ;
    ChVector<> F_rel_v =  Nv(0) * F_relA +   Nv(1) * F_relB +  Nv(2) * F_relC +  Nv(3) * F_relD ;
    ChMatrix33<> Hi;
    ComputeGammaMatrix(Hi,F_rel_i);
 //Hi.Set33Identity();
    ChVector<> kur_u = Hi.MatrT_x_Vect( F_rel_u );
    ChVector<> kur_v = Hi.MatrT_x_Vect( F_rel_v );


    // some complication: compute the Phi matrices:
    ChMatrix33<> Hai;
    ComputeGammaMatrixInverse(Hai,F_relA);
    ChMatrix33<> Hbi;
    ComputeGammaMatrixInverse(Hbi,F_relB);
    ChMatrix33<> Hci;
    ComputeGammaMatrixInverse(Hci,F_relC);
    ChMatrix33<> Hdi;
    ComputeGammaMatrixInverse(Hdi,F_relD);
// Hai.Set33Identity();
// Hbi.Set33Identity();
// Hci.Set33Identity();
// Hdi.Set33Identity();

    ChMatrix33<> mTavgT(TavgT);
    ChMatrix33<> mTavg(Tavg);
    ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * m_element->GetNodeA()->GetA(); 
    ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * m_element->GetNodeB()->GetA();
    ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * m_element->GetNodeC()->GetA();
    ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * m_element->GetNodeD()->GetA();
    // note: respect to Masarati paper, added the ....* m_element->GetNodeA()->GetA() part because 
    // incremental rotations in C::E are considered in body coords, not in abs.coords 
//PhiA.Set33Identity();
//PhiB.Set33Identity();
//PhiC.Set33Identity();
//PhiD.Set33Identity();

    // Build the B matrix:
    ChMatrixNM<double, 12,24> B;
    
    ChMatrix33<> mT_i_t(T_i.GetConjugate());
    ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
    ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);
    ChMatrix33<> mk_u_X; mk_u_X.Set_X_matrix(kur_u);
    ChMatrix33<> mk_v_X; mk_v_X.Set_X_matrix(kur_v);

    ChMatrix33<> block;
    block = mT_i_t * Nu(0);     B.PasteMatrix(&block, 0,0);
    block = mT_i_t * Nv(0);     B.PasteMatrix(&block, 3,0);
    block = mT_i_t * Nu(1);     B.PasteMatrix(&block, 0,6);
    block = mT_i_t * Nv(1);     B.PasteMatrix(&block, 3,6);
    block = mT_i_t * Nu(2);     B.PasteMatrix(&block, 0,12);
    block = mT_i_t * Nv(2);     B.PasteMatrix(&block, 3,12);
    block = mT_i_t * Nu(3);     B.PasteMatrix(&block, 0,18);
    block = mT_i_t * Nv(3);     B.PasteMatrix(&block, 3,18);
        
    block = mT_i_t * myi_u_X * PhiA * N(0);     B.PasteMatrix(&block, 0,3);
    block = mT_i_t * myi_v_X * PhiA * N(0);     B.PasteMatrix(&block, 3,3);
    block = mT_i_t * myi_u_X * PhiB * N(1);     B.PasteMatrix(&block, 0,9);
    block = mT_i_t * myi_v_X * PhiB * N(1);     B.PasteMatrix(&block, 3,9);
    block = mT_i_t * myi_u_X * PhiC * N(2);     B.PasteMatrix(&block, 0,15);
    block = mT_i_t * myi_v_X * PhiC * N(2);     B.PasteMatrix(&block, 3,15);
    block = mT_i_t * myi_u_X * PhiD * N(3);     B.PasteMatrix(&block, 0,21);
    block = mT_i_t * myi_v_X * PhiD * N(3);     B.PasteMatrix(&block, 3,21);
    
    ChMatrix33<> mKu = PhiA*Nu(0); // .. + Elle() term, to be added?
    ChMatrix33<> mKv = PhiA*Nv(0); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiA * N(0) + mKu);      B.PasteMatrix(&block, 6,3);
    block = mT_i_t * (mk_v_X * PhiA * N(0) + mKv);      B.PasteMatrix(&block, 9,3);
    mKu = PhiB*Nu(1); // .. + Elle() term, to be added?
    mKv = PhiB*Nv(1); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiB * N(1) + mKu);      B.PasteMatrix(&block, 6,9);
    block = mT_i_t * (mk_v_X * PhiB * N(1) + mKv);      B.PasteMatrix(&block, 9,9);
    mKu = PhiC*Nu(2); // .. + Elle() term, to be added?
    mKv = PhiC*Nv(2); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiC * N(2) + mKu);      B.PasteMatrix(&block, 6,15);
    block = mT_i_t * (mk_v_X * PhiC * N(2) + mKv);      B.PasteMatrix(&block, 9,15);
    mKu = PhiD*Nu(3); // .. + Elle() term, to be added?
    mKv = PhiD*Nv(3); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiD * N(3) + mKu);      B.PasteMatrix(&block, 6,21);
    block = mT_i_t * (mk_v_X * PhiD * N(3) + mKv);      B.PasteMatrix(&block, 9,21);

    // ANS CORRECTION:

    // replace transversal shear with ANS interpolated shear from tying points:
    eps_u.z = N_ANS(2)*this->m_element->m_strainANS(2,2)  // (0,-1)
            + N_ANS(3)*this->m_element->m_strainANS(2,3); // (0,+1)
    eps_v.z = N_ANS(0)*this->m_element->m_strainANS(5,0)  // (-1,0)
            + N_ANS(1)*this->m_element->m_strainANS(5,1); // (+1,0)
    for (int ic=0; ic<24; ++ic)
        B(2,ic) = N_ANS(2)* this->m_element->m_B3_ANS(2,ic) + 
                  N_ANS(3)* this->m_element->m_B3_ANS(3,ic);
    for (int ic=0; ic<24; ++ic)
        B(5,ic) = N_ANS(0)* this->m_element->m_B6_ANS(0,ic) + 
                  N_ANS(1)* this->m_element->m_B6_ANS(1,ic);
                  
    //GetLog() << eps_u << eps_v << "\n .....";

    // STRESSES - forces n and torques m 
    ChVector<> n_u;
    ChVector<> n_v;
    ChVector<> m_u;
    ChVector<> m_v;
    m_element->GetLayer(layer_i).GetMaterial()->ComputeStress(n_u, n_v, m_u, m_v, eps_u, eps_v, kur_u, kur_v);

    // CONVERT n AND m TO GENERALIZED FORCES 'result':
    ChMatrixNM<double,12,1> sigma;
    sigma.PasteVector(n_u, 0,0);
    sigma.PasteVector(n_v, 3,0);
    sigma.PasteVector(m_u, 6,0);
    sigma.PasteVector(m_v, 9,0);
     //GetLog() << "sigma:" << sigma << "\n ..."; 
    // F = int{B*sigma*dv} , so at each Gauss integration point:  
    //   F_i = B_i * sigma_i  
    result.MatrTMultiply(B,sigma);
    //GetLog() << "Fi:" << result << "\n ...";
    result.MatrScale(-1);
}


void ChElementShellEANS4::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    this->ComputeInternalForces_Impl(
                GetNodeA()->GetPos(), GetNodeA()->GetRot(),
                GetNodeB()->GetPos(), GetNodeB()->GetRot(),
                GetNodeC()->GetPos(), GetNodeC()->GetRot(),
                GetNodeD()->GetPos(), GetNodeD()->GetRot(), 
                Fi);
}

void ChElementShellEANS4::ComputeInternalForces_Impl(const ChVector<>& pA, const ChQuaternion<>& rA,
                                    const ChVector<>& pB, const ChQuaternion<>& rB,
                                    const ChVector<>& pC, const ChQuaternion<>& rC,
                                    const ChVector<>& pD, const ChQuaternion<>& rD,
                                    ChMatrixDynamic<>& Fi) {

    double jacobian = this->m_lenY * this->m_lenX  / 4.0;

    Fi.Reset();

    // Assumed Natural Strain (ANS):  precompute m_strainANS 
    CalcStrainANSbilinearShell(pA,rA, pB,rB, pC,rC, pD,rD);

    MyForceEANS myformula(this,0);
    myformula.pA = pA;
    myformula.pB = pB;
    myformula.pC = pC;
    myformula.pD = pD;
    myformula.rA = rA;
    myformula.rB = rB;
    myformula.rC = rC;
    myformula.rD = rD;

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        myformula.layer_i = kl;
        ChMatrixNM<double, 24, 1> TempForce;

        ChQuadrature::Integrate2D<ChMatrixNM<double, 24, 1> >(TempForce,  // result of integration will go there
                                                               myformula,       // formula to integrate
                                                               -1, 1,           // x limits
                                                               -1, 1,           // y limits
                                                               2                                // order of integration
                                                               );
        TempForce *= jacobian;
        Fi += TempForce;

    }  // Layer Loop
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

// The class MyJacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer.
class MyJacobianEANS : public ChIntegrable2D<ChMatrixNM<double, 24, 24> > {
  public:
    MyJacobianEANS(ChElementShellEANS4* element,  // Containing element
               size_t kl                     // Current layer index
               )
        : m_element(element), m_Kfactor(1), m_Rfactor(0), layer_i(kl) {}
  public:
    ChVector<> pA;
    ChVector<> pB;
    ChVector<> pC;
    ChVector<> pD;
    ChQuaternion<> rA;
    ChQuaternion<> rB;
    ChQuaternion<> rC;
    ChQuaternion<> rD;
    double m_Kfactor;
    double m_Rfactor;
    size_t layer_i;
  private:
    ChElementShellEANS4* m_element; 

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y) override;
};

void MyJacobianEANS::Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y) {
        // Element shape function
    ChMatrixNM<double, 1, 4> N;
    m_element->ShapeFunctions(N, x, y, 0);
    ChMatrixNM<double, 1, 4> N_ANS;
    m_element->ShapeFunctionANSbilinearShell(N_ANS, x, y);

    ChMatrixNM<double, 1, 4> Nu;
    ChMatrixNM<double, 1, 4> Nv;
    m_element->ShapeFunctionsDerivativeX(Nu, x, y, 0);
    m_element->ShapeFunctionsDerivativeY(Nv, x, y, 0);

    const ChVector<>& pA0 = m_element->GetNodeA()->GetX0().GetPos();
    const ChVector<>& pB0 = m_element->GetNodeB()->GetX0().GetPos();
    const ChVector<>& pC0 = m_element->GetNodeC()->GetX0().GetPos();
    const ChVector<>& pD0 = m_element->GetNodeD()->GetX0().GetPos();
    ChVector<> yA = pA - pA0;
    ChVector<> yB = pB - pB0;
    ChVector<> yC = pC - pC0;
    ChVector<> yD = pD - pD0;

    // Tn = Rn*Rn0'
    ChQuaternion<> Ta = rA * m_element->GetNodeAreferenceRot().GetConjugate();
    ChQuaternion<> Tb = rB * m_element->GetNodeBreferenceRot().GetConjugate();
    ChQuaternion<> Tc = rC * m_element->GetNodeCreferenceRot().GetConjugate();
    ChQuaternion<> Td = rD * m_element->GetNodeDreferenceRot().GetConjugate();
    
    // Tavg = exp(log(1/4(Ta+Tb+Tc+Td)), also approx as:
//    ChQuaternion<> Tavg = (Ta+Tb+Tc+Td).GetNormalized();
    ChQuaternion<> Tavg =(m_element->GetNodeA()->GetRot() * m_element->GetNodeAreferenceRot().GetConjugate() +
                          m_element->GetNodeB()->GetRot() * m_element->GetNodeBreferenceRot().GetConjugate() +
                          m_element->GetNodeC()->GetRot() * m_element->GetNodeCreferenceRot().GetConjugate() +
                          m_element->GetNodeD()->GetRot() * m_element->GetNodeDreferenceRot().GetConjugate() ).GetNormalized();
    // Tavg' 
    ChQuaternion<> TavgT = Tavg.GetConjugate();

    // R_rel_n = Tavg'* T_n
    // F_rel_n = log(R_rel_n)    i.e. to rot vector
    ChVector<> F_relA = (TavgT * Ta).Q_to_Rotv();
    ChVector<> F_relB = (TavgT * Tb).Q_to_Rotv();
    ChVector<> F_relC = (TavgT * Tc).Q_to_Rotv();
    ChVector<> F_relD = (TavgT * Td).Q_to_Rotv();
    
    // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
    ChVector<> F_rel_i = N(0)*F_relA + 
                         N(1)*F_relB + 
                         N(2)*F_relC +
                         N(3)*F_relD;
    
    // T_i = Tavg*exp(phi_i)
    ChQuaternion<> expPhi; 
    expPhi.Q_from_Rotv(F_rel_i);
    ChQuaternion<> T_i = Tavg * expPhi;

    // STRAIN  in local frame
    // eps_u = T_i'* Yi,u = T_i' * sum_n(Nin,u Yn)
    // eps_v = T_i'* Yi,v = T_i' * sum_n(Nin,v Yn)
    ChVector<> yi_u =  Nu(0) * yA +   Nu(1) * yB +  Nu(2) * yC +  Nu(3) * yD ;
    ChVector<> yi_v =  Nv(0) * yA +   Nv(1) * yB +  Nv(2) * yC +  Nv(3) * yD ;


    // CURVATURES in local frame
    // kur_u = T_i'* abs_kur,u = Hi' * sum( Nin,u F_rel_n )
    // kur_v = T_i'* abs_kur,v = Hi' * sum( Nin,u F_rel_n )
    ChVector<> F_rel_u =  Nu(0) * F_relA +   Nu(1) * F_relB +  Nu(2) * F_relC +  Nu(3) * F_relD ;
    ChVector<> F_rel_v =  Nv(0) * F_relA +   Nv(1) * F_relB +  Nv(2) * F_relC +  Nv(3) * F_relD ;
    ChMatrix33<> Hi;
    ComputeGammaMatrix(Hi,F_rel_i);

    ChVector<> kur_u = Hi.MatrT_x_Vect( F_rel_u );
    ChVector<> kur_v = Hi.MatrT_x_Vect( F_rel_v );

    // some complication: compute the Phi matrices:
    ChMatrix33<> Hai;
    ComputeGammaMatrixInverse(Hai,F_relA);
    ChMatrix33<> Hbi;
    ComputeGammaMatrixInverse(Hbi,F_relB);
    ChMatrix33<> Hci;
    ComputeGammaMatrixInverse(Hci,F_relC);
    ChMatrix33<> Hdi;
    ComputeGammaMatrixInverse(Hdi,F_relD);

    ChMatrix33<> mTavgT(TavgT);
    ChMatrix33<> mTavg(Tavg);
    ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * m_element->GetNodeA()->GetA(); 
    ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * m_element->GetNodeB()->GetA();
    ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * m_element->GetNodeC()->GetA();
    ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * m_element->GetNodeD()->GetA();
    // note: respect to Masarati paper, added the ....* m_element->GetNodeA()->GetA() part because 
    // incremental rotations in C::E are considered in body coords, not in abs.coords 

    // Build the B matrix:
    ChMatrixNM<double, 12,24> B;
    
    ChMatrix33<> mT_i_t(T_i.GetConjugate());
    ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
    ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);
    ChMatrix33<> mk_u_X; mk_u_X.Set_X_matrix(kur_u);
    ChMatrix33<> mk_v_X; mk_v_X.Set_X_matrix(kur_v);

    ChMatrix33<> block;
    block = mT_i_t * Nu(0);     B.PasteMatrix(&block, 0,0);
    block = mT_i_t * Nv(0);     B.PasteMatrix(&block, 3,0);
    block = mT_i_t * Nu(1);     B.PasteMatrix(&block, 0,6);
    block = mT_i_t * Nv(1);     B.PasteMatrix(&block, 3,6);
    block = mT_i_t * Nu(2);     B.PasteMatrix(&block, 0,12);
    block = mT_i_t * Nv(2);     B.PasteMatrix(&block, 3,12);
    block = mT_i_t * Nu(3);     B.PasteMatrix(&block, 0,18);
    block = mT_i_t * Nv(3);     B.PasteMatrix(&block, 3,18);
        
    block = mT_i_t * myi_u_X * PhiA * N(0);     B.PasteMatrix(&block, 0,3);
    block = mT_i_t * myi_v_X * PhiA * N(0);     B.PasteMatrix(&block, 3,3);
    block = mT_i_t * myi_u_X * PhiB * N(1);     B.PasteMatrix(&block, 0,9);
    block = mT_i_t * myi_v_X * PhiB * N(1);     B.PasteMatrix(&block, 3,9);
    block = mT_i_t * myi_u_X * PhiC * N(2);     B.PasteMatrix(&block, 0,15);
    block = mT_i_t * myi_v_X * PhiC * N(2);     B.PasteMatrix(&block, 3,15);
    block = mT_i_t * myi_u_X * PhiD * N(3);     B.PasteMatrix(&block, 0,21);
    block = mT_i_t * myi_v_X * PhiD * N(3);     B.PasteMatrix(&block, 3,21);

    ChMatrix33<> mKu = PhiA*Nu(0); // .. + Elle() term, to be added?
    ChMatrix33<> mKv = PhiA*Nv(0); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiA * N(0) + mKu);      B.PasteMatrix(&block, 6,3);
    block = mT_i_t * (mk_v_X * PhiA * N(0) + mKv);      B.PasteMatrix(&block, 9,3);
    mKu = PhiB*Nu(1); // .. + Elle() term, to be added?
    mKv = PhiB*Nv(1); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiB * N(1) + mKu);      B.PasteMatrix(&block, 6,9);
    block = mT_i_t * (mk_v_X * PhiB * N(1) + mKv);      B.PasteMatrix(&block, 9,9);
    mKu = PhiC*Nu(2); // .. + Elle() term, to be added?
    mKv = PhiC*Nv(2); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiC * N(2) + mKu);      B.PasteMatrix(&block, 6,15);
    block = mT_i_t * (mk_v_X * PhiC * N(2) + mKv);      B.PasteMatrix(&block, 9,15);
    mKu = PhiD*Nu(3); // .. + Elle() term, to be added?
    mKv = PhiD*Nv(3); // .. + Elle() term, to be added?
    block = mT_i_t * (mk_u_X * PhiD * N(3) + mKu);      B.PasteMatrix(&block, 6,21);
    block = mT_i_t * (mk_v_X * PhiD * N(3) + mKv);      B.PasteMatrix(&block, 9,21);

    // ANS CORRECTION:

    // replace transversal shear with ANS interpolated shear from tying points:
    for (int ic=0; ic<24; ++ic)
        B(2,ic) = N_ANS(2)* this->m_element->m_B3_ANS(2,ic) + 
                  N_ANS(3)* this->m_element->m_B3_ANS(3,ic);
    for (int ic=0; ic<24; ++ic)
        B(5,ic) = N_ANS(0)* this->m_element->m_B6_ANS(0,ic) + 
                  N_ANS(1)* this->m_element->m_B6_ANS(1,ic);

    // COMPUTE K_m

    ChMatrixNM<double,12,1> CBi;
    ChMatrixNM<double,24,1> Kmi;
    ChVector<> n_u;
    ChVector<> n_v;
    ChVector<> m_u;
    ChVector<> m_v;
    ChVector<> beps_u;
    ChVector<> beps_v;
    ChVector<> bkur_u;
    ChVector<> bkur_v;
    for (int ic=0; ic<24; ++ic) {
        beps_u = B.ClipVector(0,ic);
        beps_v = B.ClipVector(3,ic);
        bkur_u = B.ClipVector(6,ic);
        bkur_v = B.ClipVector(9,ic);
        m_element->GetLayer(layer_i).GetMaterial()->ComputeStress(n_u, n_v, m_u, m_v, beps_u, beps_v, bkur_u, bkur_v);
        CBi.PasteVector(n_u, 0,0);
        CBi.PasteVector(n_v, 3,0);
        CBi.PasteVector(m_u, 6,0);
        CBi.PasteVector(m_v, 9,0);
        
        Kmi.MatrTMultiply(B,CBi);
        result.PasteMatrix(&Kmi,0,ic);
    }
    //result.MatrScale(-1);
}

void ChElementShellEANS4::ComputeInternalJacobians(double Kfactor, double Rfactor) {

    m_JacobianMatrix.Reset();

    bool use_numerical_differentiation = true;
    
    if (use_numerical_differentiation) {

        double diff = 1e-4;
        ChMatrixNM<double,24,1> Kcolumn;
        ChMatrixDynamic<> F0(24, 1);
        ChMatrixDynamic<> F1(24, 1);

        this->ComputeInternalForces(F0);

        // Create local copies of the nodal coordinates and use the implementation version
        // of the function for calculating the internal forces.  With this, the calculation
        // of the Jacobian with finite differences is thread safe (otherwise, there would
        // be race conditions when adjacent elements attempt to perturb a common node).
        ChVector<> pos[4] ={GetNodeA()->GetPos(), 
                            GetNodeB()->GetPos(),
                            GetNodeC()->GetPos(),
                            GetNodeD()->GetPos()};
        ChQuaternion<> rot[4] ={GetNodeA()->GetRot(), 
                                GetNodeB()->GetRot(), 
                                GetNodeC()->GetRot(), 
                                GetNodeD()->GetRot()}; 

        double scaler = (1.0 / diff) * (Kfactor + Rfactor * this->m_Alpha);

        for (int inode = 0; inode < 4; ++inode) {
            pos[inode].x += diff;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 0 + inode * 6);
            pos[inode].x -= diff;

            pos[inode].y += diff;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 1 + inode * 6);
            pos[inode].y -= diff;
            
            pos[inode].z += diff;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 2 + inode * 6);
            pos[inode].z -= diff;

            ChQuaternion<> qdrot;
            ChQuaternion<> qbackup;
            qbackup = rot[inode];

            qdrot.Q_from_Rotv( {diff,0,0} ); // incremental rotation on x axis (in frame basis, as C::E rot.increments are frame-local)
            rot[inode] = rot[inode] * qdrot;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 3 + inode * 6);
            rot[inode] = qbackup;

            qdrot.Q_from_Rotv( {0,diff,0} ); // incremental rotation on y axis (in frame basis, as C::E rot.increments are frame-local)
            rot[inode] = rot[inode] * qdrot;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 4 + inode * 6);
            rot[inode] = qbackup;

            qdrot.Q_from_Rotv( {0,0,diff} ); // incremental rotation on z axis (in frame basis, as C::E rot.increments are frame-local)
            rot[inode] = rot[inode] * qdrot;
            this->ComputeInternalForces_Impl(pos[0], rot[0], pos[1], rot[1],  pos[2], rot[2], pos[3], rot[3], F1);
            Kcolumn = (F0 - F1) * scaler;
            this->m_JacobianMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 5 + inode * 6);
            rot[inode] = qbackup;
        }
    }
    else {
        double jacobian = this->m_lenY * this->m_lenX  / 4.0;
               // Assumed Natural Strain (ANS):  precompute m_strainANS 
        CalcStrainANSbilinearShell( GetNodeA()->GetPos(),GetNodeA()->GetRot(),
                                    GetNodeB()->GetPos(),GetNodeB()->GetRot(),
                                    GetNodeC()->GetPos(),GetNodeC()->GetRot(),
                                    GetNodeD()->GetPos(),GetNodeD()->GetRot());

        MyJacobianEANS myformula(this,0);
        myformula.pA = GetNodeA()->GetPos();
        myformula.pB = GetNodeB()->GetPos();
        myformula.pC = GetNodeC()->GetPos();
        myformula.pD = GetNodeD()->GetPos();
        myformula.rA = GetNodeA()->GetRot();
        myformula.rB = GetNodeB()->GetRot();
        myformula.rC = GetNodeC()->GetRot();
        myformula.rD = GetNodeD()->GetRot();

        for (size_t kl = 0; kl < m_numLayers; kl++) {
            myformula.layer_i = kl;
            ChMatrixNM<double, 24, 24> TempJacobian;

            ChQuadrature::Integrate2D<ChMatrixNM<double, 24, 24> >(TempJacobian,  // result of integration will go there
                                                                   myformula,       // formula to integrate
                                                                   -1, 1,           // x limits
                                                                   -1, 1,           // y limits
                                                                   2                                // order of integration
                                                                   );
            TempJacobian *= jacobian * (Kfactor + Rfactor * this->m_Alpha);
            this->m_JacobianMatrix += TempJacobian;
        }  // Layer Loop

    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellEANS4::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    N(0) = 0.25 * (1.0 - x) * (1.0 - y);
    N(1) = 0.25 * (1.0 + x) * (1.0 - y);
    N(2) = 0.25 * (1.0 - x) * (1.0 + y);
    N(3) = 0.25 * (1.0 + x) * (1.0 + y);
}

void ChElementShellEANS4::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetLengthX();

    Nx(0) = 0.25 * (-2.0 / a) * (1.0 - y);
    Nx(1) = 0.25 * (2.0 / a) * (1.0 - y);
    Nx(2) = 0.25 * (-2.0 / a) * (1.0 + y);
    Nx(3) = 0.25 * (2.0 / a) * (1.0 + y);
}

void ChElementShellEANS4::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double b = GetLengthY();

    Ny(0) = 0.25 * (1.0 - x) * (-2.0 / b);
    Ny(1) = 0.25 * (1.0 + x) * (-2.0 / b);
    Ny(2) = 0.25 * (1.0 - x) * (2.0 / b);
    Ny(3) = 0.25 * (1.0 + x) * (2.0 / b);
}



// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Assumed Natural Strain
// -----------------------------------------------------------------------------


// ANS shape function (interpolation of strains from tying points)
void ChElementShellEANS4::ShapeFunctionANSbilinearShell(ChMatrix<>& S_ANS, double x, double y) {
    S_ANS(0, 0) = -0.5 * x + 0.5;
    S_ANS(0, 1) = 0.5 * x + 0.5;
    S_ANS(0, 2) = -0.5 * y + 0.5;
    S_ANS(0, 3) = 0.5 * y + 0.5;
}

// Calculate ANS strain and its Jacobian
void ChElementShellEANS4::CalcStrainANSbilinearShell(const ChVector<>& pA, const ChQuaternion<>& rA,
                                    const ChVector<>& pB, const ChQuaternion<>& rB,
                                    const ChVector<>& pC, const ChQuaternion<>& rC,
                                    const ChVector<>& pD, const ChQuaternion<>& rD) {
    /// nodes a b c d, first two along  u axis, second two along v axis
    std::vector<double> knots_u = {-1,  1,  0 , 0};
    std::vector<double> knots_v = { 0,  0, -1 , 1};

    ChMatrixNM<double, 1, 4> N;
    ChMatrixNM<double, 1, 4> Nu;
    ChMatrixNM<double, 1, 4> Nv;

    for (int kk = 0; kk < 4; kk++) {
        ShapeFunctions(N, knots_u[kk], knots_v[kk], 0);
        ShapeFunctionsDerivativeX(Nu, knots_u[kk], knots_v[kk], 0);
        ShapeFunctionsDerivativeY(Nv, knots_u[kk], knots_v[kk], 0);

        const ChVector<>& pA0 = GetNodeA()->GetX0().GetPos();
        const ChVector<>& pB0 = GetNodeB()->GetX0().GetPos();
        const ChVector<>& pC0 = GetNodeC()->GetX0().GetPos();
        const ChVector<>& pD0 = GetNodeD()->GetX0().GetPos();
        ChVector<> yA = pA - pA0;
        ChVector<> yB = pB - pB0;
        ChVector<> yC = pC - pC0;
        ChVector<> yD = pD - pD0;

        // Tn = Rn*Rn0'
        ChQuaternion<> Ta = rA * GetNodeAreferenceRot();
        ChQuaternion<> Tb = rB * GetNodeBreferenceRot();
        ChQuaternion<> Tc = rC * GetNodeCreferenceRot();
        ChQuaternion<> Td = rD * GetNodeDreferenceRot();
    
        // Tavg = exp(log(1/4(Ta+Tb+Tc+Td)), also approx as:
    // ChQuaternion<> Tavg = (Ta+Tb+Tc+Td).GetNormalized();
    ChQuaternion<> Tavg =(this->GetNodeA()->GetRot() * this->GetNodeAreferenceRot().GetConjugate() +
                          this->GetNodeB()->GetRot() * this->GetNodeBreferenceRot().GetConjugate() +
                          this->GetNodeC()->GetRot() * this->GetNodeCreferenceRot().GetConjugate() +
                          this->GetNodeD()->GetRot() * this->GetNodeDreferenceRot().GetConjugate() ).GetNormalized();

        // Tavg' 
        ChQuaternion<> TavgT = Tavg.GetConjugate();

        // R_rel_n = Tavg'* T_n
        // F_rel_n = log(R_rel_n)    i.e. to rot vector
        ChVector<> F_relA = (TavgT * Ta).Q_to_Rotv();
        ChVector<> F_relB = (TavgT * Tb).Q_to_Rotv();
        ChVector<> F_relC = (TavgT * Tc).Q_to_Rotv();
        ChVector<> F_relD = (TavgT * Td).Q_to_Rotv();
    
        // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
        ChVector<> F_rel_i = N(0)*F_relA + 
                             N(1)*F_relB + 
                             N(2)*F_relC +
                             N(3)*F_relD;
    
        // T_i = Tavg*exp(phi_i)
        ChQuaternion<> expPhi; 
        expPhi.Q_from_Rotv(F_rel_i);
        ChQuaternion<> T_i = Tavg * expPhi;

        // STRAIN  in local frame
        // eps_u = T_i'* Yi,u = T_i' * sum_n(Nin,u Yn)
        // eps_v = T_i'* Yi,v = T_i' * sum_n(Nin,v Yn)
        ChVector<> yi_u =  Nu(0) * yA +   Nu(1) * yB +  Nu(2) * yC +  Nu(3) * yD ;
        ChVector<> yi_v =  Nv(0) * yA +   Nv(1) * yB +  Nv(2) * yC +  Nv(3) * yD ;
        ChVector<> eps_u = T_i.RotateBack ( yi_u );
        ChVector<> eps_v = T_i.RotateBack ( yi_v );

        this->m_strainANS.PasteVector(eps_u,0,kk);
        this->m_strainANS.PasteVector(eps_v,3,kk);

        ChMatrix33<> Hi;
        ComputeGammaMatrix(Hi,F_rel_i);
        Hi.Set33Identity();

        // some complication: compute the Phi matrices:
        ChMatrix33<> Hai;
        ComputeGammaMatrixInverse(Hai,F_relA);
        ChMatrix33<> Hbi;
        ComputeGammaMatrixInverse(Hbi,F_relB);
        ChMatrix33<> Hci;
        ComputeGammaMatrixInverse(Hci,F_relC);
        ChMatrix33<> Hdi;
        ComputeGammaMatrixInverse(Hdi,F_relD);

        ChMatrix33<> mTavgT(TavgT);
        ChMatrix33<> mTavg(Tavg);
        ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * this->GetNodeA()->GetA(); 
        ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * this->GetNodeB()->GetA();
        ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * this->GetNodeC()->GetA();
        ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * this->GetNodeD()->GetA();

        ChMatrix33<> mT_i_t(T_i.GetConjugate());
        ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
        ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);

        ChMatrixDynamic<> B_ans(6,24);
        ChMatrix33<> block;
        block = mT_i_t * Nu(0);     B_ans.PasteMatrix(&block, 0,0);
        block = mT_i_t * Nv(0);     B_ans.PasteMatrix(&block, 3,0);
        block = mT_i_t * Nu(1);     B_ans.PasteMatrix(&block, 0,6);
        block = mT_i_t * Nv(1);     B_ans.PasteMatrix(&block, 3,6);
        block = mT_i_t * Nu(2);     B_ans.PasteMatrix(&block, 0,12);
        block = mT_i_t * Nv(2);     B_ans.PasteMatrix(&block, 3,12);
        block = mT_i_t * Nu(3);     B_ans.PasteMatrix(&block, 0,18);
        block = mT_i_t * Nv(3);     B_ans.PasteMatrix(&block, 3,18);
        
        block = mT_i_t * myi_u_X * PhiA * N(0);     B_ans.PasteMatrix(&block, 0,3);
        block = mT_i_t * myi_v_X * PhiA * N(0);     B_ans.PasteMatrix(&block, 3,3);
        block = mT_i_t * myi_u_X * PhiB * N(1);     B_ans.PasteMatrix(&block, 0,9);
        block = mT_i_t * myi_v_X * PhiB * N(1);     B_ans.PasteMatrix(&block, 3,9);
        block = mT_i_t * myi_u_X * PhiC * N(2);     B_ans.PasteMatrix(&block, 0,15);
        block = mT_i_t * myi_v_X * PhiC * N(2);     B_ans.PasteMatrix(&block, 3,15);
        block = mT_i_t * myi_u_X * PhiD * N(3);     B_ans.PasteMatrix(&block, 0,21);
        block = mT_i_t * myi_v_X * PhiD * N(3);     B_ans.PasteMatrix(&block, 3,21);

        this->m_B3_ANS.PasteClippedMatrix(&B_ans, 2,0, 1,24, kk,0);
        this->m_B6_ANS.PasteClippedMatrix(&B_ans, 5,0, 1,24, kk,0);
    }
}


// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------

void ChElementShellEANS4::EvaluateSectionDisplacement(const double u,
                                                     const double v,
                                                     const ChMatrix<>& displ,
                                                     ChVector<>& u_displ,
                                                     ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, displ, u_displ);
    u_rotaz = VNULL;  // no angles.. this is ANCF (or maybe return here the slope derivatives?)
}

void ChElementShellEANS4::EvaluateSectionFrame(const double u,
                                              const double v,
                                              const ChMatrix<>& displ,
                                              ChVector<>& point,
                                              ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, displ, point);
    rot = QUNIT;  // or maybe use gram-schmidt to get csys of section from slopes?
}

void ChElementShellEANS4::EvaluateSectionPoint(const double u,
                                              const double v,
                                              const ChMatrix<>& displ,
                                              ChVector<>& point) {
    ChMatrixNM<double, 1, 4> N;

    double x = u;  // because ShapeFunctions() works in -1..1 range
    double y = v;  // because ShapeFunctions() works in -1..1 range
    double z = 0;

    this->ShapeFunctions(N, x, y, z);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();

    point.x = N(0) * pA.x + N(1) * pB.x + N(2) * pC.x + N(3) * pD.x;
    point.y = N(0) * pA.y + N(1) * pB.y + N(2) * pC.y + N(3) * pD.y;
    point.z = N(0) * pA.z + N(1) * pB.z + N(2) * pC.z + N(3) * pD.z;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).
void ChElementShellEANS4::LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
    mD.PasteVector(m_nodes[0]->GetPos(), block_offset, 0);
    mD.PasteQuaternion(m_nodes[0]->GetRot(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), block_offset + 7, 0);
    mD.PasteQuaternion(m_nodes[1]->GetRot(), block_offset + 10, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), block_offset + 14, 0);
    mD.PasteQuaternion(m_nodes[2]->GetRot(), block_offset + 17, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), block_offset + 21, 0);
    mD.PasteQuaternion(m_nodes[3]->GetRot(), block_offset + 24, 0);
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellEANS4::LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
    mD.PasteVector(m_nodes[0]->GetPos_dt(), block_offset, 0);
    mD.PasteQuaternion(m_nodes[0]->GetRot_dt(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos_dt(), block_offset + 6, 0);
    mD.PasteQuaternion(m_nodes[1]->GetRot_dt(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[2]->GetPos_dt(), block_offset + 12, 0);
    mD.PasteQuaternion(m_nodes[2]->GetRot_dt(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[3]->GetPos_dt(), block_offset + 18, 0);
    mD.PasteQuaternion(m_nodes[3]->GetRot_dt(), block_offset + 21, 0);
}

void ChElementShellEANS4::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ChMatrixNM<double, 4, 1> N;
    ShapeFunctions(N, U, V, 0);
    for (unsigned int ii = 0; ii < 4; ii++) {
        Result += N(ii) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
void ChElementShellEANS4::LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChElementShellEANS4::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 4> N;
    ShapeFunctions(N, U, V, 0);

    detJ =  GetLengthX() *  GetLengthY() / 4.0;

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    ChVector<> Mv = F.ClipVector(3, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(0) * Mv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(1) * Mv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(2) * Mv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(3) * Mv;
    Qi.PasteVector(tmp, 21, 0);
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
void ChElementShellEANS4::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 4> N;
    ShapeFunctions(N, U, V, W);

    detJ =  GetLengthX() *  GetLengthY() / 4.0 ;
    detJ *= GetThickness();

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    ChVector<> Mv = F.ClipVector(3, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(0) * Mv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(1) * Mv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(2) * Mv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(3) * Mv;
    Qi.PasteVector(tmp, 21, 0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate average element density (needed for ChLoaderVolumeGravity).
double ChElementShellEANS4::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / m_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellEANS4::ComputeNormal(const double U, const double V) {

    ChMatrixNM<double, 1, 4> Nx;
    ChMatrixNM<double, 1, 4> Ny;
    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();

    ChVector<> Gx = Nx(0)*pA +
                    Nx(1)*pB +
                    Nx(2)*pC +
                    Nx(3)*pD;
    ChVector<> Gy = Ny(0)*pA +
                    Ny(1)*pB +
                    Ny(2)*pC +
                    Ny(3)*pD;

    ChVector<> mnorm = Vcross(Gx,Gy);
    return mnorm.GetNormalized();
}

// -----------------------------------------------------------------------------
// Utility functions for inverting a 5x5 matrix
// -----------------------------------------------------------------------------




// ============================================================================
// Implementation of ChElementShellANCF::Layer methods
// ============================================================================

// Private constructor (a layer can be created only by adding it to an element)
ChElementShellEANS4::Layer::Layer(ChElementShellEANS4* element,
                                 double thickness,
                                 double theta,
                                 std::shared_ptr<ChMaterialShellEANS> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer:
void ChElementShellEANS4::Layer::SetupInitial() {
    /*
    // Evaluate shape functions at element center
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    m_element->ShapeFunctionsDerivativeX(Nx, 0, 0, 0);
    m_element->ShapeFunctionsDerivativeY(Ny, 0, 0, 0);

    ChMatrixNM<double, 1, 3> Nx_d0 = Nx * m_element->m_d0;
    ChMatrixNM<double, 1, 3> Ny_d0 = Ny * m_element->m_d0;
    ChMatrixNM<double, 1, 3> Nz_d0 = Nz * m_element->m_d0;

    // Determinant of position vector gradient matrix: Initial configuration
    m_detJ0C = Nx_d0(0, 0) * Ny_d0(0, 1) * Nz_d0(0, 2) + Ny_d0(0, 0) * Nz_d0(0, 1) * Nx_d0(0, 2) +
               Nz_d0(0, 0) * Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1) * Nz_d0(0, 0) -
               Ny_d0(0, 2) * Nz_d0(0, 1) * Nx_d0(0, 0) - Nz_d0(0, 2) * Nx_d0(0, 1) * Ny_d0(0, 0);

    //// Transformation : Orthogonal transformation (A and J) ////
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x = Nx_d0[0][1] * Ny_d0[0][2] - Nx_d0[0][2] * Ny_d0[0][1];
    G1xG2.y = Nx_d0[0][2] * Ny_d0[0][0] - Nx_d0[0][0] * Ny_d0[0][2];
    G1xG2.z = Nx_d0[0][0] * Ny_d0[0][1] - Nx_d0[0][1] * Ny_d0[0][0];
    G1dotG1 = Nx_d0[0][0] * Nx_d0[0][0] + Nx_d0[0][1] * Nx_d0[0][1] + Nx_d0[0][2] * Nx_d0[0][2];

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x = Nx_d0[0][0];
    A1.y = Nx_d0[0][1];
    A1.z = Nx_d0[0][2];
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(m_theta) + A2 * sin(m_theta);
    AA2 = -A1 * sin(m_theta) + A2 * cos(m_theta);
    AA3 = A3;

    ////Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChMatrixNM<double, 9, 1> beta;

    j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
    j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
    j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
    j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
    j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
    j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
    j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
    j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
    j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
    j0.MatrDivScale(m_detJ0C);

    j01(0) = j0(0, 0);
    j02(0) = j0(1, 0);
    j03(0) = j0(2, 0);
    j01(1) = j0(0, 1);
    j02(1) = j0(1, 1);
    j03(1) = j0(2, 1);
    j01(2) = j0(0, 2);
    j02(2) = j0(1, 2);
    j03(2) = j0(2, 2);

    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Calculate T0: transformation matrix, function of fiber angle (see Yamashita et al, 2015, JCND)
    m_T0(0, 0) = pow(beta(0), 2);
    m_T0(1, 0) = pow(beta(1), 2);
    m_T0(2, 0) = 2.0 * beta(0) * beta(1);
    m_T0(3, 0) = pow(beta(2), 2);
    m_T0(4, 0) = 2.0 * beta(0) * beta(2);
    m_T0(5, 0) = 2.0 * beta(1) * beta(2);

    m_T0(0, 1) = pow(beta(3), 2);
    m_T0(1, 1) = pow(beta(4), 2);
    m_T0(2, 1) = 2.0 * beta(3) * beta(4);
    m_T0(3, 1) = pow(beta(5), 2);
    m_T0(4, 1) = 2.0 * beta(3) * beta(5);
    m_T0(5, 1) = 2.0 * beta(4) * beta(5);

    m_T0(0, 2) = beta(0) * beta(3);
    m_T0(1, 2) = beta(1) * beta(4);
    m_T0(2, 2) = beta(0) * beta(4) + beta(1) * beta(3);
    m_T0(3, 2) = beta(2) * beta(5);
    m_T0(4, 2) = beta(0) * beta(5) + beta(2) * beta(3);
    m_T0(5, 2) = beta(2) * beta(4) + beta(1) * beta(5);

    m_T0(0, 3) = pow(beta(6), 2);
    m_T0(1, 3) = pow(beta(7), 2);
    m_T0(2, 3) = 2.0 * beta(6) * beta(7);
    m_T0(3, 3) = pow(beta(8), 2);
    m_T0(4, 3) = 2.0 * beta(6) * beta(8);
    m_T0(5, 3) = 2.0 * beta(7) * beta(8);

    m_T0(0, 4) = beta(0) * beta(6);
    m_T0(1, 4) = beta(1) * beta(7);
    m_T0(2, 4) = beta(0) * beta(7) + beta(6) * beta(1);
    m_T0(3, 4) = beta(2) * beta(8);
    m_T0(4, 4) = beta(0) * beta(8) + beta(2) * beta(6);
    m_T0(5, 4) = beta(1) * beta(8) + beta(2) * beta(7);

    m_T0(0, 5) = beta(3) * beta(6);
    m_T0(1, 5) = beta(4) * beta(7);
    m_T0(2, 5) = beta(3) * beta(7) + beta(4) * beta(6);
    m_T0(3, 5) = beta(5) * beta(8);
    m_T0(4, 5) = beta(3) * beta(8) + beta(6) * beta(5);
    m_T0(5, 5) = beta(4) * beta(8) + beta(5) * beta(7);
    */
}

}  // end of namespace fea
}  // end of namespace chrono
