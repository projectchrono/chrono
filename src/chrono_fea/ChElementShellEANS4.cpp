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

#define CHSIMPLIFY_GAMMAS 
//#define CHUSE_KGEOMETRIC
#define CHUSE_ANS

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
    m_v.y = kur_v.y * 2* F;
    m_v.z = kur_v.z * m_beta * F;
}



//////////////////////////////





double ChElementShellEANS4::xi_i[ChElementShellEANS4::NUMGP][2] = {
	{-1. / std::sqrt(3.), -1. / std::sqrt(3.)},
	{ 1. / std::sqrt(3.), -1. / std::sqrt(3.)},
	{ 1. / std::sqrt(3.),  1. / std::sqrt(3.)},
	{-1. / std::sqrt(3.),  1. / std::sqrt(3.)}
};

double ChElementShellEANS4::w_i[ChElementShellEANS4::NUMGP] = {
	1.,
	1.,
	1.,
	1.
};

double ChElementShellEANS4::xi_S[ChElementShellEANS4::NUMSP][2] =  {
	{-1.,  0.},
	{ 1.,  0.},
	{ 0., -1.},
	{ 0.,  1.}
};

double ChElementShellEANS4::xi_n[ChElementShellEANS4::NUMNO][2] = {
	{ -1.,  -1.},
	{  1.,  -1.},
	{  1.,   1.},
	{ -1.,   1.}
};



// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellEANS4::ChElementShellEANS4() :  m_numLayers(0), m_thickness(0) {
    m_nodes.resize(4);
    m_Alpha = 0;

    iTa  = {QUNIT, QUNIT, QUNIT, QUNIT};
    T_i0 = {QUNIT, QUNIT, QUNIT, QUNIT};
    T_S0 = {QUNIT, QUNIT, QUNIT, QUNIT};
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
// Set as neutral position
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellEANS4::SetAsNeutral() {

    GetNodeA()->GetX0().SetPos( GetNodeA()->GetX0().GetPos() );
    GetNodeB()->GetX0().SetPos( GetNodeB()->GetX0().GetPos() );
    GetNodeC()->GetX0().SetPos( GetNodeC()->GetX0().GetPos() );
    GetNodeD()->GetX0().SetPos( GetNodeD()->GetX0().GetPos() );
    GetNodeA()->GetX0().SetRot( GetNodeA()->GetX0().GetRot() );
    GetNodeB()->GetX0().SetRot( GetNodeB()->GetX0().GetRot() );
    GetNodeC()->GetX0().SetRot( GetNodeC()->GetX0().GetRot() );
    GetNodeD()->GetX0().SetRot( GetNodeD()->GetX0().GetRot() );

}

// -----------------------------------------------------------------------------
// Compute instant net rotations and avg. rotation
// -----------------------------------------------------------------------------


void ChElementShellEANS4::ComputeNodeAndAverageRotations(
        const ChQuaternion<>& mrA, const ChQuaternion<>& mrB,
        const ChQuaternion<>& mrC, const ChQuaternion<>& mrD,  
        ChQuaternion<>& mTa, ChQuaternion<>& mTb, 
        ChQuaternion<>& mTc, ChQuaternion<>& mTd,
        ChVector<>& mphi_tilde_A, ChVector<>& mphi_tilde_B,
        ChVector<>& mphi_tilde_C, ChVector<>& mphi_tilde_D) {

    // Tn = Rn*iTa  = Rn*(R0'*t0)
    mTa = mrA * iTa[0];
    mTb = mrB * iTa[1];
    mTc = mrC * iTa[2];
    mTd = mrD * iTa[3];
    
    // Tavg = exp(log(1/4(Ta+Tb+Tc+Td))  : rather as:
    // average four rotations with quaternion averaging:
    ChQuaternion<> qTa = mTa;
    ChQuaternion<> qTb = mTb;
    if ( (qTa ^ qTb) < 0) 
        qTb *= -1;
    ChQuaternion<> qTc = mTc;
    if ( (qTa ^ qTc) < 0) 
        qTc *= -1;
    ChQuaternion<> qTd = mTd;
    if ( (qTa ^ qTd) < 0) 
        qTd *= -1;
    this->Tavg =(qTa + qTb + qTc + qTd ).GetNormalized();
    
    // Tavg' 
    ChQuaternion<> mTavgT = this->Tavg.GetConjugate();

    // R_rel_n = Tavg'* T_n
    // F_rel_n = log(R_rel_n)    i.e. to rot vector
    mphi_tilde_A = (mTavgT * qTa).Q_to_Rotv();
    mphi_tilde_B = (mTavgT * qTb).Q_to_Rotv();
    mphi_tilde_C = (mTavgT * qTc).Q_to_Rotv();
    mphi_tilde_D = (mTavgT * qTd).Q_to_Rotv();
 // if(mphi_tilde_A.Length()>0.03)  GetLog() << "mphi_tilde_A = " <<   mphi_tilde_A.Length()*CH_C_RAD_TO_DEG << "\n";
 // if(mphi_tilde_B.Length()>0.03)  GetLog() << "mphi_tilde_B = " <<   mphi_tilde_B.Length()*CH_C_RAD_TO_DEG << "\n";
 // if(mphi_tilde_C.Length()>0.03)  GetLog() << "mphi_tilde_C = " <<   mphi_tilde_C.Length()*CH_C_RAD_TO_DEG << "\n";
 // if(mphi_tilde_D.Length()>0.03)  GetLog() << "mphi_tilde_D = " <<   mphi_tilde_D.Length()*CH_C_RAD_TO_DEG << "\n";
}


// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellEANS4::SetupInitial(ChSystem* system) {

    // Align initial pos/rot of nodes to actual pos/rot
    SetAsNeutral();

    // Shortcuts:
    const std::array<const ChVector<>, 4> pi0 = {
        GetNodeA()->GetX0().GetPos(),
        GetNodeB()->GetX0().GetPos(),
        GetNodeC()->GetX0().GetPos(),
        GetNodeD()->GetX0().GetPos()};
    const std::array<const ChQuaternion<>, 4> ri0 = {
        GetNodeA()->GetX0().GetRot(),
        GetNodeB()->GetX0().GetRot(),
        GetNodeC()->GetX0().GetRot(),
        GetNodeD()->GetX0().GetRot()};
    const std::array<const ChVector<>, 4> pi = {
        GetNodeA()->GetPos(),
        GetNodeB()->GetPos(),
        GetNodeC()->GetPos(),
        GetNodeD()->GetPos()};
    const std::array<const ChQuaternion<>, 4> ri = {
        GetNodeA()->GetRot(),
        GetNodeB()->GetRot(),
        GetNodeC()->GetRot(),
        GetNodeD()->GetRot()};
    const ChQuaternion<>& rA = GetNodeA()->GetRot();
    const ChQuaternion<>& rB = GetNodeB()->GetRot();
    const ChQuaternion<>& rC = GetNodeC()->GetRot();
    const ChQuaternion<>& rD = GetNodeD()->GetRot();

    // Pre-compute the iTa rotations:
    for (int inode = 0; inode < NUMNO; inode++) {
        // Element shape functions
        double u = xi_n[inode][0];
        double v = xi_n[inode][1];
        ChMatrixNM<double, 1, 4> Nu;
        ChMatrixNM<double, 1, 4> Nv;
        this->ShapeFunctionsDerivativeX(Nu, u, v, 0);
        this->ShapeFunctionsDerivativeY(Nv, u, v, 0);
		ChVector<> t1 = Nu(0) * pi[0] +   Nu(1) * pi[1] +  Nu(2) * pi[2] +  Nu(3) * pi[3] ;
		ChVector<> t2 = Nv(0) * pi[0] +   Nv(1) * pi[1] +  Nv(2) * pi[2] +  Nv(3) * pi[3] ;
        t1.Normalize();
		t2.Normalize();
		ChVector<> t3 = Vcross(t1,t2);
		t3.Normalize();
		t2 = Vcross(t3,t1);
        ChMatrix33<> T123; 
        T123.Set_A_axis(t1,t2,t3);
        //GetLog() << "T123=" << ChMatrix33<>(T123) << "\n";
		iTa[inode] = ri0[inode].GetConjugate()  *  T123.Get_A_quaternion();  
	}

    // Compute net node rotations, average shell rotation, rot. vectors respect to average:
    ChQuaternion<>  Ta, Tb, Tc, Td;
    ChVector<>      phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D;
    ComputeNodeAndAverageRotations(
        rA, rB, rC, rD, //in
        Ta, Tb, Tc, Td, phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D); //out

    // Precompute Gauss-point values
    for (int igp = 0; igp < NUMGP; igp++) {
        // Element shape functions
        double u = xi_i[igp][0];
        double v = xi_i[igp][1];
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(N, u, v, 0);
        ChMatrixNM<double, 1, 4> Nu;
        ChMatrixNM<double, 1, 4> Nv;
        this->ShapeFunctionsDerivativeX(Nu, u, v, 0);
        this->ShapeFunctionsDerivativeY(Nv, u, v, 0);
		ChVector<> t1 = Nu(0) * pi[0] +   Nu(1) * pi[1] +  Nu(2) * pi[2] +  Nu(3) * pi[3] ;
		ChVector<> t2 = Nv(0) * pi[0] +   Nv(1) * pi[1] +  Nv(2) * pi[2] +  Nv(3) * pi[3] ;

        // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
        ChVector<> phi_tilde_i = N(0)*phi_tilde_A + 
                             N(1)*phi_tilde_B + 
                             N(2)*phi_tilde_C +
                             N(3)*phi_tilde_D;
    
        // T_i = Tavg*exp(phi_i)
        ChQuaternion<> Ri; 
        Ri.Q_from_Rotv(phi_tilde_i);
        T_i0[igp] = Tavg * Ri;                          // -----store T_i0;

        ChMatrix33<> T_0_i(T_i0[igp]);
        ChMatrixNM<double,2,2> S_alpha_beta_i;
		S_alpha_beta_i(0,0) = T_0_i.Get_A_Xaxis() ^ t1;
		S_alpha_beta_i(1,0) = T_0_i.Get_A_Yaxis() ^ t1;
		S_alpha_beta_i(0,1) = T_0_i.Get_A_Xaxis() ^ t2;
		S_alpha_beta_i(1,1) = T_0_i.Get_A_Yaxis() ^ t2;
		// alpha_i = det(S_alpha_beta_i)
		alpha_i[igp] = S_alpha_beta_i(0,0) * S_alpha_beta_i(1,1) -
			S_alpha_beta_i(0,1) * S_alpha_beta_i(1,0);      // -----store alpha_i;

        S_alpha_beta_i.MatrInverse();
        ChMatrixNM<double,4,2> L_alpha_B_i;
        L_alpha_B_i.PasteTranspMatrix(&Nu,0,0);
        L_alpha_B_i.PasteTranspMatrix(&Nv,0,1);

        L_alpha_beta_i[igp].MatrMultiply(L_alpha_B_i, S_alpha_beta_i); // -----store L_alpha_beta_i;

        // precompute iTa_i
        t1.Normalize();
		t2.Normalize();
		ChVector<> t3 = Vcross(t1,t2);
		t3.Normalize();
		t2 = Vcross(t3,t1);
        ChMatrix33<> T123; 
        T123.Set_A_axis(t1, t2, t3);
        //GetLog() << "T123=" << ChMatrix33<>(T123) << "\n";
		iTa_i[igp] = T_i0[igp].GetConjugate()  *  T123.Get_A_quaternion();  
    }

    // Precompute shear stitching point values
    for (int isp = 0; isp < NUMSP; isp++) {
        // Element shape functions
        double u = xi_S[isp][0];
        double v = xi_S[isp][1];
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(N, u, v, 0);
        ChMatrixNM<double, 1, 4> Nu;
        ChMatrixNM<double, 1, 4> Nv;
        this->ShapeFunctionsDerivativeX(Nu, u, v, 0);
        this->ShapeFunctionsDerivativeY(Nv, u, v, 0);
		ChVector<> t1 = Nu(0) * pi[0] +   Nu(1) * pi[1] +  Nu(2) * pi[2] +  Nu(3) * pi[3] ;
		ChVector<> t2 = Nv(0) * pi[0] +   Nv(1) * pi[1] +  Nv(2) * pi[2] +  Nv(3) * pi[3] ;

        // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
        ChVector<> phi_tilde_i = N(0)*phi_tilde_A + 
                             N(1)*phi_tilde_B + 
                             N(2)*phi_tilde_C +
                             N(3)*phi_tilde_D;
    
        // T_i = Tavg*exp(phi_i)
        ChQuaternion<> Ri; 
        Ri.Q_from_Rotv(phi_tilde_i);
        T_S0[isp] = Tavg * Ri;                          // -----store T_S0;

        ChMatrix33<> T_0_S(T_S0[isp]);
        ChMatrixNM<double,2,2> S_alpha_beta_S;
		S_alpha_beta_S(0,0) = T_0_S.Get_A_Xaxis() ^ t1;
		S_alpha_beta_S(1,0) = T_0_S.Get_A_Yaxis() ^ t1;
		S_alpha_beta_S(0,1) = T_0_S.Get_A_Xaxis() ^ t2;
		S_alpha_beta_S(1,1) = T_0_S.Get_A_Yaxis() ^ t2;

        S_alpha_beta_S.MatrInverse();
        ChMatrixNM<double,4,2> L_alpha_B_S;
        L_alpha_B_S.PasteTranspMatrix(&Nu,0,0);
        L_alpha_B_S.PasteTranspMatrix(&Nv,0,1);

        L_alpha_beta_S[isp].MatrMultiply(L_alpha_B_S, S_alpha_beta_S); // -----store L_alpha_beta_S;

        // precompute iTa_S
        t1.Normalize();
		t2.Normalize();
		ChVector<> t3 = Vcross(t1,t2);
		t3.Normalize();
		t2 = Vcross(t3,t1);
        ChMatrix33<> T123; 
        T123.Set_A_axis(t1, t2, t3);
        //GetLog() << "T123=" << ChMatrix33<>(T123) << "\n";
		iTa_S[isp] = T_S0[isp].GetConjugate()  *  T123.Get_A_quaternion(); 
    }




    // Perform layer initialization and accumulate element thickness. OBSOLETE
    m_numLayers = m_layers.size();
    m_thickness = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        m_layers[kl].SetupInitial();
        m_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the range for Gauss integration in the
    // z direction (values in [-1,1]). OBSOLETE
    m_GaussZ.push_back(-1);
    double z = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        z += m_layers[kl].Get_thickness();
        m_GaussZ.push_back(2 * z / m_thickness - 1);
    }

    // compute initial sizes (just for auxiliary information)
    m_lenX = (0.5*(GetNodeA()->coord.pos+GetNodeD()->coord.pos) - 0.5*(GetNodeB()->coord.pos+GetNodeC()->coord.pos) ).Length();
    m_lenY = (0.5*(GetNodeA()->coord.pos+GetNodeB()->coord.pos) - 0.5*(GetNodeD()->coord.pos+GetNodeC()->coord.pos) ).Length();

    // Compute mass matrix
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


//***TODO*** just lumped? maybe approximating as 4 tiles for the inertias?
void ChElementShellEANS4::ComputeMassMatrix() {
    m_MassMatrix.Reset();

    double thickness = this->GetLayer(0).GetMaterial()->Get_thickness();
    double rho =       this->GetLayer(0).GetMaterial()->Get_rho();

    for (int igp = 0; igp < NUMGP; igp++) {

        double jacobian = alpha_i[igp]; // scale by jacobian (determinant of parametric-carthesian transformation)

        // Element shape functions
        double u = xi_i[igp][0];
        double v = xi_i[igp][1];

        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(N, u, v, 0);
        double N00 = N(0)*N(0) *(rho * jacobian * thickness);
        double N11 = N(1)*N(1) *(rho * jacobian * thickness);
        double N22 = N(2)*N(2) *(rho * jacobian * thickness);
        double N33 = N(3)*N(3) *(rho * jacobian * thickness);
        double rotfactor = 0.01; //***TODO*** this just applies a bit of mass to rotational DOFs - not needed if implicit integration btw.
        m_MassMatrix(0,0)   += N00;
        m_MassMatrix(1,1)   += N00;
        m_MassMatrix(2,2)   += N00;
        m_MassMatrix(3,3)   += N00*rotfactor;
        m_MassMatrix(4,4)   += N00*rotfactor;
        m_MassMatrix(5,5)   += N00*rotfactor;

        m_MassMatrix(6,6)   += N11;
        m_MassMatrix(7,7)   += N11;
        m_MassMatrix(8,8)   += N11;
        m_MassMatrix(9,9)   += N11*rotfactor;
        m_MassMatrix(10,10) += N11*rotfactor;
        m_MassMatrix(11,11) += N11*rotfactor;

        m_MassMatrix(12,12) += N22;
        m_MassMatrix(13,13) += N22;
        m_MassMatrix(14,14) += N22;
        m_MassMatrix(15,15) += N22*rotfactor;
        m_MassMatrix(16,16) += N22*rotfactor;
        m_MassMatrix(17,17) += N22*rotfactor;

        m_MassMatrix(18,18) += N33;
        m_MassMatrix(19,19) += N33;
        m_MassMatrix(20,20) += N33;
        m_MassMatrix(21,21) += N33*rotfactor;
        m_MassMatrix(22,22) += N33*rotfactor;
        m_MassMatrix(23,23) += N33*rotfactor;
    }// end loop on gauss points
}


// Compute a 3x3 matrix as a tensor product between two vectors (outer product of vectors)
ChMatrix33<> TensorProduct(const ChVector<>& vA, const ChVector<>& vB) {
    ChMatrix33<> T;
    T(0,0) = vA.x*vB.x;
    T(0,1) = vA.x*vB.y;
    T(0,2) = vA.x*vB.z;
    T(1,0) = vA.y*vB.x;
    T(1,1) = vA.y*vB.y;
    T(1,2) = vA.y*vB.z;
    T(2,0) = vA.z*vB.x;
    T(2,1) = vA.z*vB.y;
    T(2,2) = vA.z*vB.z;
    return T;
}

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
    if (fabs(ang)>1e-7) {
        ChMatrix33<> Phi;
        Phi.Set_X_matrix(phi);
        H += (Phi * ((1.-cos(ang))/(ang*ang)));
        H += (Phi*Phi * ((ang-sin(ang))/(ang*ang*ang)));
    }
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------


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

    Fi.Reset();

    // Shortcuts
    const ChVector<>& pA0 = GetNodeA()->GetX0().GetPos();
    const ChVector<>& pB0 = GetNodeB()->GetX0().GetPos();
    const ChVector<>& pC0 = GetNodeC()->GetX0().GetPos();
    const ChVector<>& pD0 = GetNodeD()->GetX0().GetPos();

    // Compute net node rotations, average shell rotation, rot. vectors respect to average:
    ChQuaternion<>  Ta, Tb, Tc, Td;
    ChVector<>      phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D;
    ComputeNodeAndAverageRotations(
        rA, rB, rC, rD, //in
        Ta, Tb, Tc, Td, phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D); //out

    // Assumed Natural Strain (ANS):  precompute m_strainANS 
    CalcStrainANSbilinearShell(pA,rA, pB,rB, pC,rC, pD,rD);

    for (int igp = 0; igp < NUMGP; igp++) {

        // Element shape functions
        double u = xi_i[igp][0];
        double v = xi_i[igp][1];
        ChMatrixNM<double, 1, 4> N;
        this->ShapeFunctions(N, u, v, 0);
        ChMatrixNM<double, 1, 4> N_ANS;
        this->ShapeFunctionANSbilinearShell(N_ANS, u, v); 

        // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
        ChVector<> phi_tilde_i = N(0)*phi_tilde_A + 
                             N(1)*phi_tilde_B + 
                             N(2)*phi_tilde_C +
                             N(3)*phi_tilde_D;
    
        // T_i = Tavg*exp(phi_i)
        ChQuaternion<> Ri; 
        Ri.Q_from_Rotv(phi_tilde_i);
        ChQuaternion<> T_i = Tavg * Ri * iTa_i[igp];

        // STRAIN 
        // eps_u_tilde = T_i'* Yi,u - T_i0'* Yi,u0 
        // eps_v_tilde = T_i'* Yi,v - T_i0'* Yi,v0 
        ChVector<> yi_u =  L_alpha_beta_i[igp](0,0) * pA +   L_alpha_beta_i[igp](1,0) * pB +  L_alpha_beta_i[igp](2,0) * pC +  L_alpha_beta_i[igp](3,0) * pD ;
        ChVector<> yi_v =  L_alpha_beta_i[igp](0,1) * pA +   L_alpha_beta_i[igp](1,1) * pB +  L_alpha_beta_i[igp](2,1) * pC +  L_alpha_beta_i[igp](3,1) * pD ;
        ChVector<> yi_u0 =  L_alpha_beta_i[igp](0,0) * pA0 +   L_alpha_beta_i[igp](1,0) * pB0 +  L_alpha_beta_i[igp](2,0) * pC0 +  L_alpha_beta_i[igp](3,0) * pD0 ;
        ChVector<> yi_v0 =  L_alpha_beta_i[igp](0,1) * pA0 +   L_alpha_beta_i[igp](1,1) * pB0 +  L_alpha_beta_i[igp](2,1) * pC0 +  L_alpha_beta_i[igp](3,1) * pD0 ;
        ChVector<> eps_u_tilde = T_i.RotateBack ( yi_u ) - T_i0[igp].RotateBack(yi_u0);
        ChVector<> eps_v_tilde = T_i.RotateBack ( yi_v ) - T_i0[igp].RotateBack(yi_v0);

        // CURVATURES 
        ChVector<> F_u_tilde =  L_alpha_beta_i[igp](0,0) * phi_tilde_A +   L_alpha_beta_i[igp](1,0) * phi_tilde_B +  L_alpha_beta_i[igp](2,0) * phi_tilde_C +  L_alpha_beta_i[igp](3,0) * phi_tilde_D ;
        ChVector<> F_v_tilde =  L_alpha_beta_i[igp](0,1) * phi_tilde_A +   L_alpha_beta_i[igp](1,1) * phi_tilde_B +  L_alpha_beta_i[igp](2,1) * phi_tilde_C +  L_alpha_beta_i[igp](3,1) * phi_tilde_D ;

        ChMatrix33<> Hi;
        ComputeGammaMatrix(Hi,phi_tilde_i);

        #ifdef CHSIMPLIFY_GAMMAS
            Hi = ChMatrix33<>(1);
        #endif

        // kur_u = Tavg * gammatilde * F_u_tilde 
        // kur_v = Tavg * gammatilde * F_v_tilde
        ChVector<> kur_u = Tavg.Rotate( Hi * F_u_tilde);
        ChVector<> kur_v = Tavg.Rotate( Hi * F_v_tilde);

        // kur_u_tilde = T_i' * kur_u_i - T_i0' * kur0_u_i;
        // kur_v_tilde = T_i' * kur_v_i - T_i0' * kur0_v_i;
        ChVector<> kur_u_tilde = T_i.RotateBack ( kur_u ) - T_i0[igp].RotateBack(VNULL); //***TODO*** precompute kur0_u_i
        ChVector<> kur_v_tilde = T_i.RotateBack ( kur_v ) - T_i0[igp].RotateBack(VNULL); //***TODO*** precompute kur0_v_i


        // some complication: compute the Phi matrices:
        ChMatrix33<> Hai;
        ComputeGammaMatrixInverse(Hai,phi_tilde_A);
        ChMatrix33<> Hbi;
        ComputeGammaMatrixInverse(Hbi,phi_tilde_B);
        ChMatrix33<> Hci;
        ComputeGammaMatrixInverse(Hci,phi_tilde_C);
        ChMatrix33<> Hdi;
        ComputeGammaMatrixInverse(Hdi,phi_tilde_D);

        ChMatrix33<> mTavgT(Tavg.GetConjugate());
        ChMatrix33<> mTavg(Tavg);
        ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * GetNodeA()->GetA(); 
        ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * GetNodeB()->GetA();
        ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * GetNodeC()->GetA();
        ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * GetNodeD()->GetA();
        // note: respect to Masarati paper, added the ....* m_element->GetNodeA()->GetA() part because 
        // incremental rotations in C::E are considered in body coords, not in abs.coords 

        #ifdef CHSIMPLIFY_GAMMAS
            PhiA = GetNodeA()->GetA();
            PhiB = GetNodeB()->GetA();
            PhiC = GetNodeC()->GetA();
            PhiD = GetNodeD()->GetA();
        #endif

        // Build the B matrix:
        ChMatrixNM<double, 12,24> B;
    
        ChMatrix33<> mT_i_t(T_i.GetConjugate());
        ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
        ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);
        ChMatrix33<> mk_u_X; mk_u_X.Set_X_matrix(kur_u);
        ChMatrix33<> mk_v_X; mk_v_X.Set_X_matrix(kur_v);

        ChMatrix33<> block;
        block = mT_i_t * L_alpha_beta_i[igp](0,0);     B.PasteMatrix(&block, 0,0);
        block = mT_i_t * L_alpha_beta_i[igp](0,1);     B.PasteMatrix(&block, 3,0);
        block = mT_i_t * L_alpha_beta_i[igp](1,0);     B.PasteMatrix(&block, 0,6);
        block = mT_i_t * L_alpha_beta_i[igp](1,1);     B.PasteMatrix(&block, 3,6);
        block = mT_i_t * L_alpha_beta_i[igp](2,0);     B.PasteMatrix(&block, 0,12);
        block = mT_i_t * L_alpha_beta_i[igp](2,1);     B.PasteMatrix(&block, 3,12);
        block = mT_i_t * L_alpha_beta_i[igp](3,0);     B.PasteMatrix(&block, 0,18);
        block = mT_i_t * L_alpha_beta_i[igp](3,1);     B.PasteMatrix(&block, 3,18);
        
        block = mT_i_t * myi_u_X * PhiA * N(0);     B.PasteMatrix(&block, 0,3);
        block = mT_i_t * myi_v_X * PhiA * N(0);     B.PasteMatrix(&block, 3,3);
        block = mT_i_t * myi_u_X * PhiB * N(1);     B.PasteMatrix(&block, 0,9);
        block = mT_i_t * myi_v_X * PhiB * N(1);     B.PasteMatrix(&block, 3,9);
        block = mT_i_t * myi_u_X * PhiC * N(2);     B.PasteMatrix(&block, 0,15);
        block = mT_i_t * myi_v_X * PhiC * N(2);     B.PasteMatrix(&block, 3,15);
        block = mT_i_t * myi_u_X * PhiD * N(3);     B.PasteMatrix(&block, 0,21);
        block = mT_i_t * myi_v_X * PhiD * N(3);     B.PasteMatrix(&block, 3,21);

        ChMatrix33<> mKu = PhiA*L_alpha_beta_i[igp](0,0); // .. + Elle() term, to be added?
        ChMatrix33<> mKv = PhiA*L_alpha_beta_i[igp](0,1); // .. + Elle() term, to be added?
        block = mT_i_t * (mk_u_X * PhiA * N(0) + mKu);      B.PasteMatrix(&block, 6,3);
        block = mT_i_t * (mk_v_X * PhiA * N(0) + mKv);      B.PasteMatrix(&block, 9,3);
        mKu = PhiB*L_alpha_beta_i[igp](1,0); // .. + Elle() term, to be added?
        mKv = PhiB*L_alpha_beta_i[igp](1,1); // .. + Elle() term, to be added?
        block = mT_i_t * (mk_u_X * PhiB * N(1) + mKu);      B.PasteMatrix(&block, 6,9);
        block = mT_i_t * (mk_v_X * PhiB * N(1) + mKv);      B.PasteMatrix(&block, 9,9);
        mKu = PhiC*L_alpha_beta_i[igp](2,0); // .. + Elle() term, to be added?
        mKv = PhiC*L_alpha_beta_i[igp](2,1); // .. + Elle() term, to be added?
        block = mT_i_t * (mk_u_X * PhiC * N(2) + mKu);      B.PasteMatrix(&block, 6,15);
        block = mT_i_t * (mk_v_X * PhiC * N(2) + mKv);      B.PasteMatrix(&block, 9,15);
        mKu = PhiD*L_alpha_beta_i[igp](3,0); // .. + Elle() term, to be added?
        mKv = PhiD*L_alpha_beta_i[igp](3,1); // .. + Elle() term, to be added?
        block = mT_i_t * (mk_u_X * PhiD * N(3) + mKu);      B.PasteMatrix(&block, 6,21);
        block = mT_i_t * (mk_v_X * PhiD * N(3) + mKv);      B.PasteMatrix(&block, 9,21);

        // ANS CORRECTION:
        #ifdef CHUSE_ANS
        // replace transversal shear with ANS interpolated shear from tying points:
        eps_u_tilde.z = N_ANS(2)*this->m_strainANS(2,2)  // (0,-1)
                      + N_ANS(3)*this->m_strainANS(2,3); // (0,+1)
        eps_v_tilde.z = N_ANS(0)*this->m_strainANS(5,0)  // (-1,0)
                      + N_ANS(1)*this->m_strainANS(5,1); // (+1,0)

        for (int ic=0; ic<24; ++ic)
            B(2,ic) = N_ANS(2)* this->m_B3_ANS(2,ic) + 
                      N_ANS(3)* this->m_B3_ANS(3,ic);
        for (int ic=0; ic<24; ++ic)
            B(5,ic) = N_ANS(0)* this->m_B6_ANS(0,ic) + 
                      N_ANS(1)* this->m_B6_ANS(1,ic);              
        #endif
        // STRESSES - forces n and torques m 
        ChVector<> n_u;
        ChVector<> n_v;
        ChVector<> m_u;
        ChVector<> m_v;
        this->GetLayer(0).GetMaterial()->ComputeStress(n_u, n_v, m_u, m_v, eps_u_tilde, eps_v_tilde, kur_u_tilde, kur_v_tilde);

        // CONVERT n AND m TO GENERALIZED FORCES 'result':
        ChMatrixNM<double,12,1> sigma;
        sigma.PasteVector(n_u, 0,0);
        sigma.PasteVector(n_v, 3,0);
        sigma.PasteVector(m_u, 6,0);
        sigma.PasteVector(m_v, 9,0);

        // F = int{B*sigma*dv} , so at each Gauss integration point:  
        //   F_i = B_i * sigma_i  
        ChMatrixNM<double,24,1> result;

        result.MatrTMultiply(B,sigma);

        result.MatrScale(- alpha_i[igp]); // scale by jacobian (determinant of parametric-carthesian transformation)

        Fi += result;

        //***TODO*** add optional EAS terms

    }  // end loop on gauss points
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------


void ChElementShellEANS4::ComputeInternalJacobians(double Kfactor, double Rfactor) {

    m_JacobianMatrix.Reset();

    bool use_numerical_differentiation = false;
    
    if (use_numerical_differentiation) {

        double diff = 1e-6;
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
               // Assumed Natural Strain (ANS):  precompute m_strainANS 
        CalcStrainANSbilinearShell( GetNodeA()->GetPos(),GetNodeA()->GetRot(),
                                    GetNodeB()->GetPos(),GetNodeB()->GetRot(),
                                    GetNodeC()->GetPos(),GetNodeC()->GetRot(),
                                    GetNodeD()->GetPos(),GetNodeD()->GetRot());

        // Shortcuts:
        const ChVector<>& pA0 = GetNodeA()->GetX0().GetPos();
        const ChVector<>& pB0 = GetNodeB()->GetX0().GetPos();
        const ChVector<>& pC0 = GetNodeC()->GetX0().GetPos();
        const ChVector<>& pD0 = GetNodeD()->GetX0().GetPos();
        const ChVector<>& pA = GetNodeA()->GetPos();
        const ChVector<>& pB = GetNodeB()->GetPos();
        const ChVector<>& pC = GetNodeC()->GetPos();
        const ChVector<>& pD = GetNodeD()->GetPos();
        const ChQuaternion<>& rA = GetNodeA()->GetRot();
        const ChQuaternion<>& rB = GetNodeB()->GetRot();
        const ChQuaternion<>& rC = GetNodeC()->GetRot();
        const ChQuaternion<>& rD = GetNodeD()->GetRot();

        // Compute net node rotations, average shell rotation, rot. vectors respect to average:
        ChQuaternion<>  Ta, Tb, Tc, Td;
        ChVector<>      phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D;
        ComputeNodeAndAverageRotations(
        rA, rB, rC, rD, //in
        Ta, Tb, Tc, Td, phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D); //out

        for (int igp = 0; igp < NUMGP; igp++) {

            // Element shape functions
            double u = xi_i[igp][0];
            double v = xi_i[igp][1];
            ChMatrixNM<double, 1, 4> N;
            this->ShapeFunctions(N, u, v, 0);
            ChMatrixNM<double, 1, 4> N_ANS;
            this->ShapeFunctionANSbilinearShell(N_ANS, u, v);

            ChMatrixNM<double, 1, 4> Nu;
            ChMatrixNM<double, 1, 4> Nv;
            this->ShapeFunctionsDerivativeX(Nu, u, v, 0);
            this->ShapeFunctionsDerivativeY(Nv, u, v, 0);  
    
            // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
            ChVector<> phi_tilde_i = N(0)*phi_tilde_A + 
                                 N(1)*phi_tilde_B + 
                                 N(2)*phi_tilde_C +
                                 N(3)*phi_tilde_D;
    
            // T_i = Tavg*exp(phi_i)
            ChQuaternion<> Ri; 
            Ri.Q_from_Rotv(phi_tilde_i);
            ChQuaternion<> T_i = Tavg * Ri * iTa_i[igp];

            // STRAIN 
            // eps_u_tilde = T_i'* Yi,u - T_i0'* Yi,u0 
            // eps_v_tilde = T_i'* Yi,v - T_i0'* Yi,v0 
            ChVector<> yi_u =  L_alpha_beta_i[igp](0,0) * pA +   L_alpha_beta_i[igp](1,0) * pB +  L_alpha_beta_i[igp](2,0) * pC +  L_alpha_beta_i[igp](3,0) * pD ;
            ChVector<> yi_v =  L_alpha_beta_i[igp](0,1) * pA +   L_alpha_beta_i[igp](1,1) * pB +  L_alpha_beta_i[igp](2,1) * pC +  L_alpha_beta_i[igp](3,1) * pD ;
            ChVector<> yi_u0 =  L_alpha_beta_i[igp](0,0) * pA0 +   L_alpha_beta_i[igp](1,0) * pB0 +  L_alpha_beta_i[igp](2,0) * pC0 +  L_alpha_beta_i[igp](3,0) * pD0 ;
            ChVector<> yi_v0 =  L_alpha_beta_i[igp](0,1) * pA0 +   L_alpha_beta_i[igp](1,1) * pB0 +  L_alpha_beta_i[igp](2,1) * pC0 +  L_alpha_beta_i[igp](3,1) * pD0 ;
            ChVector<> eps_u_tilde = T_i.RotateBack ( yi_u ) - T_i0[igp].RotateBack(yi_u0);
            ChVector<> eps_v_tilde = T_i.RotateBack ( yi_v ) - T_i0[igp].RotateBack(yi_v0);

            // CURVATURES
            ChVector<> F_u_tilde =  L_alpha_beta_i[igp](0,0) * phi_tilde_A +   L_alpha_beta_i[igp](1,0) * phi_tilde_B +  L_alpha_beta_i[igp](2,0) * phi_tilde_C +  L_alpha_beta_i[igp](3,0) * phi_tilde_D ;
            ChVector<> F_v_tilde =  L_alpha_beta_i[igp](0,1) * phi_tilde_A +   L_alpha_beta_i[igp](1,1) * phi_tilde_B +  L_alpha_beta_i[igp](2,1) * phi_tilde_C +  L_alpha_beta_i[igp](3,1) * phi_tilde_D ;

            ChMatrix33<> Hi;
            ComputeGammaMatrix(Hi,phi_tilde_i);

            #ifdef CHSIMPLIFY_GAMMAS
                Hi = ChMatrix33<>(1);
            #endif

            // kur_u = Tavg * gammatilde * F_u_tilde 
            // kur_v = Tavg * gammatilde * F_v_tilde
            ChVector<> kur_u = Tavg.Rotate( Hi * F_u_tilde);
            ChVector<> kur_v = Tavg.Rotate( Hi * F_v_tilde);

            // kur_u_tilde = T_i' * kur_u_i - T_i0' * kur0_u_i;
            // kur_v_tilde = T_i' * kur_v_i - T_i0' * kur0_v_i;
            ChVector<> kur_u_tilde = T_i.RotateBack ( kur_u ) - T_i0[igp].RotateBack(VNULL); //***TODO*** precompute kur0_u_i
            ChVector<> kur_v_tilde = T_i.RotateBack ( kur_v ) - T_i0[igp].RotateBack(VNULL); //***TODO*** precompute kur0_v_i

            // some complication: compute the Phi matrices:
            ChMatrix33<> Hai;
            ComputeGammaMatrixInverse(Hai,phi_tilde_A);
            ChMatrix33<> Hbi;
            ComputeGammaMatrixInverse(Hbi,phi_tilde_B);
            ChMatrix33<> Hci;
            ComputeGammaMatrixInverse(Hci,phi_tilde_C);
            ChMatrix33<> Hdi;
            ComputeGammaMatrixInverse(Hdi,phi_tilde_D);

            ChMatrix33<> mTavgT(Tavg.GetConjugate());
            ChMatrix33<> mTavg(Tavg);
            ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * GetNodeA()->GetA(); 
            ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * GetNodeB()->GetA();
            ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * GetNodeC()->GetA();
            ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * GetNodeD()->GetA();
            // note: respect to Masarati paper, added the ....* m_element->GetNodeA()->GetA() part because 
            // incremental rotations in C::E are considered in body coords, not in abs.coords 

            #ifdef CHSIMPLIFY_GAMMAS
                PhiA = GetNodeA()->GetA();
                PhiB = GetNodeB()->GetA();
                PhiC = GetNodeC()->GetA();
                PhiD = GetNodeD()->GetA();
            #endif

            // Build the B matrix:
            ChMatrixNM<double, 12,24> B;
    
            ChMatrix33<> mT_i_t(T_i.GetConjugate());
            ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
            ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);
            ChMatrix33<> mk_u_X; mk_u_X.Set_X_matrix(kur_u);
            ChMatrix33<> mk_v_X; mk_v_X.Set_X_matrix(kur_v);

            ChMatrix33<> block;
            block = mT_i_t * L_alpha_beta_i[igp](0,0);     B.PasteMatrix(&block, 0,0);
            block = mT_i_t * L_alpha_beta_i[igp](0,1);     B.PasteMatrix(&block, 3,0);
            block = mT_i_t * L_alpha_beta_i[igp](1,0);     B.PasteMatrix(&block, 0,6);
            block = mT_i_t * L_alpha_beta_i[igp](1,1);     B.PasteMatrix(&block, 3,6);
            block = mT_i_t * L_alpha_beta_i[igp](2,0);     B.PasteMatrix(&block, 0,12);
            block = mT_i_t * L_alpha_beta_i[igp](2,1);     B.PasteMatrix(&block, 3,12);
            block = mT_i_t * L_alpha_beta_i[igp](3,0);     B.PasteMatrix(&block, 0,18);
            block = mT_i_t * L_alpha_beta_i[igp](3,1);     B.PasteMatrix(&block, 3,18);
        
            block = mT_i_t * myi_u_X * PhiA * N(0);     B.PasteMatrix(&block, 0,3);
            block = mT_i_t * myi_v_X * PhiA * N(0);     B.PasteMatrix(&block, 3,3);
            block = mT_i_t * myi_u_X * PhiB * N(1);     B.PasteMatrix(&block, 0,9);
            block = mT_i_t * myi_v_X * PhiB * N(1);     B.PasteMatrix(&block, 3,9);
            block = mT_i_t * myi_u_X * PhiC * N(2);     B.PasteMatrix(&block, 0,15);
            block = mT_i_t * myi_v_X * PhiC * N(2);     B.PasteMatrix(&block, 3,15);
            block = mT_i_t * myi_u_X * PhiD * N(3);     B.PasteMatrix(&block, 0,21);
            block = mT_i_t * myi_v_X * PhiD * N(3);     B.PasteMatrix(&block, 3,21);

            ChMatrix33<> mKuA = PhiA*L_alpha_beta_i[igp](0,0); // .. + Elle() term, to be added?
            ChMatrix33<> mKvA = PhiA*L_alpha_beta_i[igp](0,1); // .. + Elle() term, to be added?
            block = mT_i_t * (mk_u_X * PhiA * N(0) + mKuA);      B.PasteMatrix(&block, 6,3);
            block = mT_i_t * (mk_v_X * PhiA * N(0) + mKvA);      B.PasteMatrix(&block, 9,3);
            ChMatrix33<> mKuB = PhiB*L_alpha_beta_i[igp](1,0); // .. + Elle() term, to be added?
            ChMatrix33<> mKvB = PhiB*L_alpha_beta_i[igp](1,1); // .. + Elle() term, to be added?
            block = mT_i_t * (mk_u_X * PhiB * N(1) + mKuB);      B.PasteMatrix(&block, 6,9);
            block = mT_i_t * (mk_v_X * PhiB * N(1) + mKvB);      B.PasteMatrix(&block, 9,9);
            ChMatrix33<> mKuC = PhiC*L_alpha_beta_i[igp](2,0); // .. + Elle() term, to be added?
            ChMatrix33<> mKvC = PhiC*L_alpha_beta_i[igp](2,1); // .. + Elle() term, to be added?
            block = mT_i_t * (mk_u_X * PhiC * N(2) + mKuC);      B.PasteMatrix(&block, 6,15);
            block = mT_i_t * (mk_v_X * PhiC * N(2) + mKvC);      B.PasteMatrix(&block, 9,15);
            ChMatrix33<> mKuD = PhiD*L_alpha_beta_i[igp](3,0); // .. + Elle() term, to be added?
            ChMatrix33<> mKvD = PhiD*L_alpha_beta_i[igp](3,1); // .. + Elle() term, to be added?
            block = mT_i_t * (mk_u_X * PhiD * N(3) + mKuD);      B.PasteMatrix(&block, 6,21);
            block = mT_i_t * (mk_v_X * PhiD * N(3) + mKvD);      B.PasteMatrix(&block, 9,21);

            // ANS CORRECTION:
            #ifdef CHUSE_ANS
            // replace transversal shear with ANS interpolated shear from tying points:

            eps_u_tilde.z = N_ANS(2)*this->m_strainANS(2,2)  // (0,-1)
                          + N_ANS(3)*this->m_strainANS(2,3); // (0,+1)
            eps_v_tilde.z = N_ANS(0)*this->m_strainANS(5,0)  // (-1,0)
                          + N_ANS(1)*this->m_strainANS(5,1); // (+1,0)

            for (int ic=0; ic<24; ++ic)
                B(2,ic) = N_ANS(2)* this->m_B3_ANS(2,ic) + 
                          N_ANS(3)* this->m_B3_ANS(3,ic);
            for (int ic=0; ic<24; ++ic)
                B(5,ic) = N_ANS(0)* this->m_B6_ANS(0,ic) + 
                          N_ANS(1)* this->m_B6_ANS(1,ic);
            #endif
            // STRESSES - forces n and torques m 
            ChVector<> n_u;
            ChVector<> n_v;
            ChVector<> m_u;
            ChVector<> m_v;
            this->GetLayer(0).GetMaterial()->ComputeStress(n_u, n_v, m_u, m_v, eps_u_tilde, eps_v_tilde, kur_u_tilde, kur_v_tilde);

            // COMPUTE material tangent matrix K_m

            ChMatrixNM<double,12,1> CBj;
            ChMatrixNM<double,24,1> Kmj;
            ChVector<> n_u_j;
            ChVector<> n_v_j;
            ChVector<> m_u_j;
            ChVector<> m_v_j;
            ChVector<> beps_u_tilde;
            ChVector<> beps_v_tilde;
            ChVector<> bkur_u_tilde;
            ChVector<> bkur_v_tilde;
            ChMatrixNM<double,24,24> K_m_i;
            for (int j=0; j<24; ++j) {
                beps_u_tilde = B.ClipVector(0,j);
                beps_v_tilde = B.ClipVector(3,j);
                bkur_u_tilde = B.ClipVector(6,j);
                bkur_v_tilde = B.ClipVector(9,j);
                this->GetLayer(0).GetMaterial()->ComputeStress(n_u_j, n_v_j, m_u_j, m_v_j, beps_u_tilde, beps_v_tilde, bkur_u_tilde, bkur_v_tilde);
                CBj.PasteVector(n_u_j, 0,0);
                CBj.PasteVector(n_v_j, 3,0);
                CBj.PasteVector(m_u_j, 6,0);
                CBj.PasteVector(m_v_j, 9,0);
        
                Kmj.MatrTMultiply(B,CBj);
                K_m_i.PasteMatrix(&Kmj,0,j);
            }

            double jacobian = alpha_i[igp]; // scale by jacobian (determinant of parametric-carthesian transformation)

            K_m_i.MatrScale(jacobian * (Kfactor + Rfactor * this->m_Alpha));
            this->m_JacobianMatrix += K_m_i;


            // COMPUTE geometric tangent matrix
            #ifdef CHUSE_KGEOMETRIC
                ChMatrixNM<double,24,24> K_g_i;
                ChMatrixNM<double, 15,24> D_overline_i;
                ChMatrix33<> Ieye(1);
                block = Ieye * L_alpha_beta_i[igp](0,0);     B.PasteMatrix(&block, 0,0);
                block = Ieye * L_alpha_beta_i[igp](0,1);     B.PasteMatrix(&block, 3,0);
                block = Ieye * L_alpha_beta_i[igp](1,0);     B.PasteMatrix(&block, 0,6);
                block = Ieye * L_alpha_beta_i[igp](1,1);     B.PasteMatrix(&block, 3,6);
                block = Ieye * L_alpha_beta_i[igp](2,0);     B.PasteMatrix(&block, 0,12);
                block = Ieye * L_alpha_beta_i[igp](2,1);     B.PasteMatrix(&block, 3,12);
                block = Ieye * L_alpha_beta_i[igp](3,0);     B.PasteMatrix(&block, 0,18);
                block = Ieye * L_alpha_beta_i[igp](3,1);     B.PasteMatrix(&block, 3,18);
                B.PasteMatrix(&mKuA, 6,3);
                B.PasteMatrix(&mKvA, 9,3);
                B.PasteMatrix(&mKuB, 6,9);
                B.PasteMatrix(&mKvB, 9,9);
                B.PasteMatrix(&mKuC, 6,15);
                B.PasteMatrix(&mKvC, 9,15);
                B.PasteMatrix(&mKuD, 6,21);
                B.PasteMatrix(&mKvD, 9,21);
                block = PhiA * N(0);  B.PasteMatrix(&block, 12,3);
                block = PhiB * N(1);  B.PasteMatrix(&block, 12,9);
                block = PhiC * N(2);  B.PasteMatrix(&block, 12,15);
                block = PhiD * N(3);  B.PasteMatrix(&block, 12,21);

                ChMatrixNM<double, 15,15> G_i;
                ChMatrix33<> mT_i(T_i);          
		        ChVector<> Tn_u = mT_i * n_u;
		        ChVector<> Tn_v = mT_i * n_v;
		        ChVector<> Tm_u = mT_i * m_u;
		        ChVector<> Tm_v = mT_i * m_v;
           
                block.Set_X_matrix(Tn_u); G_i.PasteMatrix(&block, 12,0);
                block.Set_X_matrix(Tn_v); G_i.PasteMatrix(&block, 12,3);
                block.Set_X_matrix(Tm_u); G_i.PasteMatrix(&block, 12,6);
                block.Set_X_matrix(Tm_v); G_i.PasteMatrix(&block, 12,9);
                block.Set_X_matrix(Tn_u); G_i.PasteTranspMatrix(&block, 0,12);
                block.Set_X_matrix(Tn_v); G_i.PasteTranspMatrix(&block, 3,12);

                ChMatrix33<> Hh;
		        Hh =  TensorProduct(Tn_u, yi_u) - ChMatrix33<>(Tn_u ^ yi_u)
			        + TensorProduct(Tn_v, yi_v) - ChMatrix33<>(Tn_v ^ yi_v)
			        + TensorProduct(Tm_u, kur_u) - ChMatrix33<>(Tm_u ^ kur_u)
			        + TensorProduct(Tm_v, kur_v) - ChMatrix33<>(Tm_v ^ kur_v);
                G_i.PasteMatrix(&Hh, 12,12);
            
                // K_g_i = D' * G * D
                ChMatrixNM<double,24,15> DtG_i;
                DtG_i.MatrTMultiply(D_overline_i,G_i);
                K_g_i.MatrTMultiply(DtG_i, D_overline_i); 

                K_g_i.MatrScale(jacobian * (Kfactor + Rfactor * this->m_Alpha));
                this->m_JacobianMatrix += K_g_i;
            #endif

            //***TODO*** add optional EAS terms
            
        } // end loop on gauss points
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellEANS4::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {

    N(0) = 0.25 * (1.0 - x) * (1.0 - y);
    N(1) = 0.25 * (1.0 + x) * (1.0 - y);
    N(2) = 0.25 * (1.0 + x) * (1.0 + y);
    N(3) = 0.25 * (1.0 - x) * (1.0 + y);
}

void ChElementShellEANS4::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {

    Nx(0) = - 0.25 * (1.0 - y);
    Nx(1) =   0.25 * (1.0 - y);
    Nx(2) =   0.25 * (1.0 + y);
    Nx(3) = - 0.25 * (1.0 + y);
}

void ChElementShellEANS4::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {

    Ny(0) = - 0.25 * (1.0 - x);
    Ny(1) = - 0.25 * (1.0 + x);
    Ny(2) =   0.25 * (1.0 + x);
    Ny(3) =   0.25 * (1.0 - x);
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

    // Shortcuts
    const ChVector<>& pA0 = GetNodeA()->GetX0().GetPos();
    const ChVector<>& pB0 = GetNodeB()->GetX0().GetPos();
    const ChVector<>& pC0 = GetNodeC()->GetX0().GetPos();
    const ChVector<>& pD0 = GetNodeD()->GetX0().GetPos();

    ChMatrixNM<double, 1, 4> N;

    // Compute net node rotations, average shell rotation, rot. vectors respect to average:
    ChQuaternion<>  Ta, Tb, Tc, Td;
    ChVector<>      phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D;
    ComputeNodeAndAverageRotations(
        rA, rB, rC, rD, //in
        Ta, Tb, Tc, Td, phi_tilde_A, phi_tilde_B, phi_tilde_C, phi_tilde_D); //out

    for (int isp = 0; isp < 4; isp++) {
        double xi_Au = xi_S[isp][0];
        double xi_Av = xi_S[isp][1];
        ShapeFunctions(N, xi_Au, xi_Av, 0);
    
        // phi_i = sum ( Ni * log(R_rel_i))  at this i-th  integration point
        ChVector<> phi_tilde_i = N(0)*phi_tilde_A + 
                             N(1)*phi_tilde_B + 
                             N(2)*phi_tilde_C +
                             N(3)*phi_tilde_D;
    
        // T_i = Tavg*exp(phi_i)
        ChQuaternion<> Ri; 
        Ri.Q_from_Rotv(phi_tilde_i);
        ChQuaternion<> T_i = Tavg * Ri * iTa_S[isp];

        // STRAIN  
        // eps_u_tilde = T_i'* Yi,u - T_i0'* Yi,u0 = T_i' * sum_n(Nin,u Yn) - T_S0'* Yi,u0
        // eps_v_tilde = T_i'* Yi,v - T_i0'* Yi,v0 = T_i' * sum_n(Nin,v Yn) - T_S0'* Yi,v0
        ChVector<> yi_u =  L_alpha_beta_S[isp](0,0) * pA +   L_alpha_beta_S[isp](1,0) * pB +  L_alpha_beta_S[isp](2,0) * pC +  L_alpha_beta_S[isp](3,0) * pD ;
        ChVector<> yi_v =  L_alpha_beta_S[isp](0,1) * pA +   L_alpha_beta_S[isp](1,1) * pB +  L_alpha_beta_S[isp](2,1) * pC +  L_alpha_beta_S[isp](3,1) * pD ;
        ChVector<> yi_u0 =  L_alpha_beta_S[isp](0,0) * pA0 +   L_alpha_beta_S[isp](1,0) * pB0 +  L_alpha_beta_S[isp](2,0) * pC0 +  L_alpha_beta_S[isp](3,0) * pD0 ;
        ChVector<> yi_v0 =  L_alpha_beta_S[isp](0,1) * pA0 +   L_alpha_beta_S[isp](1,1) * pB0 +  L_alpha_beta_S[isp](2,1) * pC0 +  L_alpha_beta_S[isp](3,1) * pD0 ;
        ChVector<> eps_u_tilde = T_i.RotateBack ( yi_u ) - T_S0[isp].RotateBack(yi_u0);
        ChVector<> eps_v_tilde = T_i.RotateBack ( yi_v ) - T_S0[isp].RotateBack(yi_v0);

        this->m_strainANS.PasteVector(eps_u_tilde,0,isp);
        this->m_strainANS.PasteVector(eps_v_tilde,3,isp);

        ChMatrix33<> Hi;
        ComputeGammaMatrix(Hi,phi_tilde_i);

        #ifdef CHSIMPLIFY_GAMMAS
            Hi = ChMatrix33<>(1);
        #endif

        // some complication: compute the Phi matrices:
        ChMatrix33<> Hai;
        ComputeGammaMatrixInverse(Hai,phi_tilde_A);
        ChMatrix33<> Hbi;
        ComputeGammaMatrixInverse(Hbi,phi_tilde_B);
        ChMatrix33<> Hci;
        ComputeGammaMatrixInverse(Hci,phi_tilde_C);
        ChMatrix33<> Hdi;
        ComputeGammaMatrixInverse(Hdi,phi_tilde_D);

        ChMatrix33<> mTavgT(Tavg.GetConjugate());
        ChMatrix33<> mTavg(Tavg);
        ChMatrix33<> PhiA = mTavg * Hi * Hai * mTavgT * this->GetNodeA()->GetA(); 
        ChMatrix33<> PhiB = mTavg * Hi * Hbi * mTavgT * this->GetNodeB()->GetA();
        ChMatrix33<> PhiC = mTavg * Hi * Hci * mTavgT * this->GetNodeC()->GetA();
        ChMatrix33<> PhiD = mTavg * Hi * Hdi * mTavgT * this->GetNodeD()->GetA();

        #ifdef CHSIMPLIFY_GAMMAS
            PhiA = GetNodeA()->GetA();
            PhiB = GetNodeB()->GetA();
            PhiC = GetNodeC()->GetA();
            PhiD = GetNodeD()->GetA();
        #endif

        ChMatrix33<> mT_i_t(T_i.GetConjugate());
        ChMatrix33<> myi_u_X; myi_u_X.Set_X_matrix(yi_u);
        ChMatrix33<> myi_v_X; myi_v_X.Set_X_matrix(yi_v);

        ChMatrixDynamic<> B_ans(6,24);
        ChMatrix33<> block;
        block = mT_i_t * L_alpha_beta_S[isp](0,0);     B_ans.PasteMatrix(&block, 0,0);
        block = mT_i_t * L_alpha_beta_S[isp](0,1);     B_ans.PasteMatrix(&block, 3,0);
        block = mT_i_t * L_alpha_beta_S[isp](1,0);     B_ans.PasteMatrix(&block, 0,6);
        block = mT_i_t * L_alpha_beta_S[isp](1,1);     B_ans.PasteMatrix(&block, 3,6);
        block = mT_i_t * L_alpha_beta_S[isp](2,0);     B_ans.PasteMatrix(&block, 0,12);
        block = mT_i_t * L_alpha_beta_S[isp](2,1);     B_ans.PasteMatrix(&block, 3,12);
        block = mT_i_t * L_alpha_beta_S[isp](3,0);     B_ans.PasteMatrix(&block, 0,18);
        block = mT_i_t * L_alpha_beta_S[isp](3,1);     B_ans.PasteMatrix(&block, 3,18);
        
        block = mT_i_t * myi_u_X * PhiA * N(0);     B_ans.PasteMatrix(&block, 0,3);
        block = mT_i_t * myi_v_X * PhiA * N(0);     B_ans.PasteMatrix(&block, 3,3);
        block = mT_i_t * myi_u_X * PhiB * N(1);     B_ans.PasteMatrix(&block, 0,9);
        block = mT_i_t * myi_v_X * PhiB * N(1);     B_ans.PasteMatrix(&block, 3,9);
        block = mT_i_t * myi_u_X * PhiC * N(2);     B_ans.PasteMatrix(&block, 0,15);
        block = mT_i_t * myi_v_X * PhiC * N(2);     B_ans.PasteMatrix(&block, 3,15);
        block = mT_i_t * myi_u_X * PhiD * N(3);     B_ans.PasteMatrix(&block, 0,21);
        block = mT_i_t * myi_v_X * PhiD * N(3);     B_ans.PasteMatrix(&block, 3,21);

        this->m_B3_ANS.PasteClippedMatrix(&B_ans, 2,0, 1,24, isp,0);
        this->m_B6_ANS.PasteClippedMatrix(&B_ans, 5,0, 1,24, isp,0);
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





// Private constructor (a layer can be created only by adding it to an element)
ChElementShellEANS4::Layer::Layer(ChElementShellEANS4* element,
                                 double thickness,
                                 double theta,
                                 std::shared_ptr<ChMaterialShellEANS> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer:
void ChElementShellEANS4::Layer::SetupInitial() {
 
}

}  // end of namespace fea
}  // end of namespace chrono
