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
// Authors: Antonio Recuero, Radu Serban
// =============================================================================
// ANCF beam element with 3 nodes.
// =============================================================================

//// RADU
//// A lot more to do here...
//// - more use of Eigen expressions
//// - remove unecessary initializations to zero

#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChElementBeamANCF.h"
#include <cmath>

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementBeamANCF::ChElementBeamANCF() : m_lenX(0), m_thicknessY(0), m_thicknessZ(0), m_Alpha(0), m_gravity_on(false) {
    m_nodes.resize(3);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementBeamANCF::SetNodes(std::shared_ptr<ChNodeFEAxyzDD> nodeA,
                                 std::shared_ptr<ChNodeFEAxyzDD> nodeB,
                                 std::shared_ptr<ChNodeFEAxyzDD> nodeC) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;

    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[0]->Variables_DD());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables_DD());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[2]->Variables_D());
    mvars.push_back(&m_nodes[2]->Variables_DD());

    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    CalcCoordMatrix(m_d0);
    m_d0d0T = m_d0 * m_d0.transpose();
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementBeamANCF::SetupInitial(ChSystem* system) {
    // Compute mass matrix and gravitational forces (constant)
    m_GaussScaling = (m_lenX * m_thicknessY * m_thicknessZ) / 8;
    ComputeMassMatrix();
    ComputeGravityForce(system->Get_G_acc());
    // Cache the scaling factor (due to change of integration intervals)
}

// State update.
void ChElementBeamANCF::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector with the current field values at the element nodes.
void ChElementBeamANCF::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(6, 3) = m_nodes[0]->GetDD().eigen();
    mD.segment(9, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(12, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(15, 3) = m_nodes[1]->GetDD().eigen();
    mD.segment(18, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(21, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(24, 3) = m_nodes[2]->GetDD().eigen();
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementBeamANCF::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 27) && (H.cols() == 27));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 27; i++)
        for (int j = 0; j < 27; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementBeamANCF::ComputeMmatrixGlobal(ChMatrixRef M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class BeamANCF_Mass : public ChIntegrable3D<ChMatrixNM<double, 27, 27>> {
  public:
    BeamANCF_Mass(ChElementBeamANCF* element) : m_element(element) {}
    ~BeamANCF_Mass() {}

  private:
    ChElementBeamANCF* m_element;

    virtual void Evaluate(ChMatrixNM<double, 27, 27>& result, const double x, const double y, const double z) override;
};

void BeamANCF_Mass::Evaluate(ChMatrixNM<double, 27, 27>& result, const double x, const double y, const double z) {
    ChElementBeamANCF::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3) N9*eye(3)]

    ChMatrixNM<double, 3, 27> S;
    S.setZero();

    for (int i = 0; i < 9; i++) {
        S(0, 3 * i + 0) = N(i);
        S(1, 3 * i + 1) = N(i);
        S(2, 3 * i + 2) = N(i);
    }

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S
    result = S.transpose() * S;

    // multiply integration weights
    result *= detJ0 * (m_element->m_GaussScaling);
};

void ChElementBeamANCF::ComputeMassMatrix() {
    m_MassMatrix.setZero();

    double rho = GetMaterial()->Get_rho();
    BeamANCF_Mass myformula(this);
    ChMatrixNM<double, 27, 27> TempMassMatrix;
    TempMassMatrix.setZero();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 27, 27>>(TempMassMatrix,  // result of integration will go there
                                                          myformula,       // formula to integrate
                                                          -1, 1,           // x limits
                                                          -1, 1,           // y limits
                                                          -1, 1,           // z limits
                                                          3                // order of integration
    );
    TempMassMatrix *= rho;
    m_MassMatrix += TempMassMatrix;
}
/// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementBeamANCF::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0, 0) + m_MassMatrix(0, 9) + m_MassMatrix(0, 18);
    m_nodes[1]->m_TotalMass += m_MassMatrix(9, 9) + m_MassMatrix(9, 0) + m_MassMatrix(9, 18);
    m_nodes[2]->m_TotalMass += m_MassMatrix(18, 18) + m_MassMatrix(18, 0) + m_MassMatrix(18, 9);
}
// -----------------------------------------------------------------------------
// Gravitational force calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the element gravity forces
class BeamANCF_Gravity : public ChIntegrable3D<ChVectorN<double, 27>> {
  public:
    BeamANCF_Gravity(ChElementBeamANCF* element, const ChVector<> gacc) : m_element(element), m_gacc(gacc) {}
    ~BeamANCF_Gravity() {}

  private:
    ChElementBeamANCF* m_element;
    ChVector<> m_gacc;

    virtual void Evaluate(ChVectorN<double, 27>& result, const double x, const double y, const double z) override;
};

void BeamANCF_Gravity::Evaluate(ChVectorN<double, 27>& result, const double x, const double y, const double z) {
    ChElementBeamANCF::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    for (int i = 0; i < 9; i++) {
        result(i * 3 + 0) = N(i) * m_gacc.x();
        result(i * 3 + 1) = N(i) * m_gacc.y();
        result(i * 3 + 2) = N(i) * m_gacc.z();
    }

    result *= detJ0 * m_element->m_GaussScaling;
};

void ChElementBeamANCF::ComputeGravityForce(const ChVector<>& g_acc) {
    //// RADU
    //// why set GravForce to zero when it's overwritten below?!?
    //// why even have Fgravity at all (instead of loading directly GravForce)?

    m_GravForce.setZero();

    double rho = GetMaterial()->Get_rho();
    BeamANCF_Gravity myformula(this, g_acc);
    ChVectorN<double, 27> Fgravity;
    Fgravity.setZero();
    ChQuadrature::Integrate3D<ChVectorN<double, 27>>(Fgravity,   // result of integration will go there
                                                     myformula,  // formula to integrate
                                                     -1, 1,      // x limits
                                                     -1, 1,      // y limits
                                                     -1, 1,      // z limits
                                                     3           // order of integration
    );

    m_GravForce += Fgravity * rho;
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

// The class BeamANCF_Force provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The 27 entries in the integrand represent the internal force.

class BeamANCF_Force : public ChIntegrable3D<ChVectorN<double, 27>> {
  public:
    BeamANCF_Force(ChElementBeamANCF* element) : m_element(element) {}
    ~BeamANCF_Force() {}

  private:
    ChElementBeamANCF* m_element;

    /// Evaluate (strainD'*strain)  at point x, include ANS and EAS.
    virtual void Evaluate(ChVectorN<double, 27>& result, const double x, const double y, const double z) override;
};

void BeamANCF_Force::Evaluate(ChVectorN<double, 27>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementBeamANCF::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementBeamANCF::ShapeVector Nx;
    ChElementBeamANCF::ShapeVector Ny;
    ChElementBeamANCF::ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1);
    G1xG2.y() = Nx_d0(0, 2) * Ny_d0(0, 0) - Nx_d0(0, 0) * Ny_d0(0, 2);
    G1xG2.z() = Nx_d0(0, 0) * Ny_d0(0, 1) - Nx_d0(0, 1) * Ny_d0(0, 0);
    G1dotG1 = Nx_d0(0, 0) * Nx_d0(0, 0) + Nx_d0(0, 1) * Nx_d0(0, 1) + Nx_d0(0, 2) * Nx_d0(0, 2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0, 0);
    A1.y() = Nx_d0(0, 1);
    A1.z() = Nx_d0(0, 2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    double theta = 0.0;  // TODO
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;

    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
    j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
    j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
    j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
    j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
    j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
    j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
    j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
    j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    ChVectorN<double, 9> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 9> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 9> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 9> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 9> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 9> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * (Nx.dot(ddNx) - Nx.dot(d0d0Nx));
    strain_til(1) = 0.5 * (Ny.dot(ddNy) - Ny.dot(d0d0Ny));
    strain_til(2) = Nx.dot(ddNy) - Nx.dot(d0d0Ny);
    strain_til(3) = 0.5 * (Nz.dot(ddNz) - Nz.dot(d0d0Nz));
    strain_til(4) = Nx.dot(ddNz) - Nx.dot(d0d0Nz);
    strain_til(5) = Ny.dot(ddNz) - Ny.dot(d0d0Nz);

    // For orthotropic material
    ChVectorN<double, 6> strain;

    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));
    // std::cout << "With beta: " << strain(0, 0) << " " << strain(1, 0) << " " << strain(2, 0) << " " <<
    //    strain(3, 0) << " " << strain(4, 0) << " " << strain(5, 0) << " \n";

    // Strain derivative component
    ChMatrixNM<double, 6, 27> strainD_til;
    ChMatrixNM<double, 1, 27> tempB;
    ChMatrixNM<double, 1, 3> tempB3;   // x
    ChMatrixNM<double, 1, 3> tempB31;  // y
    ChMatrixNM<double, 1, 3> tempB32;  // z

    //// RADU
    //// Check this for more compact Eigen expressions

    strainD_til.setZero();

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(i);
        }
    }
    strainD_til.row(0) = tempB;  // xx

    tempB31 = Ny * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(i);
        }
    }
    strainD_til.row(1) = tempB;  // strainD for yy

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nx(i) + tempB3(0, j) * Ny(i);
        }
    }
    strainD_til.row(2) = tempB;  // strainD for xy

    tempB32 = Nz * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(i);
        }
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(i) + tempB32(0, j) * Nx(i);
        }
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nz(i) + tempB32(0, j) * Ny(i);
        }
    }
    strainD_til.row(5) = tempB;  // strainD for yz

    // For orthotropic material
    ChMatrixNM<double, 6, 27> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 27; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 27; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: the input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetMaterial()->Get_E_eps();

    // Internal force calculation
    ChMatrixNM<double, 27, 6> tempC = strainD.transpose() * E_eps;

    // for (unsigned int i = 0; i < 26; i++) {
    //    tempC(i, 0) = E_eps(0, 0) * strainD(0, i);
    //    tempC(i, 1) = E_eps(1, 1) * strainD(1, i);
    //    tempC(i, 2) = E_eps(2, 2) * strainD(2, i);
    //    tempC(i, 3) = E_eps(3, 3) * strainD(3, i);
    //    tempC(i, 4) = E_eps(4, 4) * strainD(4, i);
    //    tempC(i, 5) = E_eps(5, 5) * strainD(5, i);
    //}
    result = (tempC * strain) * (detJ0 * m_element->m_GaussScaling);
}

// The class BeamANCF_ForceNu provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The 27 entries in the integrand represent the internal force.

class BeamANCF_ForceNu : public ChIntegrable1D<ChVectorN<double, 27>> {
  public:
    BeamANCF_ForceNu(ChElementBeamANCF* element) : m_element(element) {}
    ~BeamANCF_ForceNu() {}

  private:
    ChElementBeamANCF* m_element;

    /// Evaluate (strainD'*strain)  at point x
    virtual void Evaluate(ChVectorN<double, 27>& result, const double x) override;
};

void BeamANCF_ForceNu::Evaluate(ChVectorN<double, 27>& result, const double x) {
    // Element shape function
    ChElementBeamANCF::ShapeVector N;
    double y = 0;
    double z = 0;

    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementBeamANCF::ShapeVector Nx;
    ChElementBeamANCF::ShapeVector Ny;
    ChElementBeamANCF::ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1);
    G1xG2.y() = Nx_d0(0, 2) * Ny_d0(0, 0) - Nx_d0(0, 0) * Ny_d0(0, 2);
    G1xG2.z() = Nx_d0(0, 0) * Ny_d0(0, 1) - Nx_d0(0, 1) * Ny_d0(0, 0);
    G1dotG1 = Nx_d0(0, 0) * Nx_d0(0, 0) + Nx_d0(0, 1) * Nx_d0(0, 1) + Nx_d0(0, 2) * Nx_d0(0, 2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0, 0);
    A1.y() = Nx_d0(0, 1);
    A1.z() = Nx_d0(0, 2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    double theta = 0.0;  // TODO
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
    j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
    j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
    j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
    j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
    j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
    j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
    j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
    j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    ChVectorN<double, 9> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 9> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 9> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 9> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 9> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 9> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * (Nx.dot(ddNx) - Nx.dot(d0d0Nx));
    strain_til(1) = 0.5 * (Ny.dot(ddNy) - Ny.dot(d0d0Ny));
    strain_til(2) = Nx.dot(ddNy) - Nx.dot(d0d0Ny);
    strain_til(3) = 0.5 * (Nz.dot(ddNz) - Nz.dot(d0d0Nz));
    strain_til(4) = Nx.dot(ddNz) - Nx.dot(d0d0Nz);
    strain_til(5) = Ny.dot(ddNz) - Ny.dot(d0d0Nz);

    // For orthotropic material
    ChVectorN<double, 6> strain;

    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Strain derivative component

    ChMatrixNM<double, 6, 27> strainD_til;
    ChMatrixNM<double, 1, 27> tempB;
    ChMatrixNM<double, 1, 3> tempB3;   // x
    ChMatrixNM<double, 1, 3> tempB31;  // y
    ChMatrixNM<double, 1, 3> tempB32;  // z

    strainD_til.setZero();

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;  // strain for xx

    tempB31 = Ny * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;  // strainD for yz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nx(0, i) + tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(2) = tempB;  // strainD for xy

    tempB32 = Nz * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nz(0, i) + tempB32(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(5) = tempB;  // strainD for yz

    // For orthotropic material
    ChMatrixNM<double, 6, 27> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 27; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 27; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: the input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetMaterial()->Get_E_eps_Nu();

    // Internal force calculation
    ChMatrixNM<double, 27, 6> tempC = strainD.transpose() * E_eps;
    result = (tempC * strain) * (detJ0 * m_element->GetLengthX() / 2);
}

void ChElementBeamANCF::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    // Current nodal coordinates and velocities
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT = m_d * m_d.transpose();

    Fi.setZero();

    // Three-dimensional integration of Poisson-less terms
    BeamANCF_Force formula(this);
    ChVectorN<double, 27> result;
    result.setZero();
    ChQuadrature::Integrate3D<ChVectorN<double, 27>>(result,   // result of integration
                                                     formula,  // integrand formula
                                                     -1, 1,    // x limits
                                                     -1, 1,    // y limits
                                                     -1, 1,    // z limits
                                                     3         // order of integration
    );

    // Accumulate internal force
    Fi -= result;

    if (GetStrainFormulation() == ChElementBeamANCF::StrainFormulation::CMPoisson) {
        // One-dimensional integration of Poisson terms (over centerline)
        BeamANCF_ForceNu formula_Nu(this);
        result.setZero();
        ChQuadrature::Integrate1D<ChVectorN<double, 27>>(result,      // result of integration
                                                         formula_Nu,  // integrand formula
                                                         -1, 1,       // x limits
                                                         2            // order of integration
        );

        // Accumulate internal force
        Fi -= result * (GetThicknessY() * GetThicknessZ());
    }

    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

// The class BeamANCF_Jacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer of an ANCF
// shell element.
// The integrated quantity represents the 27x27 Jacobian
//      Kfactor * [K] + Rfactor * [R]

class BeamANCF_Jacobian : public ChIntegrable3D<ChMatrixNM<double, 27, 27>> {
  public:
    BeamANCF_Jacobian(ChElementBeamANCF* element,  // Containing element
                      double Kfactor,              // Scaling coefficient for stiffness component
                      double Rfactor               // Scaling coefficient for damping component
                      )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {}

  private:
    ChElementBeamANCF* m_element;
    double m_Kfactor;
    double m_Rfactor;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChMatrixNM<double, 27, 27>& result, const double x, const double y, const double z) override;
};

void BeamANCF_Jacobian::Evaluate(ChMatrixNM<double, 27, 27>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementBeamANCF::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementBeamANCF::ShapeVector Nx;
    ChElementBeamANCF::ShapeVector Ny;
    ChElementBeamANCF::ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1);
    G1xG2.y() = Nx_d0(0, 2) * Ny_d0(0, 0) - Nx_d0(0, 0) * Ny_d0(0, 2);
    G1xG2.z() = Nx_d0(0, 0) * Ny_d0(0, 1) - Nx_d0(0, 1) * Ny_d0(0, 0);
    G1dotG1 = Nx_d0(0, 0) * Nx_d0(0, 0) + Nx_d0(0, 1) * Nx_d0(0, 1) + Nx_d0(0, 2) * Nx_d0(0, 2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0, 0);
    A1.y() = Nx_d0(0, 1);
    A1.z() = Nx_d0(0, 2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    double theta = 0.0;  // TODO
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
    j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
    j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
    j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
    j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
    j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
    j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
    j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
    j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    ChVectorN<double, 9> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 9> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 9> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 9> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 9> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 9> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * (Nx.dot(ddNx) - Nx.dot(d0d0Nx));
    strain_til(1) = 0.5 * (Ny.dot(ddNy) - Ny.dot(d0d0Ny));
    strain_til(2) = Nx.dot(ddNy) - Nx.dot(d0d0Ny);
    strain_til(3) = 0.5 * (Nz.dot(ddNz) - Nz.dot(d0d0Nz));
    strain_til(4) = Nx.dot(ddNz) - Nx.dot(d0d0Nz);
    strain_til(5) = Ny.dot(ddNz) - Ny.dot(d0d0Nz);

    // For orthotropic material
    ChVectorN<double, 6> strain;

    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Strain derivative component

    ChMatrixNM<double, 6, 27> strainD_til;
    ChMatrixNM<double, 1, 27> tempB;
    ChMatrixNM<double, 1, 3> tempB3;   // x
    ChMatrixNM<double, 1, 3> tempB31;  // y
    ChMatrixNM<double, 1, 3> tempB32;  // z

    strainD_til.setZero();

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;  // strain for xx

    tempB31 = Ny * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;  // strainD for yz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nx(0, i) + tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(2) = tempB;  // strainD for xy

    tempB32 = Nz * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nz(0, i) + tempB32(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(5) = tempB;  // strainD for yz

    // For orthotropic material
    ChMatrixNM<double, 6, 27> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 27; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    /// Gd : Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
    ChMatrixNM<double, 9, 27> Gd;
    Gd.setZero();

    for (int ii = 0; ii < 9; ii++) {
        Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(1, 3 * (ii) + 1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(2, 3 * (ii) + 2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

        Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(4, 3 * (ii) + 1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(5, 3 * (ii) + 2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

        Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(7, 3 * (ii) + 1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(8, 3 * (ii) + 2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
    }

    // Structural damping
    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();

    for (int ii = 0; ii < 27; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: The input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetMaterial()->Get_E_eps();

    // Stress tensor calculation
    ChVectorN<double, 6> stress;
    for (unsigned int i = 0; i < 6; i++) {
        stress(i) = E_eps(i, i) * strain(i);
    }

    // Declaration and computation of Sigm, to be removed
    ChMatrixNM<double, 9, 9> Sigm;  ///< Rearrangement of stress vector (not always needed)
    Sigm.setZero();

    Sigm(0, 0) = stress(0);  // XX
    Sigm(1, 1) = stress(0);
    Sigm(2, 2) = stress(0);

    Sigm(0, 3) = stress(2);  // XY
    Sigm(1, 4) = stress(2);
    Sigm(2, 5) = stress(2);

    Sigm(0, 6) = stress(4);  // XZ
    Sigm(1, 7) = stress(4);
    Sigm(2, 8) = stress(4);

    Sigm(3, 0) = stress(2);  // XY
    Sigm(4, 1) = stress(2);
    Sigm(5, 2) = stress(2);

    Sigm(3, 3) = stress(1);  // YY
    Sigm(4, 4) = stress(1);
    Sigm(5, 5) = stress(1);

    Sigm(3, 6) = stress(5);  // YZ
    Sigm(4, 7) = stress(5);
    Sigm(5, 8) = stress(5);

    Sigm(6, 0) = stress(4);  // XZ
    Sigm(7, 1) = stress(4);
    Sigm(8, 2) = stress(4);

    Sigm(6, 3) = stress(5);  // YZ
    Sigm(7, 4) = stress(5);
    Sigm(8, 5) = stress(5);

    Sigm(6, 6) = stress(3);  // ZZ
    Sigm(7, 7) = stress(3);
    Sigm(8, 8) = stress(3);

    // Jacobian of internal forces (excluding the EAS contribution).
    ChMatrixNM<double, 27, 6> temp276 = strainD.transpose() * E_eps;  // TODO : Simplify, E_eps is diagonal
    ChMatrixNM<double, 27, 9> temp279 = Gd.transpose() * Sigm;

    // Load result vector (integrand)
    result = (temp276 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp279 * Gd) * m_Kfactor;
    result *= detJ0 * m_element->m_GaussScaling;
}

class BeamANCF_JacobianNu : public ChIntegrable1D<ChMatrixNM<double, 27, 27>> {
  public:
    BeamANCF_JacobianNu(ChElementBeamANCF* element,  // Containing element
                        double Kfactor,              // Scaling coefficient for stiffness component
                        double Rfactor               // Scaling coefficient for damping component
                        )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {}

  private:
    ChElementBeamANCF* m_element;
    double m_Kfactor;
    double m_Rfactor;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChMatrixNM<double, 27, 27>& result, const double x) override;
};

void BeamANCF_JacobianNu::Evaluate(ChMatrixNM<double, 27, 27>& result, const double x) {
    // Element shape function
    ChElementBeamANCF::ShapeVector N;
    double y = 0;
    double z = 0;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementBeamANCF::ShapeVector Nx;
    ChElementBeamANCF::ShapeVector Ny;
    ChElementBeamANCF::ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1);
    G1xG2.y() = Nx_d0(0, 2) * Ny_d0(0, 0) - Nx_d0(0, 0) * Ny_d0(0, 2);
    G1xG2.z() = Nx_d0(0, 0) * Ny_d0(0, 1) - Nx_d0(0, 1) * Ny_d0(0, 0);
    G1dotG1 = Nx_d0(0, 0) * Nx_d0(0, 0) + Nx_d0(0, 1) * Nx_d0(0, 1) + Nx_d0(0, 2) * Nx_d0(0, 2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0, 0);
    A1.y() = Nx_d0(0, 1);
    A1.z() = Nx_d0(0, 2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    double theta = 0.0;  // TODO
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
    j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
    j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
    j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
    j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
    j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
    j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
    j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
    j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    ChVectorN<double, 9> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 9> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 9> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 9> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 9> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 9> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * (Nx.dot(ddNx) - Nx.dot(d0d0Nx));
    strain_til(1) = 0.5 * (Ny.dot(ddNy) - Ny.dot(d0d0Ny));
    strain_til(2) = Nx.dot(ddNy) - Nx.dot(d0d0Ny);
    strain_til(3) = 0.5 * (Nz.dot(ddNz) - Nz.dot(d0d0Nz));
    strain_til(4) = Nx.dot(ddNz) - Nx.dot(d0d0Nz);
    strain_til(5) = Ny.dot(ddNz) - Ny.dot(d0d0Nz);

    // For orthotropic material
    ChVectorN<double, 6> strain;

    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1, 0) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Strain derivative component

    ChMatrixNM<double, 6, 27> strainD_til;
    ChMatrixNM<double, 1, 27> tempB;
    ChMatrixNM<double, 1, 3> tempB3;   // x
    ChMatrixNM<double, 1, 3> tempB31;  // y
    ChMatrixNM<double, 1, 3> tempB32;  // z

    strainD_til.setZero();

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;  // strain for xx

    tempB31 = Ny * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;  // strainD for yz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nx(0, i) + tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(2) = tempB;  // strainD for xy

    tempB32 = Nz * m_element->m_d;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Nz(0, i) + tempB32(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(5) = tempB;  // strainD for yz

    // For orthotropic material
    ChMatrixNM<double, 6, 27> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 27; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    /// Gd : Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
    ChMatrixNM<double, 9, 27> Gd;
    Gd.setZero();

    for (int ii = 0; ii < 9; ii++) {
        Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(1, 3 * (ii) + 1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(2, 3 * (ii) + 2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

        Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(4, 3 * (ii) + 1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(5, 3 * (ii) + 2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

        Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(7, 3 * (ii) + 1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(8, 3 * (ii) + 2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
    }

    // Structural damping
    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 27; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: The input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetMaterial()->Get_E_eps_Nu();

    // Stress tensor calculation
    ChVectorN<double, 6> stress = E_eps * strain;

    // Declaration and computation of Sigm, to be removed
    ChMatrixNM<double, 9, 9> Sigm;  ///< Rearrangement of stress vector (not always needed)
    Sigm.setZero();

    Sigm(0, 0) = stress(0);  // XX
    Sigm(1, 1) = stress(0);
    Sigm(2, 2) = stress(0);

    Sigm(0, 3) = stress(2);  // XY
    Sigm(1, 4) = stress(2);
    Sigm(2, 5) = stress(2);

    Sigm(0, 6) = stress(4);  // XZ
    Sigm(1, 7) = stress(4);
    Sigm(2, 8) = stress(4);

    Sigm(3, 0) = stress(2);  // XY
    Sigm(4, 1) = stress(2);
    Sigm(5, 2) = stress(2);

    Sigm(3, 3) = stress(1);  // YY
    Sigm(4, 4) = stress(1);
    Sigm(5, 5) = stress(1);

    Sigm(3, 6) = stress(5);  // YZ
    Sigm(4, 7) = stress(5);
    Sigm(5, 8) = stress(5);

    Sigm(6, 0) = stress(4);  // XZ
    Sigm(7, 1) = stress(4);
    Sigm(8, 2) = stress(4);

    Sigm(6, 3) = stress(5);  // YZ
    Sigm(7, 4) = stress(5);
    Sigm(8, 5) = stress(5);

    Sigm(6, 6) = stress(3);  // ZZ
    Sigm(7, 7) = stress(3);
    Sigm(8, 8) = stress(3);

    // Jacobian of internal forces
    ChMatrixNM<double, 27, 6> temp276 = strainD.transpose() * E_eps;
    ChMatrixNM<double, 27, 9> temp279 = Gd.transpose() * Sigm;

    // Load result vector (integrand)
    result = (temp276 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp279 * Gd) * m_Kfactor;
    result *= detJ0 * m_element->GetLengthX() / 2;
}

void ChElementBeamANCF::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    // Note that the matrices with current nodal coordinates and velocities are
    // already available in m_d and m_d_dt (as set in ComputeInternalForces).
    // Similarly, the ANS strain and strain derivatives are already available in
    // m_strainANS and m_strainANS_D (as calculated in ComputeInternalForces).

    //// RADU
    //// There is no need to initialize the Jacobian to zero as it is completely overwritten below...
    m_JacobianMatrix.setZero();

    // Jacobian from diagonal terms D0 (three-dimensional)
    BeamANCF_Jacobian formula(this, Kfactor, Rfactor);
    ChMatrixNM<double, 27, 27> result;
    result.setZero();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 27, 27>>(result,   // result of integration
                                                          formula,  // integrand formula
                                                          -1, 1,    // x limits
                                                          -1, 1,    // y limits
                                                          -1, 1,    // z limits
                                                          3         // order of integration
    );

    m_JacobianMatrix = result;

    // Jacobian from diagonal terms Dv (one-dimensional)
    if (GetStrainFormulation() == ChElementBeamANCF::StrainFormulation::CMPoisson) {
        BeamANCF_JacobianNu formula_Nu(this, Kfactor, Rfactor);
        result.setZero();
        ChQuadrature::Integrate1D<ChMatrixNM<double, 27, 27>>(result,      // result of integration
                                                              formula_Nu,  // integrand formula
                                                              -1, 1,       // x limits
                                                              2            // order of integration
        );

        // Extract matrices from result of integration
        result *= GetThicknessY() * GetThicknessZ();

        m_JacobianMatrix += result;
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementBeamANCF::ShapeFunctions(ShapeVector& N, double x, double y, double z) {
    double b = GetThicknessY();
    double c = GetThicknessZ();

    N(0) = -x / 2 * (1 - x);
    N(3) = x / 2 * (1 + x);
    N(6) = -(x - 1) * (x + 1);
    N(1) = y * b / 2 * N(0);
    N(2) = z * c / 2 * N(0);
    N(4) = y * b / 2 * N(3);
    N(5) = z * c / 2 * N(3);
    N(7) = y * b / 2 * N(6);
    N(8) = z * c / 2 * N(6);
}

void ChElementBeamANCF::ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z) {
    double L = GetLengthX();
    double b = GetThicknessY();
    double c = GetThicknessZ();

    Nx(0) = (2 * x - 1) / L;
    Nx(1) = (b * y * (2 * x - 1)) / (2 * L);
    Nx(2) = (c * z * (2 * x - 1)) / (2 * L);
    Nx(3) = (2 * x + 1) / L;
    Nx(4) = (b * y * (2 * x + 1)) / (2 * L);
    Nx(5) = (c * z * (2 * x + 1)) / (2 * L);
    Nx(6) = -(4 * x) / L;
    Nx(7) = -(2 * b * x * y) / L;
    Nx(8) = -(2 * c * x * z) / L;
}

void ChElementBeamANCF::ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z) {
    Ny(0) = 0;
    Ny(1) = (x * (x - 1)) / 2;
    Ny(2) = 0;
    Ny(3) = 0;
    Ny(4) = (x * (x + 1)) / 2;
    Ny(5) = 0;
    Ny(6) = 0;
    Ny(7) = 1 - x * x;
    Ny(8) = 0;
}

void ChElementBeamANCF::ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z) {
    Nz(0) = 0;
    Nz(1) = 0;
    Nz(2) = (x * (x - 1)) / 2;
    Nz(3) = 0;
    Nz(4) = 0;
    Nz(5) = (x * (x + 1)) / 2;
    Nz(6) = 0;
    Nz(7) = 0;
    Nz(8) = 1 - x * x;
}

void ChElementBeamANCF::Calc_Sxi_D(ChMatrixNM<double, 9, 3>& Sxi_D, double xi, double eta, double zeta) {
    Sxi_D(0, 0) = xi - 0.5;
    Sxi_D(1, 0) = 0.25*m_thicknessY*eta*(2.0*xi - 1.0);
    Sxi_D(2, 0) = 0.25*m_thicknessZ*zeta*(2.0*xi - 1.0);
    Sxi_D(3, 0) = xi + 0.5;
    Sxi_D(4, 0) = 0.25*m_thicknessY*eta*(2.0*xi + 1.0);
    Sxi_D(5, 0) = 0.25*m_thicknessZ*zeta*(2.0*xi + 1.0);
    Sxi_D(6, 0) = -2.0*xi;
    Sxi_D(7, 0) = -m_thicknessY*eta*xi;
    Sxi_D(8, 0) = -m_thicknessZ*zeta*xi;

    Sxi_D(0, 1) = 0.0;
    Sxi_D(1, 1) = 0.25*m_thicknessY*(xi*xi - xi);
    Sxi_D(2, 1) = 0.0;
    Sxi_D(3, 1) = 0.0;
    Sxi_D(4, 1) = 0.25*m_thicknessY*(xi*xi + xi);
    Sxi_D(5, 1) = 0.0;
    Sxi_D(6, 1) = 0.0;
    Sxi_D(7, 1) = 0.5*m_thicknessY*(1 - xi*xi);
    Sxi_D(8, 1) = 0.0;

    Sxi_D(0, 2) = 0.0;
    Sxi_D(1, 2) = 0.0;
    Sxi_D(2, 2) = 0.25*m_thicknessZ*(xi*xi - xi);
    Sxi_D(3, 2) = 0.0;
    Sxi_D(4, 2) = 0.0;
    Sxi_D(5, 2) = 0.25*m_thicknessZ*(xi*xi + xi);
    Sxi_D(6, 2) = 0.0;
    Sxi_D(7, 2) = 0.0;
    Sxi_D(8, 2) = 0.5*m_thicknessZ*(1 - xi*xi);
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------
double ChElementBeamANCF::Calc_detJ0(double x,
                                     double y,
                                     double z,
                                     ShapeVector& Nx,
                                     ShapeVector& Ny,
                                     ShapeVector& Nz,
                                     ChMatrixNM<double, 1, 3>& Nx_d0,
                                     ChMatrixNM<double, 1, 3>& Ny_d0,
                                     ChMatrixNM<double, 1, 3>& Nz_d0) {
    ShapeFunctionsDerivativeX(Nx, x, y, z);
    ShapeFunctionsDerivativeY(Ny, x, y, z);
    ShapeFunctionsDerivativeZ(Nz, x, y, z);

    Nx_d0 = Nx * m_d0;
    Ny_d0 = Ny * m_d0;
    Nz_d0 = Nz * m_d0;

    double detJ0 = Nx_d0(0, 0) * Ny_d0(0, 1) * Nz_d0(0, 2) + Ny_d0(0, 0) * Nz_d0(0, 1) * Nx_d0(0, 2) +
                   Nz_d0(0, 0) * Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1) * Nz_d0(0, 0) -
                   Ny_d0(0, 2) * Nz_d0(0, 1) * Nx_d0(0, 0) - Nz_d0(0, 2) * Nx_d0(0, 1) * Ny_d0(0, 0);

    return detJ0;
}

double ChElementBeamANCF::Calc_detJ0(double x, double y, double z) {
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;

    return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
}

void ChElementBeamANCF::CalcCoordMatrix(ChMatrixNM<double, 9, 3>& d) {
    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& dA = m_nodes[0]->GetD();
    const ChVector<>& ddA = m_nodes[0]->GetDD();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& dB = m_nodes[1]->GetD();
    const ChVector<>& ddB = m_nodes[1]->GetDD();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& dC = m_nodes[2]->GetD();
    const ChVector<>& ddC = m_nodes[2]->GetDD();

    d(0, 0) = pA.x();
    d(0, 1) = pA.y();
    d(0, 2) = pA.z();
    d(1, 0) = dA.x();
    d(1, 1) = dA.y();
    d(1, 2) = dA.z();
    d(2, 0) = ddA.x();
    d(2, 1) = ddA.y();
    d(2, 2) = ddA.z();

    d(3, 0) = pB.x();
    d(3, 1) = pB.y();
    d(3, 2) = pB.z();
    d(4, 0) = dB.x();
    d(4, 1) = dB.y();
    d(4, 2) = dB.z();
    d(5, 0) = ddB.x();
    d(5, 1) = ddB.y();
    d(5, 2) = ddB.z();

    d(6, 0) = pC.x();
    d(6, 1) = pC.y();
    d(6, 2) = pC.z();
    d(7, 0) = dC.x();
    d(7, 1) = dC.y();
    d(7, 2) = dC.z();
    d(8, 0) = ddC.x();
    d(8, 1) = ddC.y();
    d(8, 2) = ddC.z();
}

void ChElementBeamANCF::CalcCoordDerivMatrix(ChVectorN<double, 27>& dt) {
    const ChVector<>& pA_dt = m_nodes[0]->GetPos_dt();
    const ChVector<>& dA_dt = m_nodes[0]->GetD_dt();
    const ChVector<>& ddA_dt = m_nodes[0]->GetDD_dt();

    const ChVector<>& pB_dt = m_nodes[1]->GetPos_dt();
    const ChVector<>& dB_dt = m_nodes[1]->GetD_dt();
    const ChVector<>& ddB_dt = m_nodes[1]->GetDD_dt();

    const ChVector<>& pC_dt = m_nodes[2]->GetPos_dt();
    const ChVector<>& dC_dt = m_nodes[2]->GetD_dt();
    const ChVector<>& ddC_dt = m_nodes[2]->GetDD_dt();

    dt(0) = pA_dt.x();
    dt(1) = pA_dt.y();
    dt(2) = pA_dt.z();
    dt(3) = dA_dt.x();
    dt(4) = dA_dt.y();
    dt(5) = dA_dt.z();
    dt(6) = ddA_dt.x();
    dt(7) = ddA_dt.y();
    dt(8) = ddA_dt.z();

    dt(9) = pB_dt.x();
    dt(10) = pB_dt.y();
    dt(11) = pB_dt.z();
    dt(12) = dB_dt.x();
    dt(13) = dB_dt.y();
    dt(14) = dB_dt.z();
    dt(15) = ddB_dt.x();
    dt(16) = ddB_dt.y();
    dt(17) = ddB_dt.z();

    dt(18) = pC_dt.x();
    dt(19) = pC_dt.y();
    dt(20) = pC_dt.z();
    dt(21) = dC_dt.x();
    dt(22) = dC_dt.y();
    dt(23) = dC_dt.z();
    dt(24) = ddC_dt.x();
    dt(25) = ddC_dt.y();
    dt(26) = ddC_dt.z();
}

// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------
ChVector<> ChElementBeamANCF::EvaluateBeamSectionStrains() {
    // Element shape function
    ShapeVector N;
    this->ShapeFunctions(N, 0, 0, 0);

    // Determinant of position vector gradient matrix: Initial configuration
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = this->Calc_detJ0(0, 0, 0, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1);
    G1xG2.y() = Nx_d0(0, 2) * Ny_d0(0, 0) - Nx_d0(0, 0) * Ny_d0(0, 2);
    G1xG2.z() = Nx_d0(0, 0) * Ny_d0(0, 1) - Nx_d0(0, 1) * Ny_d0(0, 0);
    G1dotG1 = Nx_d0(0, 0) * Nx_d0(0, 0) + Nx_d0(0, 1) * Nx_d0(0, 1) + Nx_d0(0, 2) * Nx_d0(0, 2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0, 0);
    A1.y() = Nx_d0(0, 1);
    A1.z() = Nx_d0(0, 2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1;
    AA2 = A2;
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(0, 1) * Nz_d0(0, 2) - Nz_d0(0, 1) * Ny_d0(0, 2);
    j0(0, 1) = Ny_d0(0, 2) * Nz_d0(0, 0) - Ny_d0(0, 0) * Nz_d0(0, 2);
    j0(0, 2) = Ny_d0(0, 0) * Nz_d0(0, 1) - Nz_d0(0, 0) * Ny_d0(0, 1);
    j0(1, 0) = Nz_d0(0, 1) * Nx_d0(0, 2) - Nx_d0(0, 1) * Nz_d0(0, 2);
    j0(1, 1) = Nz_d0(0, 2) * Nx_d0(0, 0) - Nx_d0(0, 2) * Nz_d0(0, 0);
    j0(1, 2) = Nz_d0(0, 0) * Nx_d0(0, 1) - Nz_d0(0, 1) * Nx_d0(0, 0);
    j0(2, 0) = Nx_d0(0, 1) * Ny_d0(0, 2) - Ny_d0(0, 1) * Nx_d0(0, 2);
    j0(2, 1) = Ny_d0(0, 0) * Nx_d0(0, 2) - Nx_d0(0, 0) * Ny_d0(0, 2);
    j0(2, 2) = Nx_d0(0, 0) * Ny_d0(0, 1) - Ny_d0(0, 0) * Nx_d0(0, 1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    ChVectorN<double, 9> ddNx = m_ddT * Nx.transpose();
    ChVectorN<double, 9> ddNy = m_ddT * Ny.transpose();
    ChVectorN<double, 9> ddNz = m_ddT * Nz.transpose();

    ChVectorN<double, 9> d0d0Nx = this->m_d0d0T * Nx.transpose();
    ChVectorN<double, 9> d0d0Ny = this->m_d0d0T * Ny.transpose();
    ChVectorN<double, 9> d0d0Nz = this->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * (Nx.dot(ddNx) - Nx.dot(d0d0Nx));
    strain_til(1) = 0.5 * (Ny.dot(ddNy) - Ny.dot(d0d0Ny));
    strain_til(2) = Nx.dot(ddNy) - Nx.dot(d0d0Ny);
    strain_til(3) = 0.5 * (Nz.dot(ddNz) - Nz.dot(d0d0Nz));
    strain_til(4) = Nx.dot(ddNz) - Nx.dot(d0d0Nz);
    strain_til(5) = Ny.dot(ddNz) - Ny.dot(d0d0Nz);

    // For orthotropic material
    ChVectorN<double, 6> strain;

    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    return ChVector<>(strain(0), strain(1), strain(2));
}

void ChElementBeamANCF::EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) {
    //// TODO?
}

void ChElementBeamANCF::EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) {
    ChMatrixNM<double, 9, 3> mD;
    ShapeVector N;
    ShapeVector Nx;
    ShapeVector Ny;

    CalcCoordMatrix(mD);

    // r = Se
    ShapeFunctions(N, eta, 0, 0);
    point = mD.transpose() * N.transpose();

    // Since ANCF does not use rotations, calculate an approximate
    // rotation based off the position vector gradients
    ShapeFunctionsDerivativeX(Nx, eta, 0, 0);
    ShapeFunctionsDerivativeY(Ny, eta, 0, 0);
    ChVector<double> BeamAxisTangent = mD.transpose() * Nx.transpose();
    ChVector<double> CrossSectionY = mD.transpose() * Ny.transpose();

    // Since the position vector gradients are not in general orthogonal,
    // set the Dx direction tangent to the beam axis and
    // compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization, guided by the cross section Y direction
    ChMatrix33<> msect;
    msect.Set_A_Xdir(BeamAxisTangent, CrossSectionY);

    rot = msect.Get_A_quaternion();
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).
void ChElementBeamANCF::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD().eigen();

    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetDD().eigen();

    mD.segment(block_offset + 18, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetDD().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementBeamANCF::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD_dt().eigen();

    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetD_dt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetDD_dt().eigen();

    mD.segment(block_offset + 18, 3) = m_nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[2]->GetD_dt().eigen();
    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetDD_dt().eigen();
}

/// Increment all DOFs using a delta.
void ChElementBeamANCF::LoadableStateIncrement(const unsigned int off_x,
                                               ChState& x_new,
                                               const ChState& x,
                                               const unsigned int off_v,
                                               const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 9, x_new, x, off_v + 9, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 18, x_new, x, off_v + 18, Dv);
}

void ChElementBeamANCF::EvaluateSectionVelNorm(double U, ChVector<>& Result) {
    ShapeVector N;
    ShapeFunctions(N, U, 0, 0);
    for (unsigned int ii = 0; ii < 3; ii++) {
        Result += N(ii * 3) * m_nodes[ii]->GetPos_dt();
        Result += N(ii * 3 + 1) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementBeamANCF::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
        mvars.push_back(&m_nodes[i]->Variables_DD());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U) coordinates of the centerline.
void ChElementBeamANCF::ComputeNF(
    const double U,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ShapeVector N;
    ShapeFunctions(N, U, 0, 0);

    detJ = Calc_detJ0(U, 0, 0);
    detJ *= m_GaussScaling;  //// MIKE: Determine if a different Jacobian needs to be returned

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(0, 3);
    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
    Qi.segment(9, 3) = N(3) * F.segment(0, 3);
    Qi.segment(12, 3) = N(4) * F.segment(0, 3);
    Qi.segment(15, 3) = N(5) * F.segment(0, 3);
    Qi.segment(18, 3) = N(6) * F.segment(0, 3);
    Qi.segment(21, 3) = N(7) * F.segment(0, 3);
    Qi.segment(24, 3) = N(8) * F.segment(0, 3);

    // Compute the generalized force vector for the applied moment
    ChMatrixNM<double, 9, 3> e_bar;
    ChMatrixNM<double, 9, 3> Sxi_D;
    ChMatrixNM<double, 3, 9> Sxi_D_transpose;
    ChMatrix33<double> J_Cxi;
    ChMatrix33<double> J_Cxi_Inv;
    ChVectorN<double, 9> G_A;
    ChVectorN<double, 9> G_B;
    ChVectorN<double, 9> G_C;
    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    CalcCoordMatrix(e_bar);
    Calc_Sxi_D(Sxi_D, U, 0, 0);

    J_Cxi.noalias() = e_bar.transpose() * Sxi_D;
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    Sxi_D_transpose = Sxi_D.transpose();
    G_A = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 0) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 0) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 0);
    G_B = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 1) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 1) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 1);
    G_C = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 2) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 2) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 2);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    //// MIKE Clean-up when slicing becomes available in Eigen 3.4
    for (unsigned int i = 0; i < 9; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
void ChElementBeamANCF::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ShapeVector N;
    ShapeFunctions(N, U, V, W);

    detJ = Calc_detJ0(U, V, W);
    detJ *= m_GaussScaling;  //// MIKE: Determine if a different Jacobian needs to be returned

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(0, 3);
    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
    Qi.segment(9, 3) = N(3) * F.segment(0, 3);
    Qi.segment(12, 3) = N(4) * F.segment(0, 3);
    Qi.segment(15, 3) = N(5) * F.segment(0, 3);
    Qi.segment(18, 3) = N(6) * F.segment(0, 3);
    Qi.segment(21, 3) = N(7) * F.segment(0, 3);
    Qi.segment(24, 3) = N(8) * F.segment(0, 3);

    // Compute the generalized force vector for the applied moment
    ChMatrixNM<double, 9, 3> e_bar;
    ChMatrixNM<double, 9, 3> Sxi_D;
    ChMatrixNM<double, 3, 9> Sxi_D_transpose;
    ChMatrix33<double> J_Cxi;
    ChMatrix33<double> J_Cxi_Inv;
    ChVectorN<double, 9> G_A;
    ChVectorN<double, 9> G_B;
    ChVectorN<double, 9> G_C;
    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    CalcCoordMatrix(e_bar);
    Calc_Sxi_D(Sxi_D, U, V, W);

    J_Cxi.noalias() = e_bar.transpose() * Sxi_D;
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    Sxi_D_transpose = Sxi_D.transpose();
    G_A = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 0) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 0) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 0);
    G_B = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 1) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 1) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 1);
    G_C = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 2) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 2) +
          Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 2);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    //// MIKE Clean-up when slicing becomes available in Eigen 3.4
    for (unsigned int i = 0; i < 9; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate average element density (needed for ChLoaderVolumeGravity).
double ChElementBeamANCF::GetDensity() {
    return GetMaterial()->Get_rho();
}

// Calculate tangent to the centerline at (U) coordinates.
ChVector<> ChElementBeamANCF::ComputeTangent(const double U) {
    ChMatrixNM<double, 9, 3> mD;
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;

    ShapeFunctionsDerivativeX(Nx, U, 0, 0);
    ShapeFunctionsDerivativeY(Ny, U, 0, 0);
    ShapeFunctionsDerivativeZ(Nz, U, 0, 0);

    CalcCoordMatrix(mD);

    ChMatrixNM<double, 1, 3> Nx_d = Nx * mD;

    ChVector<> rd;
    rd[0] = Nx_d(0, 0);
    rd[1] = Nx_d(0, 1);
    rd[2] = Nx_d(0, 2);

    return rd.GetNormalized();
}

// ============================================================================
// Implementation of ChMaterialBeamANCF methods
// ============================================================================

// Construct an isotropic material.
ChMaterialBeamANCF::ChMaterialBeamANCF(double rho,        // material density
                                       double E,          // Young's modulus
                                       double nu,         // Poisson ratio
                                       const double& k1,  // Shear correction factor along beam local y axis
                                       const double& k2   // Shear correction factor along beam local z axis
                                       )
    : m_rho(rho) {
    double G = 0.5 * E / (1 + nu);
    Calc_E_eps(ChVector<>(E), ChVector<>(nu), ChVector<>(G), k1, k2);
    Calc_E_eps_Nu(E, nu, G);
}

// Construct a (possibly) orthotropic material.
ChMaterialBeamANCF::ChMaterialBeamANCF(double rho,            // material density
                                       const ChVector<>& E,   // elasticity moduli (E_x, E_y, E_z)
                                       const ChVector<>& nu,  // Poisson ratios (nu_xy, nu_xz, nu_yz)
                                       const ChVector<>& G,   // shear moduli (G_xy, G_xz, G_yz)
                                       const double& k1,      // Shear correction factor along beam local y axis
                                       const double& k2       // Shear correction factor along beam local z axis
                                       )
    : m_rho(rho) {
    Calc_E_eps(E, nu, G, k1, k2);
    Calc_E_eps_Nu(E, nu, G);
}

// Calculate the matrix of elastic coefficients.
// Always assume that the material could be orthotropic: E_0
void ChMaterialBeamANCF::Calc_E_eps(const ChVector<>& E,
                                    const ChVector<>& nu,
                                    const ChVector<>& G,
                                    double k1,
                                    double k2) {
    m_E_eps.setZero();
    m_E_eps(0, 0) = E.x();
    m_E_eps(1, 1) = E.y();
    m_E_eps(3, 3) = E.z();
    m_E_eps(0, 1) = 0.0;
    m_E_eps(0, 3) = 0.0;
    m_E_eps(1, 0) = 0.0;
    m_E_eps(1, 3) = 0.0;
    m_E_eps(3, 0) = 0.0;
    m_E_eps(3, 1) = 0.0;
    m_E_eps(2, 2) = G.x() * k1;
    m_E_eps(4, 4) = G.y() * k2;  // This works for Z axis loading
    m_E_eps(5, 5) = G.z();
}
void ChMaterialBeamANCF::Calc_E_eps_Nu(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G) {
    double delta = 1.0 - (nu.x() * nu.x()) * E.y() / E.x() - (nu.y() * nu.y()) * E.z() / E.x() -
                   (nu.z() * nu.z()) * E.z() / E.y() - 2.0 * nu.x() * nu.y() * nu.z() * E.z() / E.x();
    m_E_eps_Nu.setZero();
    m_E_eps_Nu(0, 0) = E.x() * ((1.0 - (nu.z() * nu.z()) * E.z() / E.y()) / delta - 1.0);
    m_E_eps_Nu(1, 1) = E.y() * ((1.0 - (nu.y() * nu.y()) * E.z() / E.x()) / delta - 1.0);
    m_E_eps_Nu(3, 3) = E.z() * ((1.0 - (nu.x() * nu.x()) * E.y() / E.x()) / delta - 1.0);
    m_E_eps_Nu(0, 1) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps_Nu(0, 3) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps_Nu(1, 0) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps_Nu(1, 3) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;
    m_E_eps_Nu(3, 0) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps_Nu(3, 1) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;

    m_E_eps_Nu(2, 2) = 0.0;
    m_E_eps_Nu(4, 4) = 0.0;
    m_E_eps_Nu(5, 5) = 0.0;
}

}  // end of namespace fea
}  // end of namespace chrono
