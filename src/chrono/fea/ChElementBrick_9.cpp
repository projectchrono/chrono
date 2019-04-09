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
// Authors: Radu Serban, Bryan Peterson, Antonio Recuero
// =============================================================================
// Brick element with 9 nodes (central node for curvature)
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChElementBrick_9.h"
#include "chrono/fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

ChElementBrick_9::ChElementBrick_9() : m_gravity_on(false) {
    m_nodes.resize(8);

    m_ddT.Reset();
    m_d_dt.Reset();

    m_Alpha_Plast.Reset();
    m_CCPinv_Plast.Reset();

    m_DPVector1.Reset();
    m_DPVector2.Reset();
}

// -----------------------------------------------------------------------------
// Initializations and initial setup
// -----------------------------------------------------------------------------

// Specify the nodes for this element
void ChElementBrick_9::SetNodes(std::shared_ptr<ChNodeFEAxyz> node1,
                                std::shared_ptr<ChNodeFEAxyz> node2,
                                std::shared_ptr<ChNodeFEAxyz> node3,
                                std::shared_ptr<ChNodeFEAxyz> node4,
                                std::shared_ptr<ChNodeFEAxyz> node5,
                                std::shared_ptr<ChNodeFEAxyz> node6,
                                std::shared_ptr<ChNodeFEAxyz> node7,
                                std::shared_ptr<ChNodeFEAxyz> node8,
                                std::shared_ptr<ChNodeFEAcurv> nodeC) {
    assert(node1);
    assert(node2);
    assert(node3);
    assert(node4);
    assert(node5);
    assert(node6);
    assert(node7);
    assert(node8);
    assert(nodeC);

    m_nodes[0] = node1;
    m_nodes[1] = node2;
    m_nodes[2] = node3;
    m_nodes[3] = node4;
    m_nodes[4] = node5;
    m_nodes[5] = node6;
    m_nodes[6] = node7;
    m_nodes[7] = node8;
    m_central_node = nodeC;

    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[4]->Variables());
    mvars.push_back(&m_nodes[5]->Variables());
    mvars.push_back(&m_nodes[6]->Variables());
    mvars.push_back(&m_nodes[7]->Variables());
    mvars.push_back(&m_central_node->Variables());

    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    CalcCoordMatrix(m_d0);
    m_d0d0T.MatrMultiplyT(m_d0, m_d0);
}

// Initial element setup
void ChElementBrick_9::SetupInitial(ChSystem* system) {
    //// TODO any other initializations go here

    m_GaussScaling = (GetDimensions().x() * GetDimensions().y() * GetDimensions().z()) / 8;

    ComputeMassMatrix();
    ComputeGravityForce(system->Get_G_acc());
}

// -----------------------------------------------------------------------------
// Calculation of shape functions and their derivatives
// -----------------------------------------------------------------------------

void ChElementBrick_9::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    double a = GetDimensions().x();
    double b = GetDimensions().y();
    double c = GetDimensions().z();
    N(0) = 0.125 * (1 - x) * (1 - y) * (1 - z);
    N(1) = 0.125 * (1 + x) * (1 - y) * (1 - z);
    N(2) = 0.125 * (1 + x) * (1 + y) * (1 - z);
    N(3) = 0.125 * (1 - x) * (1 + y) * (1 - z);
    N(4) = 0.125 * (1 - x) * (1 - y) * (1 + z);
    N(5) = 0.125 * (1 + x) * (1 - y) * (1 + z);
    N(6) = 0.125 * (1 + x) * (1 + y) * (1 + z);
    N(7) = 0.125 * (1 - x) * (1 + y) * (1 + z);
    N(8) = (a * a) * (-1.0 / 8.0) + (a * a) * (x * x) * (1.0 / 8.0);
    N(9) = (b * b) * (-1.0 / 8.0) + (b * b) * (y * y) * (1.0 / 8.0);
    N(10) = (c * c) * (-1.0 / 8.0) + (c * c) * (z * z) * (1.0 / 8.0);
}

void ChElementBrick_9::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetDimensions().x();
    Nx(0) = 0.25 / a * (-1) * (1 - y) * (1 - z);
    Nx(1) = 0.25 / a * (+1) * (1 - y) * (1 - z);
    Nx(2) = 0.25 / a * (+1) * (1 + y) * (1 - z);
    Nx(3) = 0.25 / a * (-1) * (1 + y) * (1 - z);
    Nx(4) = 0.25 / a * (-1) * (1 - y) * (1 + z);
    Nx(5) = 0.25 / a * (+1) * (1 - y) * (1 + z);
    Nx(6) = 0.25 / a * (+1) * (1 + y) * (1 + z);
    Nx(7) = 0.25 / a * (-1) * (1 + y) * (1 + z);
    //
    Nx(8) = a * x * (1.0 / 2.0);
    Nx(9) = 0;
    Nx(10) = 0;
}

void ChElementBrick_9::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double b = GetDimensions().y();
    Ny(0) = 0.25 / b * (1 - x) * (-1) * (1 - z);
    Ny(1) = 0.25 / b * (1 + x) * (-1) * (1 - z);
    Ny(2) = 0.25 / b * (1 + x) * (+1) * (1 - z);
    Ny(3) = 0.25 / b * (1 - x) * (+1) * (1 - z);
    Ny(4) = 0.25 / b * (1 - x) * (-1) * (1 + z);
    Ny(5) = 0.25 / b * (1 + x) * (-1) * (1 + z);
    Ny(6) = 0.25 / b * (1 + x) * (+1) * (1 + z);
    Ny(7) = 0.25 / b * (1 - x) * (+1) * (1 + z);
    //
    Ny(8) = 0;
    Ny(9) = b * y * (1.0 / 2.0);
    Ny(10) = 0;
}

void ChElementBrick_9::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    double c = GetDimensions().z();
    Nz(0) = 0.25 / c * (1 - x) * (1 - y) * (-1);
    Nz(1) = 0.25 / c * (1 + x) * (1 - y) * (-1);
    Nz(2) = 0.25 / c * (1 + x) * (1 + y) * (-1);
    Nz(3) = 0.25 / c * (1 - x) * (1 + y) * (-1);
    Nz(4) = 0.25 / c * (1 - x) * (1 - y) * (+1);
    Nz(5) = 0.25 / c * (1 + x) * (1 - y) * (+1);
    Nz(6) = 0.25 / c * (1 + x) * (1 + y) * (+1);
    Nz(7) = 0.25 / c * (1 - x) * (1 + y) * (+1);
    //
    Nz(8) = 0;
    Nz(9) = 0;
    Nz(10) = c * z * (1.0 / 2.0);
}

// -----------------------------------------------------------------------------
// Calculation of the mass matrix
// -----------------------------------------------------------------------------

// Private class for quadrature of the mass matrix.
class MyMassBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
  public:
    MyMassBrick9(ChElementBrick_9* element) : m_element(element) {}
    ~MyMassBrick9() {}

  private:
    ChElementBrick_9* m_element;

    virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;
};

void MyMassBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);

    ChMatrixNM<double, 3, 33> S;
    ChMatrix33<> Si;
    Si.Reset();
    
    Si.FillDiag(N(0));
    S.PasteMatrix(Si, 0, 0);
    Si.FillDiag(N(1));
    S.PasteMatrix(Si, 0, 3);
    Si.FillDiag(N(2));
    S.PasteMatrix(Si, 0, 6);
    Si.FillDiag(N(3));
    S.PasteMatrix(Si, 0, 9);
    Si.FillDiag(N(4));
    S.PasteMatrix(Si, 0, 12);
    Si.FillDiag(N(5));
    S.PasteMatrix(Si, 0, 15);
    Si.FillDiag(N(6));
    S.PasteMatrix(Si, 0, 18);
    Si.FillDiag(N(7));
    S.PasteMatrix(Si, 0, 21);
    Si.FillDiag(N(8));
    S.PasteMatrix(Si, 0, 24);
    Si.FillDiag(N(9));
    S.PasteMatrix(Si, 0, 27);
    Si.FillDiag(N(10));
    S.PasteMatrix(Si, 0, 30);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S
    result.MatrTMultiply(S, S);

    // multiply integration weights
    result *= detJ0 * (m_element->m_GaussScaling);
}

// Compute the mass matrix of the element.
void ChElementBrick_9::ComputeMassMatrix() {
    MyMassBrick9 myformula(this);
    m_MassMatrix.Reset();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 33>>(m_MassMatrix,  // result of integration will go there
                                                          myformula,     // formula to integrate
                                                          -1, 1,         // limits in x direction
                                                          -1, 1,         // limits in y direction
                                                          -1, 1,         // limits in z direction
                                                          3              // order of integration
    );

    m_MassMatrix *= m_material->Get_density();
}

// -----------------------------------------------------------------------------
// Calculation of gravitational forces
// -----------------------------------------------------------------------------

// Private class for quadrature of gravitational forces.
class MyGravityBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 1>> {
  public:
    MyGravityBrick9(ChElementBrick_9* element, const ChVector<>& gacc) : m_element(element), m_gacc(gacc) {}
    ~MyGravityBrick9() {}

  private:
    ChElementBrick_9* m_element;
    ChVector<> m_gacc;

    virtual void Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) override;
};

// Evaluate integrand at the specified point
void MyGravityBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    for (int i = 0; i < 11; i++) {
        result(i * 3 + 0, 0) = N(0, i) * m_gacc.x();
        result(i * 3 + 1, 0) = N(0, i) * m_gacc.y();
        result(i * 3 + 2, 0) = N(0, i) * m_gacc.z();
    }

    result *= detJ0 * m_element->m_GaussScaling;
}

// Compute the gravitational forces.
void ChElementBrick_9::ComputeGravityForce(const ChVector<>& g_acc) {
    MyGravityBrick9 myformula(this, g_acc);

    m_GravForce.Reset();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 1>>(m_GravForce,  // result of integration will go there
                                                         myformula,    // formula to integrate
                                                         -1, 1,        // limits in x direction
                                                         -1, 1,        // limits in y direction
                                                         -1, 1,        // limits in z direction
                                                         2             // order of integration
                                                         );

    m_GravForce *= m_material->Get_density();
}

// -----------------------------------------------------------------------------
// Calculation of the internal forces
// -----------------------------------------------------------------------------

// Private class for quadrature of internal forces
class MyForceBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 1>> {
  public:
    MyForceBrick9(ChElementBrick_9* element) : m_element(element) {}
    ~MyForceBrick9() {}

  private:
    ChElementBrick_9* m_element;
    virtual void Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) override;
};

// Evaluate integrand at the specified point
void MyForceBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 11> Nx;
    ChMatrixNM<double, 1, 11> Ny;
    ChMatrixNM<double, 1, 11> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    ChMatrixNM<double, 1, 3> Nx_d;
    ChMatrixNM<double, 1, 3> Ny_d;
    ChMatrixNM<double, 1, 3> Nz_d;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    Nx_d = Nx * m_element->m_d;
    Ny_d = Ny * m_element->m_d;
    Nz_d = Nz * m_element->m_d;

    double detJ = Nx_d(0, 0) * Ny_d(0, 1) * Nz_d(0, 2) + Ny_d(0, 0) * Nz_d(0, 1) * Nx_d(0, 2) +
                  Nz_d(0, 0) * Nx_d(0, 1) * Ny_d(0, 2) - Nx_d(0, 2) * Ny_d(0, 1) * Nz_d(0, 0) -
                  Ny_d(0, 2) * Nz_d(0, 1) * Nx_d(0, 0) - Nz_d(0, 2) * Nx_d(0, 1) * Ny_d(0, 0);

    ChMatrixNM<double, 3, 3> j0;

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
    j0.MatrDivScale(detJ0);

    // Do we need to account for deformed initial configuration in DefF?
    ChMatrixNM<double, 3, 3> DefF;
    DefF(0, 0) = Nx_d(0, 0);
    DefF(1, 0) = Nx_d(0, 1);
    DefF(2, 0) = Nx_d(0, 2);
    DefF(0, 1) = Ny_d(0, 0);
    DefF(1, 1) = Ny_d(0, 1);
    DefF(2, 1) = Ny_d(0, 2);
    DefF(0, 2) = Nz_d(0, 0);
    DefF(1, 2) = Nz_d(0, 1);
    DefF(2, 2) = Nz_d(0, 2);

    double E = m_element->GetMaterial()->Get_E();
    double nu = m_element->GetMaterial()->Get_v();
    double C1 = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    double C2 = m_element->GetMaterial()->Get_G();

    // Matrix of elastic coefficients
    ChMatrixNM<double, 6, 6> E_eps;
    E_eps.Reset();

    E_eps(0, 0) = C1 + 2.0 * C2;
    E_eps(1, 1) = C1 + 2.0 * C2;
    E_eps(3, 3) = C1 + 2.0 * C2;
    E_eps(0, 1) = C1;
    E_eps(0, 3) = C1;
    E_eps(1, 3) = C1;
    E_eps(1, 0) = E_eps(0, 1);
    E_eps(3, 0) = E_eps(0, 3);
    E_eps(3, 1) = E_eps(1, 3);
    E_eps(2, 2) = C2;
    E_eps(4, 4) = C2;
    E_eps(5, 5) = C2;

    // element->SetStrainFormulation(ChElementBrick_9::Hencky);
    // element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager);
    // if (ChElementBrick_9::Hencky == element->GetStrainFormulation) {
    switch (m_element->GetStrainFormulation()) {
        case ChElementBrick_9::GreenLagrange: {
            // ddNx = e^{T}*Nx^{T}*Nx, ddNy = e^{T}*Ny^{T}*Ny, ddNz = e^{T}*Nz^{T}*Nz
            ChMatrixNM<double, 11, 1> ddNx;
            ChMatrixNM<double, 11, 1> ddNy;
            ChMatrixNM<double, 11, 1> ddNz;
            ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
            ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
            ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

            // d0d0Nx = e0^{T}*Nx^{T}*Nx, d0d0Ny = e0^{T}*Ny^{T}*Ny, d0d0Nz = e0^{T}*Nz^{T}*Nz
            ChMatrixNM<double, 11, 1> d0d0Nx;
            ChMatrixNM<double, 11, 1> d0d0Ny;
            ChMatrixNM<double, 11, 1> d0d0Nz;
            d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
            d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
            d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

            // Green-Lagrange strain components
            ChMatrixNM<double, 6, 1> strain;
            strain(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
            strain(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
            strain(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
            strain(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
            strain(4, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
            strain(5, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);

            // Strain derivative component
            ChMatrixNM<double, 6, 33> strainD;
            ChMatrixNM<double, 1, 33> tempB;
            ChMatrixNM<double, 1, 33> tempBB;
            ChMatrixNM<double, 1, 3> tempB3;
            ChMatrixNM<double, 1, 3> tempB31;
            tempB3.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 0, 0);
            tempB3.MatrMultiply(Ny, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 1, 0);
            tempB31.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 2, 0);

            tempB3.MatrMultiply(Nz, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 3, 0);
            tempB31.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 4, 0);
            tempB31.MatrMultiply(Ny, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 5, 0);

            // Add damping strains
            ChMatrixNM<double, 6, 1> DEPS;
            DEPS.Reset();
            for (int ii = 0; ii < 33; ii++) {
                DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
                DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
                DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
                DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
                DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
                DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
            }

            // Add structural damping
            strain += DEPS * m_element->m_Alpha;

            // Internal force calculation
            ChMatrixNM<double, 33, 6> tempC;
            tempC.MatrTMultiply(strainD, E_eps);
            result.MatrMultiply(tempC, strain);
            result.MatrScale(detJ0 * m_element->m_GaussScaling);
        } break;
        case ChElementBrick_9::Hencky: {
            ChMatrixNM<double, 3, 3> Temp33;  ///< Temporary matrix
            ChMatrixNM<double, 3, 3> CCPinv;  ///< Inverse of F^{pT}*F^{p}, where F^{p} is the plastic deformation
            // gradient stemming from multiplicative decomposition
            ChMatrix33<double> BETRI;       ///< Left Cauchy-Green tensor for elastic deformation
            double BETRI_eig[3] = {0.0};    ///< Eigenvalues of spatial stretch tensor
            ChMatrix33<double> BETRI_eigv;  ///< Eigenvectors of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e1;    ///< First principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e2;    ///< Second principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e3;    ///< Third principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 3> MM1;   ///< Matrix from outer product of e1
            ChMatrixNM<double, 3, 3> MM2;   ///< Matrix from outer product of e2
            ChMatrixNM<double, 3, 3> MM3;   ///< Matrix from outer product of e3

            // Obtain inverse of F^{pT}*F^{p} for plastic formulation
            if (m_element->m_Plasticity) {
                CCPinv(0, 0) = m_element->m_CCPinv_Plast(0, m_element->m_InteCounter);
                CCPinv(0, 1) = m_element->m_CCPinv_Plast(1, m_element->m_InteCounter);
                CCPinv(0, 2) = m_element->m_CCPinv_Plast(2, m_element->m_InteCounter);
                CCPinv(1, 0) = m_element->m_CCPinv_Plast(3, m_element->m_InteCounter);
                CCPinv(1, 1) = m_element->m_CCPinv_Plast(4, m_element->m_InteCounter);
                CCPinv(1, 2) = m_element->m_CCPinv_Plast(5, m_element->m_InteCounter);
                CCPinv(2, 0) = m_element->m_CCPinv_Plast(6, m_element->m_InteCounter);
                CCPinv(2, 1) = m_element->m_CCPinv_Plast(7, m_element->m_InteCounter);
                CCPinv(2, 2) = m_element->m_CCPinv_Plast(8, m_element->m_InteCounter);
            } else {
                CCPinv.Reset();
                CCPinv(0, 0) = 1.0;
                CCPinv(1, 1) = 1.0;
                CCPinv(2, 2) = 1.0;
            }

            Temp33.MatrMultiply(DefF, CCPinv);  // Obtain elastic deformation gradient (remove plastic part)
            BETRI.MatrMultiplyT(Temp33, DefF);  // Are we sure?
            BETRI.FastEigen(BETRI_eigv, BETRI_eig);
            for (int i = 0; i < 3; i++) {
                e3(i, 0) = BETRI_eigv(i, 0);
                e1(i, 0) = BETRI_eigv(i, 1);
                e2(i, 0) = BETRI_eigv(i, 2);  // ? Why change in order
            }
            MM1.MatrMultiplyT(e1, e1);
            MM2.MatrMultiplyT(e2, e2);
            MM3.MatrMultiplyT(e3, e3);

            ChMatrixNM<double, 3, 1> LogStrain;
            LogStrain(0, 0) = 0.5 * log(BETRI_eig[1]);
            LogStrain(1, 0) = 0.5 * log(BETRI_eig[2]);
            LogStrain(2, 0) = 0.5 * log(BETRI_eig[0]);

            ChMatrixNM<double, 3, 3> FI;        ///< Inverse of total deformation gradient
            ChMatrixNM<double, 6, 33> strainD;  ///< Jacobian of strains w.r.t. coordinates
            FI = DefF;
            FI.MatrInverse();

            // Obtain Jacobian of strains w.r.t. element coordinates
            m_element->ComputeStrainD_Brick9(strainD, Nx, Ny, Nz, FI, j0);

            // Obtain eigenvalues of Kirchhoff stress tensor
            ChMatrixNM<double, 3, 1> StressK_eig;
            StressK_eig(0, 0) =
                E_eps(0, 0) * LogStrain(0, 0) + E_eps(0, 1) * LogStrain(1, 0) + E_eps(0, 3) * LogStrain(2, 0);
            StressK_eig(1, 0) =
                E_eps(1, 0) * LogStrain(0, 0) + E_eps(1, 1) * LogStrain(1, 0) + E_eps(1, 3) * LogStrain(2, 0);
            StressK_eig(2, 0) =
                E_eps(3, 0) * LogStrain(0, 0) + E_eps(3, 1) * LogStrain(1, 0) + E_eps(3, 3) * LogStrain(2, 0);

            // For plasticity, apply return mapping
            if (m_element->m_Plasticity) {
                double G = E / (2.0 * (1 + nu));
                double K = E / (3.0 * (1 - 2.0 * nu));

                // Volumetric Hencky strain
                double EEVD3 = (LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0)) / 3.0;

                // Deviatoric  Hencky strain
                ChVector<double> EETD;
                EETD.x() = LogStrain(0, 0) - EEVD3;
                EETD.y() = LogStrain(1, 0) - EEVD3;
                EETD.z() = LogStrain(2, 0) - EEVD3;
                // Norm of deviatoric Hencky strain
                double ETDNorm = EETD.Length();
                // Hydrostatic pressure
                double hydroP;
                // Deviatoric stress
                ChVector<double> devStress;
                // Norm of deviatoric stress tensor
                double NormSn;
                // Second invariant of stress tensor
                double J2Rt;
                // Current value of yield function
                double YieldFunc;
                // Flag indicating whether an elastic (0) or plastic (1) step occurs
                int YieldFlag;
                // Variation of flow rate
                double DeltaGamma;
                // Updated value of deviatoric stress tensor
                ChVector<double> devStressUp;
                // Vector of eigenvalues of current logarithmic strain
                ChVector<double> lambda;
                switch (m_element->GetPlasticityFormulation()) {
                    case ChElementBrick_9::J2: {
                        // Hydrostatic pressure , i.e. volumetric stress (from principal stresses)
                        hydroP = (StressK_eig(0, 0) + StressK_eig(1, 0) + StressK_eig(2, 0)) / 3.0;
                        // Deviatoric stress
                        devStress.x() = StressK_eig(0, 0) - hydroP;
                        devStress.y() = StressK_eig(1, 0) - hydroP;
                        devStress.z() = StressK_eig(2, 0) - hydroP;
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        // Second invariant of the stress tensor (J2)
                        J2Rt = NormSn / sqrt(2.0);

                        // Trial (elastic) deviatoric stress for yield function
                        double qtrial = sqrt(3.0) * J2Rt;

                        // Evaluation of J2 yield function
                        YieldFunc = qtrial - (m_element->m_YieldStress +
                                              m_element->m_HardeningSlope *
                                                  m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));

                        // Set Yield flag to zero (within elastic range)
                        YieldFlag = 0;

                        // If yield function reveals plasticity, apply return mapping algorithm
                        if (YieldFunc > 0.0) {
                            YieldFlag = 1;

                            // Step variation of the plastic flow (rate)
                            double DeltaGamma = YieldFunc / (3.0 * G + m_element->m_HardeningSlope);

                            // Perform return mapping on deviatoric stress tensor (Up for Update)
                            if (qtrial != 0.0) {
                                devStressUp = (1.0 - G * DeltaGamma * 3.0 / qtrial) * devStress;
                            } else {
                                devStressUp = devStress;
                            }

                            // Update stress tensor
                            StressK_eig(0, 0) = devStressUp.x() + hydroP;
                            StressK_eig(1, 0) = devStressUp.y() + hydroP;
                            StressK_eig(2, 0) = devStressUp.z() + hydroP;

                            // Update logarithmic strains
                            LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + EEVD3;
                            LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + EEVD3;
                            LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + EEVD3;

                            // Obtain eigenvalues current logarithmic strains
                            lambda.x() = exp(2.0 * LogStrain(0, 0));
                            lambda.y() = exp(2.0 * LogStrain(1, 0));
                            lambda.z() = exp(2.0 * LogStrain(2, 0));

                            ChMatrixNM<double, 3, 3> BEUP;  ///< Updated elastic left Cauchy strain tensor
                            MM1.MatrScale(lambda.x());
                            MM2.MatrScale(lambda.y());
                            MM3.MatrScale(lambda.z());
                            BEUP = MM1 + MM2 + MM3;

                            // MM1, MM2, and MM3 are outputs of the return mapping alg. so must be re-updated
                            MM1.MatrScale(1 / lambda.x());
                            MM2.MatrScale(1 / lambda.y());
                            MM3.MatrScale(1 / lambda.z());

                            // Keep track of plastic variable alpha for each integration point
                            m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) =
                                m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + DeltaGamma;
                            Temp33.MatrMultiply(FI, BEUP);
                            CCPinv.MatrMultiplyT(Temp33, FI);

                            // Store plastic deformation tensor for each iteration
                            m_element->m_CCPinv_Plast(0, m_element->m_InteCounter) = CCPinv(0, 0);
                            m_element->m_CCPinv_Plast(1, m_element->m_InteCounter) = CCPinv(0, 1);
                            m_element->m_CCPinv_Plast(2, m_element->m_InteCounter) = CCPinv(0, 2);
                            m_element->m_CCPinv_Plast(3, m_element->m_InteCounter) = CCPinv(1, 0);
                            m_element->m_CCPinv_Plast(4, m_element->m_InteCounter) = CCPinv(1, 1);
                            m_element->m_CCPinv_Plast(5, m_element->m_InteCounter) = CCPinv(1, 2);
                            m_element->m_CCPinv_Plast(6, m_element->m_InteCounter) = CCPinv(2, 0);
                            m_element->m_CCPinv_Plast(7, m_element->m_InteCounter) = CCPinv(2, 1);
                            m_element->m_CCPinv_Plast(8, m_element->m_InteCounter) = CCPinv(2, 2);
                        }
                    } break;

                    case ChElementBrick_9::DruckerPrager: {
                        double EDNInv;  // Inverse of norm of deviatoric Hencky strain

                        // Evaluate norm of deviatoric Hencky strain
                        if (ETDNorm != 0.0) {
                            EDNInv = 1.0 / ETDNorm;
                        } else {
                            EDNInv = 0.0;
                        }

                        ChMatrixNM<double, 6, 1> UniDev;
                        UniDev(0, 0) = EETD.x() * EDNInv;
                        UniDev(1, 0) = EETD.y() * EDNInv;
                        UniDev(3, 0) = EETD.z() * EDNInv;
                        UniDev(2, 0) = 0.0;
                        UniDev(4, 0) = 0.0;
                        UniDev(5, 0) = 0.0;

                        double EETV = LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0);
                        hydroP = K * EETV;
                        devStress.x() = 2.0 * G * (LogStrain(0, 0) - EEVD3);
                        devStress.y() = 2.0 * G * (LogStrain(1, 0) - EEVD3);
                        devStress.z() = 2.0 * G * (LogStrain(2, 0) - EEVD3);
                        // Euclidean natural norm of the second-order tensor
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        J2Rt = NormSn / sqrt(2.0);

                        double phi = m_element->m_FrictionAngle * CH_C_DEG_TO_RAD;    // Friction angle
                        double phi2 = m_element->m_DilatancyAngle * CH_C_DEG_TO_RAD;  // Dilatancy angle

                        double eta;  // Coefficient multiplying hydros. pressure in yield function - function of
                                     // internal friction
                        double gsi;  // Coefficient multiplying 'current' cohesion value in yield function
                        double etab;

                        if (m_element->m_DPHardening == 1) {  // Tension corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 + tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 2) {  // Compression corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 - tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 3) {  // Shear corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = 1.0 / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        }
                        double alpha1;
                        double beta1;
                        if (etab == 0) {
                            etab = 0.000001;
                        }
                        alpha1 = gsi / etab;
                        if (eta == 0) {
                            eta = 0.000001;
                        }
                        beta1 = gsi / eta;

                        // Yield function at trial stage
                        YieldFunc =
                            J2Rt + eta * hydroP -
                            gsi * (m_element->m_YieldStress +
                                   m_element->m_HardeningSlope * m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                        double SQRJ2T = J2Rt;
                        double PT = hydroP;

                        YieldFlag = 0;
                        // If YieldFunc > 0.0 there is need for return mapping
                        if (YieldFunc > 0.0) {
                            YieldFlag = 1;

                            // Initialize for Newton-Raphson
                            double DGamma = 0.0;
                            double Yfunc1 = YieldFunc;  // Initially recalculated

                            for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                // Compute Newton residual
                                double DDGamma =
                                    Yfunc1 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                DGamma = DGamma + DDGamma;
                                double EpBar = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DGamma;
                                SQRJ2T = SQRJ2T - G * DGamma;
                                double P = PT - K * etab * DGamma;
                                Yfunc1 = SQRJ2T + eta * P -
                                         gsi * (m_element->m_YieldStress + m_element->m_HardeningSlope * EpBar);
                                if (Yfunc1 < m_element->GetDPYieldTol())
                                    break;

                                if (ii == m_element->GetDPIterationNo() - 1)
                                    throw ChException(
                                        "Maximum number of iterations reached in Drucker-Prager Newton-Raphson "
                                        "algorithm");
                            }
                            DeltaGamma = DGamma;
                            double Check_DP_Cone = J2Rt - G * DGamma;

                            if (Check_DP_Cone < 0.0) {  // Cone return mapping
                                double Rtrial = hydroP -
                                                beta1 * (m_element->m_YieldStress +
                                                         m_element->m_HardeningSlope *
                                                             m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                                DeltaGamma = Rtrial / (K + alpha1 * beta1 * m_element->m_HardeningSlope);

                                // Update plastic alpha parameter
                                m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) =
                                    m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + alpha1 * DeltaGamma;

                                // Stress update
                                StressK_eig(0, 0) = hydroP - K * DeltaGamma;
                                StressK_eig(1, 0) = hydroP - K * DeltaGamma;
                                StressK_eig(2, 0) = hydroP - K * DeltaGamma;

                                // Calculate update elastic logarithmic strain
                                LogStrain = StressK_eig;
                                LogStrain.MatrScale(1 / (3.0 * K));
                            } else {
                                // Update plastic alpha parameter
                                m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) =
                                    m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DeltaGamma;

                                // Deviatoric stress
                                devStressUp = (1.0 - G * DeltaGamma / J2Rt) * devStress;
                                // Hydrostatic stress
                                double hydroPUp = hydroP - K * etab * DeltaGamma;
                                // Stress update
                                StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                // Calculate update elastic logarithmic strain
                                LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                            }
                            // Update eigenvalues of strain tensor
                            lambda.x() = exp(2.0 * LogStrain(0, 0));
                            lambda.y() = exp(2.0 * LogStrain(1, 0));
                            lambda.z() = exp(2.0 * LogStrain(2, 0));
                            // Updated left Cauchy-Green strain tensor
                            ChMatrixNM<double, 3, 3> BEUP;
                            MM1.MatrScale(lambda.x());
                            MM2.MatrScale(lambda.y());
                            MM3.MatrScale(lambda.z());
                            BEUP = MM1 + MM2 + MM3;
                            // Obtain plastic deformation tensor
                            MM1.MatrScale(1 / lambda.x());
                            MM2.MatrScale(1 / lambda.y());
                            MM3.MatrScale(1 / lambda.z());
                            Temp33.MatrMultiply(FI, BEUP);
                            CCPinv.MatrMultiplyT(Temp33, FI);

                            // Store plastic deformation tensor for each iteration
                            m_element->m_CCPinv_Plast(0, m_element->m_InteCounter) = CCPinv(0, 0);
                            m_element->m_CCPinv_Plast(1, m_element->m_InteCounter) = CCPinv(0, 1);
                            m_element->m_CCPinv_Plast(2, m_element->m_InteCounter) = CCPinv(0, 2);
                            m_element->m_CCPinv_Plast(3, m_element->m_InteCounter) = CCPinv(1, 0);
                            m_element->m_CCPinv_Plast(4, m_element->m_InteCounter) = CCPinv(1, 1);
                            m_element->m_CCPinv_Plast(5, m_element->m_InteCounter) = CCPinv(1, 2);
                            m_element->m_CCPinv_Plast(6, m_element->m_InteCounter) = CCPinv(2, 0);
                            m_element->m_CCPinv_Plast(7, m_element->m_InteCounter) = CCPinv(2, 1);
                            m_element->m_CCPinv_Plast(8, m_element->m_InteCounter) = CCPinv(2, 2);
                        }
                    } break;

                    case ChElementBrick_9::DruckerPrager_Cap: {
                        // Hydrostatic pressure (Cap)
                        double hydroPt;
                        // Current value of yield function (Cap)
                        double YieldFunc_Cap;
                        int FlagYieldType;  //(DP_Cap)
                        int PlasticCount;   //(DP_Cap)
                        double EDNInv;      // Inverse of norm of deviatoric Hencky strain

                        //// Evaluate norm of deviatoric Hencky strain
                        if (ETDNorm != 0.0) {
                            EDNInv = 1.0 / ETDNorm;
                        } else {
                            EDNInv = 0.0;
                        }

                        ChMatrixNM<double, 6, 1> UniDev;
                        UniDev(2, 0) = 0.0;
                        UniDev(4, 0) = 0.0;
                        UniDev(5, 0) = 0.0;
                        UniDev(0, 0) = EETD.x() * EDNInv;
                        UniDev(1, 0) = EETD.y() * EDNInv;
                        UniDev(3, 0) = EETD.z() * EDNInv;

                        double EETV = LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0);
                        hydroP = K * EETV;
                        devStress.x() = 2.0 * G * (LogStrain(0, 0) - EEVD3);
                        devStress.y() = 2.0 * G * (LogStrain(1, 0) - EEVD3);
                        devStress.z() = 2.0 * G * (LogStrain(2, 0) - EEVD3);
                        // Euclidean natural norm of the second-order tensor
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        J2Rt = NormSn / sqrt(2.0);

                        double phi = m_element->m_FrictionAngle * CH_C_DEG_TO_RAD;    // Friction angle
                        double phi2 = m_element->m_DilatancyAngle * CH_C_DEG_TO_RAD;  // Dilatancy angle

                        double eta;  // Coefficient multiplying hydros. pressure in yield function - function of
                                     // internal friction
                        double gsi;  // Coefficient multiplying 'current' cohesion value in yield function
                        double etab;

                        if (m_element->m_DPHardening == 1) {  // Tension corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 + tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 2) {  // Compression corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 - tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 3) {  // Shear corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = 1.0 / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        }
                        double alpha1;
                        double beta1;
                        if (etab == 0) {
                            etab = 0.000001;
                        }
                        alpha1 = gsi / etab;
                        if (eta == 0) {
                            eta = 0.000001;
                        }
                        beta1 = gsi / eta;

                        //-------------------------------//
                        // Yield function at trial stage //
                        //-------------------------------//

                        // DP yield function
                        YieldFunc =
                            J2Rt + eta * hydroP -
                            gsi * (m_element->m_YieldStress +
                                   m_element->m_HardeningSlope * m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));

                        // CAP yield function
                        hydroPt = beta1 * m_element->m_YieldStress;
                        double CapM = sqrt(3.0) * eta;

                        double MeanEffP;
                        double Hi;

                        // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                        m_element->ComputeHardening_a(
                            MeanEffP, Hi, m_element->m_Alpha_Plast(m_element->m_InteCounter, 0), m_element->m_DPVector1,
                            m_element->m_DPVector2, m_element->m_DPVector_size);

                        YieldFunc_Cap = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                            (hydroP - hydroPt + MeanEffP) * (hydroP - hydroPt + MeanEffP) +
                                        (sqrt(3.0) * J2Rt / CapM) * (sqrt(3.0) * J2Rt / CapM) - MeanEffP * MeanEffP;

                        double SQRJ2T = J2Rt;
                        double PT = hydroP;
                        double EPBARN = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0);  // alphUp;

                        YieldFlag = 0;
                        FlagYieldType = 0;

                        if (YieldFunc > 0.0 || (YieldFunc_Cap > 0.0 && hydroP < hydroPt - MeanEffP)) {
                            YieldFlag = 1;
                            PlasticCount = 1;
                            if (YieldFunc > 0.0) {  // DP smoothed surface return mapping
                                // Initialize for Newton-Raphson
                                double DGamma = 0.0;
                                double Yfunc1 = YieldFunc;  // Initially recalculated

                                for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                    // Compute Newton residual
                                    double DDGamma =
                                        Yfunc1 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                    DGamma = DGamma + DDGamma;
                                    SQRJ2T = SQRJ2T - G * DGamma;
                                    double P = PT - K * etab * DGamma;
                                    double EpBar = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DGamma;
                                    Yfunc1 = SQRJ2T + eta * P -
                                             gsi * (m_element->m_YieldStress + m_element->m_HardeningSlope * EpBar);
                                    if (std::abs(Yfunc1) < m_element->GetDPYieldTol())
                                        break;

                                    if (ii == m_element->GetDPIterationNo() - 1)
                                        throw ChException(
                                            "Maximum number of iterations reached in Drucker-Prager Surface "
                                            "Newton-Raphson algorithm");
                                }
                                DeltaGamma = DGamma;
                                double Check_DP_Cone = J2Rt - G * DGamma;

                                if (Check_DP_Cone < 0.0) {  // Cone return mapping
                                    FlagYieldType = 2;
                                    double Rtrial = hydroP -
                                                    beta1 * (m_element->m_YieldStress +
                                                             m_element->m_HardeningSlope *
                                                                 m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                                    DeltaGamma = Rtrial / (K + alpha1 * beta1 * m_element->m_HardeningSlope);

                                    // Stress update
                                    StressK_eig(0, 0) = hydroP - K * DeltaGamma;
                                    StressK_eig(1, 0) = hydroP - K * DeltaGamma;
                                    StressK_eig(2, 0) = hydroP - K * DeltaGamma;

                                    // Calculate update elastic logarithmic strain
                                    LogStrain = StressK_eig;
                                    LogStrain.MatrScale(1 / (3.0 * K));
                                } else {
                                    if (YieldFunc_Cap <= 0.0 ||
                                        ((hydroP - K * etab * DeltaGamma) >= hydroPt - MeanEffP)) {
                                        FlagYieldType = 1;
                                        //// Update plastic alpha parameter
                                        // m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) =
                                        //	m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DeltaGamma;
                                        // Deviatoric stress
                                        devStressUp = (1.0 - G * DeltaGamma / J2Rt) * devStress;
                                        // Hydrostatic stress
                                        double hydroPUp = hydroP - K * etab * DeltaGamma;
                                        // Stress update
                                        StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                        StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                        StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                        // Calculate update elastic logarithmic strain
                                        LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                        LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                        LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    }
                                }  // end of // Cone return mapping
                            }      // end of // DP smoothed surface return mapping

                            if (YieldFunc_Cap > 0.0 && FlagYieldType == 0) {  // Cap return mapping
                                // Initialize of Newton raphson for Cap surface
                                double DGamma = 0.0;
                                double EPBAR = EPBARN;
                                double SQRJ2 = SQRJ2T;
                                double P = PT;
                                ChVector<double> devS = devStress;

                                // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                              m_element->m_DPVector2, m_element->m_DPVector_size);
                                // Evaluate initial 2 residual vectors
                                double Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                   (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                               (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                               MeanEffP * MeanEffP;
                                double Res02 = EPBAR - EPBARN +
                                               DGamma * 2.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                                   (P - hydroPt + MeanEffP);

                                double Acof;
                                double A11;
                                double A12;
                                double A21;
                                double A22;
                                double B11;
                                double B12;
                                double B21;
                                double B22;
                                double DDGamma;
                                double DALPHA;
                                for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                    Acof = P - hydroPt + MeanEffP;
                                    A11 = -12.0 * G / (CapM * CapM + 6.0 * G * DGamma) * (sqrt(3.0) * SQRJ2 / CapM) *
                                          (sqrt(3.0) * SQRJ2 / CapM);
                                    A12 = 2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi) -
                                          2.0 * MeanEffP * Hi;
                                    A21 = 2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta);
                                    A22 = 1.0 +
                                          2.0 * DGamma / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi);

                                    if (A11 * A22 - A12 * A21 == 0.0) {
                                        printf("Cap Return Singular Matrix!!\n");
                                        exit(1);
                                    }

                                    B11 = A22 / (A11 * A22 - A12 * A21);
                                    B12 = -A12 / (A11 * A22 - A12 * A21);
                                    B21 = -A21 / (A11 * A22 - A12 * A21);
                                    B22 = A11 / (A11 * A22 - A12 * A21);

                                    DDGamma = -B11 * Res01 - B12 * Res02;
                                    DALPHA = -B21 * Res01 - B22 * Res02;

                                    DGamma = DGamma + DDGamma;  //! Update Newton increment in terms of DGamma
                                    EPBAR = EPBAR +
                                            DALPHA;  //! Update Newton increment in terms of alpha(hardening parameters)
                                    P = PT + K * (EPBAR - EPBARN);
                                    SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * SQRJ2T;
                                    devS.x() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.x();
                                    devS.y() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.y();
                                    devS.z() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.z();

                                    // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                    m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                  m_element->m_DPVector2, m_element->m_DPVector_size);

                                    Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                            (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                            MeanEffP * MeanEffP;
                                    Res02 = EPBAR - EPBARN +
                                            DGamma * 2.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                                (P - hydroPt + MeanEffP);

                                    if (std::abs(Hi) < m_element->GetDPYieldTol()) {
                                        if (sqrt(Res01 * Res01) < m_element->GetDPYieldTol() &&
                                            sqrt(Res02 * Res02) < m_element->GetDPYieldTol())

                                            break;
                                    } else {
                                        if (sqrt(Res01 * Res01) / std::abs(Hi) < m_element->GetDPYieldTol() &&
                                            sqrt(Res02 * Res02) / std::abs(Hi) < m_element->GetDPYieldTol())

                                            break;
                                    }

                                    if (ii == m_element->GetDPIterationNo() - 1)
                                        throw ChException(
                                            "Maximum number of iterations reached in Drucker-Prager Cap surface "
                                            "Newton-Raphson algorithm");
                                }  // End of Newton raphson

                                DeltaGamma = DGamma;

                                if (P > hydroPt - MeanEffP) {  // Transition return mapping
                                    FlagYieldType = 4;
                                    // Initialize for newton raphson
                                    double DGamma_B = 0.0;
                                    double AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B);
                                    double DGamma_A =
                                        (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                    EPBAR = EPBARN - etab * DGamma_A;
                                    SQRJ2 = SQRJ2T;
                                    P = PT;
                                    devS = devStress;
                                    // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                    m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                  m_element->m_DPVector2, m_element->m_DPVector_size);
                                    // initial resid vector
                                    Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                            (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                            MeanEffP * MeanEffP;
                                    double DGamma_B1;
                                    double Res11;
                                    double DRes01;
                                    for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                        // TransNewtonDifference

                                        //** perturbation for DGAMA_B1
                                        DGamma_B1 = DGamma_B + 1e-10;
                                        AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B1);
                                        DGamma_A = (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                        EPBAR = EPBARN - etab * DGamma_A;
                                        // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                        m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                      m_element->m_DPVector2,
                                                                      m_element->m_DPVector_size);
                                        // update stresses
                                        P = PT - K * etab * DGamma_A;
                                        SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B1) *
                                                (1.0 - G * DGamma_A / SQRJ2T) * SQRJ2T;
                                        // Check yield functions
                                        Res11 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                    (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                                (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                                MeanEffP * MeanEffP;
                                        DRes01 = (Res11 - Res01) / 1e-10;

                                        if (DRes01 == 0.0) {
                                            printf("Singular for Transition DP/CAP!!\n");
                                            exit(1);
                                        }

                                        // update 3 parameters (plastic and hardening parameters)
                                        DGamma_B = DGamma_B - 1.0 / DRes01 * Res01;
                                        AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B);
                                        DGamma_A = (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                        EPBAR = EPBARN - etab * DGamma_A;  // plastic strain
                                        // Update yield stress and the Hi(slop) at plastic strain from table
                                        m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                      m_element->m_DPVector2,
                                                                      m_element->m_DPVector_size);
                                        // Update stresses
                                        P = PT - K * etab * DGamma_A;
                                        SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                (1.0 - G * DGamma_A / SQRJ2T) * SQRJ2T;
                                        devS.x() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.x();
                                        devS.y() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.y();
                                        devS.z() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.z();
                                        // Check yield functions
                                        Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                    (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                                (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                                MeanEffP * MeanEffP;
                                        if (std::abs(Hi) < m_element->GetDPYieldTol()) {
                                            if (std::abs(Res01) < m_element->GetDPYieldTol())
                                                break;
                                        } else {
                                            if (std::abs(Res01) / std::abs(Hi) < m_element->GetDPYieldTol())
                                                break;
                                        }
                                        if (ii == m_element->GetDPIterationNo() - 1)
                                            throw ChException(
                                                "Hit the max iteration for Transient Cap_surf and DP_surf return "
                                                "mapping");
                                    }  // End of Newton raphson

                                    DeltaGamma = DGamma_A + DGamma_B;

                                    m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) = EPBAR;  //??????

                                    // Update stress
                                    devStressUp = devS;
                                    //(Hydrostatic)
                                    double hydroPUp = P;
                                    // Stress update first
                                    StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                    StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                    StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                    // calculate updated elastic logarithmic Strain
                                    LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                } else {
                                    FlagYieldType = 3;

                                    m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) = EPBAR;  //??????

                                    // Update stress
                                    devStressUp = devS;
                                    //(Hydrostatic)
                                    double hydroPUp = P;
                                    // Stress update first
                                    StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                    StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                    StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                    // calculate updated elastic logarithmic Strain
                                    LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                }  // end if of transition return mapping
                            }      // end if Cap return mapping
                            // Update eigenvalues of strain tensor
                            lambda.x() = exp(2.0 * LogStrain(0, 0));
                            lambda.y() = exp(2.0 * LogStrain(1, 0));
                            lambda.z() = exp(2.0 * LogStrain(2, 0));
                            // Updated left Cauchy-Green strain tensor
                            ChMatrixNM<double, 3, 3> BEUP;
                            MM1.MatrScale(lambda.x());
                            MM2.MatrScale(lambda.y());
                            MM3.MatrScale(lambda.z());
                            BEUP = MM1 + MM2 + MM3;
                            Temp33.MatrMultiply(FI, BEUP);
                            CCPinv.MatrMultiplyT(Temp33, FI);

                            // Obtain plastic deformation tensor
                            MM1.MatrScale(1 / lambda.x());
                            MM2.MatrScale(1 / lambda.y());
                            MM3.MatrScale(1 / lambda.z());

                            // Store plastic deformation tensor for each iteration
                            m_element->m_CCPinv_Plast(0, m_element->m_InteCounter) = CCPinv(0, 0);
                            m_element->m_CCPinv_Plast(1, m_element->m_InteCounter) = CCPinv(0, 1);
                            m_element->m_CCPinv_Plast(2, m_element->m_InteCounter) = CCPinv(0, 2);
                            m_element->m_CCPinv_Plast(3, m_element->m_InteCounter) = CCPinv(1, 0);
                            m_element->m_CCPinv_Plast(4, m_element->m_InteCounter) = CCPinv(1, 1);
                            m_element->m_CCPinv_Plast(5, m_element->m_InteCounter) = CCPinv(1, 2);
                            m_element->m_CCPinv_Plast(6, m_element->m_InteCounter) = CCPinv(2, 0);
                            m_element->m_CCPinv_Plast(7, m_element->m_InteCounter) = CCPinv(2, 1);
                            m_element->m_CCPinv_Plast(8, m_element->m_InteCounter) = CCPinv(2, 2);

                        }  // end if of Yield criteria

                        // if (FlagYieldType !=0 )
                        // GetLog() << "FlagYieldType=" << FlagYieldType <<"\n";

                    } break;  // end of Case DruckerPrager_Cap
                }
            }

            // Obtain stress tensor from MMX matrices - either elastic or plastic
            ChMatrixNM<double, 3, 3> StressK;
            MM1.MatrScale(StressK_eig(0, 0));
            MM2.MatrScale(StressK_eig(1, 0));
            MM3.MatrScale(StressK_eig(2, 0));
            StressK = MM1 + MM2 + MM3;

            // Influence of damping
            ChMatrixNM<double, 6, 1> DEPS;
            DEPS.Reset();
            for (int ii = 0; ii < 33; ii++) {
                DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
                DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
                DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
                DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
                DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
                DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
            }

            // Obtain stress from damping forces
            ChMatrixNM<double, 6, 1> Stress_damp;
            // Add structural damping
            Stress_damp.MatrMultiply(E_eps, DEPS);
            Stress_damp.MatrScale(m_element->m_Alpha);

            // Final six stress components
            ChMatrixNM<double, 6, 1> Stress;
            Stress(0, 0) = StressK(0, 0) + Stress_damp(0, 0);
            Stress(1, 0) = StressK(1, 1) + Stress_damp(1, 0);
            Stress(2, 0) = 0.5 * (StressK(1, 0) + StressK(0, 1)) + Stress_damp(2, 0);
            Stress(3, 0) = StressK(2, 2) + Stress_damp(3, 0);
            Stress(4, 0) = 0.5 * (StressK(2, 0) + StressK(0, 2)) + Stress_damp(4, 0);
            Stress(5, 0) = 0.5 * (StressK(2, 1) + StressK(1, 2)) + Stress_damp(5, 0);

            // Obtain generalized elato-(plastic) forces
            result.MatrTMultiply(strainD, Stress);
            result.MatrScale(detJ * m_element->m_GaussScaling);
            m_element->m_InteCounter++;
        } break;
    }
}

// Compute internal forces and load them in the Fi vector.
void ChElementBrick_9::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT.MatrMultiplyT(m_d, m_d);

    Fi.Reset();
    // Set plastic counter to zero. This runs for each integration point
    m_InteCounter = 0;
    MyForceBrick9 formula(this);
    ChMatrixNM<double, 33, 1> result;
    result.Reset();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 1>>(result,   // result of integration
                                                         formula,  // integrand formula
                                                         -1, 1,    // x limits
                                                         -1, 1,    // y limits
                                                         -1, 1,    // z limits
                                                         2         // order of integration
                                                         );
    Fi -= result;
    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------
// Calculation of the Jacobian of internal forces
// -----------------------------------------------------------------------------

// Private class for quadrature of the Jacobian of internal forces
class MyJacobianBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
  public:
    MyJacobianBrick9(ChElementBrick_9* element,  // Associated element
                     double Kfactor,             // Scaling coefficient for stiffness component
                     double Rfactor              // Scaling coefficient for damping component
                     )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {}

  private:
    ChElementBrick_9* m_element;
    double m_Kfactor;
    double m_Rfactor;
    ChMatrixNM<double, 33, 33> m_KTE1;
    ChMatrixNM<double, 33, 33> m_KTE2;

    virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;
};

// Evaluate integrand at the specified point
void MyJacobianBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 11> Nx;
    ChMatrixNM<double, 1, 11> Ny;
    ChMatrixNM<double, 1, 11> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    ChMatrixNM<double, 1, 3> Nx_d;
    ChMatrixNM<double, 1, 3> Ny_d;
    ChMatrixNM<double, 1, 3> Nz_d;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    Nx_d = Nx * m_element->m_d;
    Ny_d = Ny * m_element->m_d;
    Nz_d = Nz * m_element->m_d;

    double detJ = Nx_d(0, 0) * Ny_d(0, 1) * Nz_d(0, 2) + Ny_d(0, 0) * Nz_d(0, 1) * Nx_d(0, 2) +
                  Nz_d(0, 0) * Nx_d(0, 1) * Ny_d(0, 2) - Nx_d(0, 2) * Ny_d(0, 1) * Nz_d(0, 0) -
                  Ny_d(0, 2) * Nz_d(0, 1) * Nx_d(0, 0) - Nz_d(0, 2) * Nx_d(0, 1) * Ny_d(0, 0);

    ChMatrixNM<double, 3, 3> j0;
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
    j0.MatrDivScale(detJ0);

    // Current deformation gradient matrix
    ChMatrixNM<double, 3, 3> DefF;

    DefF(0, 0) = Nx_d(0, 0);
    DefF(1, 0) = Nx_d(0, 1);
    DefF(2, 0) = Nx_d(0, 2);
    DefF(0, 1) = Ny_d(0, 0);
    DefF(1, 1) = Ny_d(0, 1);
    DefF(2, 1) = Ny_d(0, 2);
    DefF(0, 2) = Nz_d(0, 0);
    DefF(1, 2) = Nz_d(0, 1);
    DefF(2, 2) = Nz_d(0, 2);

    double E = m_element->GetMaterial()->Get_E();
    double nu = m_element->GetMaterial()->Get_v();
    double C1 = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    double C2 = m_element->GetMaterial()->Get_G();

    // Matrix of elastic coefficients
    ChMatrixNM<double, 6, 6> E_eps;
    E_eps.Reset();

    E_eps(0, 0) = C1 + 2.0 * C2;
    E_eps(1, 1) = C1 + 2.0 * C2;
    E_eps(3, 3) = C1 + 2.0 * C2;
    E_eps(0, 1) = C1;
    E_eps(0, 3) = C1;
    E_eps(1, 3) = C1;
    E_eps(1, 0) = E_eps(0, 1);
    E_eps(3, 0) = E_eps(0, 3);
    E_eps(3, 1) = E_eps(1, 3);
    E_eps(2, 2) = C2;
    E_eps(4, 4) = C2;
    E_eps(5, 5) = C2;
    switch (m_element->GetStrainFormulation()) {
        case ChElementBrick_9::GreenLagrange: {  // ddNx = e^{T}*Nx^{T}*Nx, ddNy = e^{T}*Ny^{T}*Ny, ddNz =
                                                 // e^{T}*Nz^{T}*Nz
            ChMatrixNM<double, 11, 1> ddNx;
            ChMatrixNM<double, 11, 1> ddNy;
            ChMatrixNM<double, 11, 1> ddNz;
            ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
            ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
            ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

            // d0d0Nx = e0^{T}*Nx^{T}*Nx, d0d0Ny = e0^{T}*Ny^{T}*Ny, d0d0Nz = e0^{T}*Nz^{T}*Nz
            ChMatrixNM<double, 11, 1> d0d0Nx;
            ChMatrixNM<double, 11, 1> d0d0Ny;
            ChMatrixNM<double, 11, 1> d0d0Nz;
            d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
            d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
            d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

            // Green-Lagrange strain components
            ChMatrixNM<double, 6, 1> strain;
            strain(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
            strain(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
            strain(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
            strain(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
            strain(4, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
            strain(5, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);

            // Strain derivative component
            ChMatrixNM<double, 6, 33> strainD;
            ChMatrixNM<double, 1, 33> tempB;
            ChMatrixNM<double, 1, 33> tempBB;
            ChMatrixNM<double, 1, 3> tempB3;
            ChMatrixNM<double, 1, 3> tempB31;
            tempB3.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 0, 0);
            tempB3.MatrMultiply(Ny, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 1, 0);
            tempB31.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 2, 0);

            tempB3.MatrMultiply(Nz, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 3, 0);
            tempB31.MatrMultiply(Nx, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 4, 0);
            tempB31.MatrMultiply(Ny, m_element->m_d);
            for (int i = 0; i < 11; i++) {
                for (int j = 0; j < 3; j++) {
                    tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
                }
            }
            strainD.PasteClippedMatrix(tempB, 0, 0, 1, 33, 5, 0);

            // Gd is the total deformation gradient differentiated by the coordinates (9 components diff. by 33
            // coordinates)
            ChMatrixNM<double, 9, 33> Gd;
            Gd.Reset();

            for (int ii = 0; ii < 11; ii++) {
                double Temp1 = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
                double Temp2 = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
                double Temp3 = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
                Gd(0, 3 * (ii)) = Temp1;
                Gd(1, 3 * (ii) + 1) = Temp1;
                Gd(2, 3 * (ii) + 2) = Temp1;

                Gd(3, 3 * (ii)) = Temp2;
                Gd(4, 3 * (ii) + 1) = Temp2;
                Gd(5, 3 * (ii) + 2) = Temp2;

                Gd(6, 3 * (ii)) = Temp3;
                Gd(7, 3 * (ii) + 1) = Temp3;
                Gd(8, 3 * (ii) + 2) = Temp3;
            }

            // Add damping strains
            ChMatrixNM<double, 6, 1> DEPS;
            DEPS.Reset();
            for (int ii = 0; ii < 33; ii++) {
                DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
                DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
                DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
                DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
                DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
                DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
            }

            // Add structural damping
            strain += DEPS * m_element->m_Alpha;

            // Stress tensor calculation
            ChMatrixNM<double, 6, 1> stress;
            stress.MatrMultiply(E_eps, strain);

            // Declaration and computation of Sigm, to be removed
            ChMatrixNM<double, 9, 9> Sigm;
            Sigm.Reset();

            Sigm(0, 0) = stress(0, 0);  // XX
            Sigm(1, 1) = stress(0, 0);
            Sigm(2, 2) = stress(0, 0);

            Sigm(0, 3) = stress(2, 0);  // XY
            Sigm(1, 4) = stress(2, 0);
            Sigm(2, 5) = stress(2, 0);

            Sigm(0, 6) = stress(4, 0);  // XZ
            Sigm(1, 7) = stress(4, 0);
            Sigm(2, 8) = stress(4, 0);

            Sigm(3, 0) = stress(2, 0);  // XY
            Sigm(4, 1) = stress(2, 0);
            Sigm(5, 2) = stress(2, 0);

            Sigm(3, 3) = stress(1, 0);  // YY
            Sigm(4, 4) = stress(1, 0);
            Sigm(5, 5) = stress(1, 0);

            Sigm(3, 6) = stress(5, 0);  // YZ
            Sigm(4, 7) = stress(5, 0);
            Sigm(5, 8) = stress(5, 0);

            Sigm(6, 0) = stress(4, 0);  // XZ
            Sigm(7, 1) = stress(4, 0);
            Sigm(8, 2) = stress(4, 0);

            Sigm(6, 3) = stress(5, 0);  // YZ
            Sigm(7, 4) = stress(5, 0);
            Sigm(8, 5) = stress(5, 0);

            Sigm(6, 6) = stress(3, 0);  // ZZ
            Sigm(7, 7) = stress(3, 0);
            Sigm(8, 8) = stress(3, 0);

            // Jacobian of internal forces
            ChMatrixNM<double, 33, 6> temp336;
            ChMatrixNM<double, 33, 9> temp339;
            // Term from differentiation of strain w.r.t. coordinates
            temp336.MatrTMultiply(strainD, E_eps);
            // Term from  differentiation of Jacobian of strain w.r.t. coordinates w.r.t. coordinates (that is, twice)
            temp339.MatrTMultiply(Gd, Sigm);
            // Sum contributions to the final Jacobian of internal forces
            m_KTE1.MatrMultiply(temp336, strainD);
            m_KTE2.MatrMultiply(temp339, Gd);

            result = m_KTE1 * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + m_KTE2 * m_Kfactor;
            result.MatrScale(detJ0 * m_element->m_GaussScaling);
        } break;
        case ChElementBrick_9::Hencky: {
            ChMatrixNM<double, 3, 3> Temp33;  ///< Temporary matrix
            ChMatrixNM<double, 3, 3> CCPinv;  ///< Inverse of F^{pT}*F^{p}, where F^{p} is the plastic deformation
            ChMatrix33<double> BETRI;         ///< Left Cauchy-Green tensor for elastic deformation
            double BETRI_eig[3] = {0.0};      ///< Eigenvalues of spatial stretch tensor
            ChMatrix33<double> BETRI_eigv;    ///< Eigenvectors of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e1;      ///< First principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e2;      ///< Second principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 1> e3;      ///< Third principal direction of spatial stretch tensor
            ChMatrixNM<double, 3, 3> MM1;     ///< Matrix from outer product of e1
            ChMatrixNM<double, 3, 3> MM2;     ///< Matrix from outer product of e2
            ChMatrixNM<double, 3, 3> MM3;     ///< Matrix from outer product of e3

            if (m_element->m_Plasticity) {
                CCPinv(0, 0) = m_element->m_CCPinv_Plast(0, m_element->m_InteCounter);
                CCPinv(0, 1) = m_element->m_CCPinv_Plast(1, m_element->m_InteCounter);
                CCPinv(0, 2) = m_element->m_CCPinv_Plast(2, m_element->m_InteCounter);
                CCPinv(1, 0) = m_element->m_CCPinv_Plast(3, m_element->m_InteCounter);
                CCPinv(1, 1) = m_element->m_CCPinv_Plast(4, m_element->m_InteCounter);
                CCPinv(1, 2) = m_element->m_CCPinv_Plast(5, m_element->m_InteCounter);
                CCPinv(2, 0) = m_element->m_CCPinv_Plast(6, m_element->m_InteCounter);
                CCPinv(2, 1) = m_element->m_CCPinv_Plast(7, m_element->m_InteCounter);
                CCPinv(2, 2) = m_element->m_CCPinv_Plast(8, m_element->m_InteCounter);
            } else {
                CCPinv.Reset();
                CCPinv(0, 0) = 1.0;
                CCPinv(1, 1) = 1.0;
                CCPinv(2, 2) = 1.0;
            }
            Temp33.MatrMultiply(DefF, CCPinv);       // Obtain elastic deformation gradient (remove plastic part)
            BETRI.MatrMultiplyT(Temp33, DefF);       // Obtain left Cauchy-Green tensor // ?
            BETRI.FastEigen(BETRI_eigv, BETRI_eig);  // Obtain eigenvalues of left Cauchy-Green tensor
            for (int i = 0; i < 3; i++) {
                e3(i, 0) = BETRI_eigv(i, 0);
                e1(i, 0) = BETRI_eigv(i, 1);
                e2(i, 0) = BETRI_eigv(i, 2);
            }
            MM1.MatrMultiplyT(e1, e1);
            MM2.MatrMultiplyT(e2, e2);
            MM3.MatrMultiplyT(e3, e3);
            // Compute logarithmic strains
            ChMatrixNM<double, 3, 1> LogStrain;
            LogStrain(0, 0) = 0.5 * log(BETRI_eig[1]);
            LogStrain(1, 0) = 0.5 * log(BETRI_eig[2]);
            LogStrain(2, 0) = 0.5 * log(BETRI_eig[0]);

            ChMatrixNM<double, 3, 3> FI;        ///< Inverse of total deformation gradient
            ChMatrixNM<double, 6, 33> strainD;  ///< Jacobian of strains w.r.t. coordinates
            FI = DefF;
            FI.MatrInverse();
            // Obtain Jacobian of strains w.r.t. element coordinates
            m_element->ComputeStrainD_Brick9(strainD, Nx, Ny, Nz, FI, j0);

            // Obtain eigenvalues of Kirchhoff stress tensor
            ChMatrixNM<double, 3, 1> StressK_eig;
            StressK_eig(0, 0) =
                E_eps(0, 0) * LogStrain(0, 0) + E_eps(0, 1) * LogStrain(1, 0) + E_eps(0, 3) * LogStrain(2, 0);
            StressK_eig(1, 0) =
                E_eps(1, 0) * LogStrain(0, 0) + E_eps(1, 1) * LogStrain(1, 0) + E_eps(1, 3) * LogStrain(2, 0);
            StressK_eig(2, 0) =
                E_eps(3, 0) * LogStrain(0, 0) + E_eps(3, 1) * LogStrain(1, 0) + E_eps(3, 3) * LogStrain(2, 0);

            // Set Yield flag to zero (within elastic range)
            int YieldFlag = 0;

            ChMatrixNM<double, 6, 6> Dep;

            // For plasticity, apply return mapping
            if (m_element->m_Plasticity) {
                double G = E / (2.0 * (1 + nu));
                double K = E / (3.0 * (1 - 2.0 * nu));

                // Volumetric Hencky strain
                double EEVD3 = (LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0)) / 3.0;

                // Deviatoric  Hencky strain
                ChVector<double> EETD;
                EETD.x() = LogStrain(0, 0) - EEVD3;
                EETD.y() = LogStrain(1, 0) - EEVD3;
                EETD.z() = LogStrain(2, 0) - EEVD3;
                // Norm of deviatoric Hencky strain
                double ETDNorm = EETD.Length();
                // Hydrostatic pressure
                double hydroP;
                // Deviatoric stress tensor
                ChVector<double> devStress;
                // Norm of deviatoric stress tensor
                double NormSn;
                // Second invariant of stress tensor
                double J2Rt;
                // Current value of yield function
                double YieldFunc;
                // Flag indicating whether an elastic (0) or plastic (1) step occurs
                int YieldFlag;
                // Variation of flow rate
                double DeltaGamma;
                // Deviatoric stresses updated
                ChVector<double> devStressUp;
                // Vector of eigenvalues of current logarithmic strain
                ChVector<double> lambda;

                switch (m_element->GetPlasticityFormulation()) {
                    case ChElementBrick_9::J2: {  // Hydrostatic pressure , i.e. volumetric stress (from principal
                                                  // stresses)
                        hydroP = (StressK_eig(0, 0) + StressK_eig(1, 0) + StressK_eig(2, 0)) / 3.0;

                        // Deviatoric stress
                        devStress.x() = StressK_eig(0, 0) - hydroP;
                        devStress.y() = StressK_eig(1, 0) - hydroP;
                        devStress.z() = StressK_eig(2, 0) - hydroP;
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        // Second invariant of the stress tensor (J2)
                        J2Rt = NormSn / sqrt(2.0);
                        // Trial stress for yield function
                        double qtrial = sqrt(3.0) * J2Rt;

                        // Evaluation of J2 yield function
                        YieldFunc = qtrial - (m_element->m_YieldStress +
                                              m_element->m_HardeningSlope *
                                                  m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));

                        // Set Yield flag to zero (within elastic range)
                        YieldFlag = 0;

                        if (YieldFunc > 0.0) {
                            // If yield function reveals plasticity, apply return mapping algorithm
                            YieldFlag = 1;

                            // Step variation of the plastic flow (rate)
                            double DeltaGamma = YieldFunc / (3.0 * G + m_element->m_HardeningSlope);

                            // Obtain some terms necessary for plastic Jacobian of internal forces
                            qtrial = sqrt(3.0 / 2.0) * NormSn + 3.0 * G * DeltaGamma;
                            double AFACT = 2.0 * G * (1.0 - 3.0 * G * DeltaGamma / qtrial);
                            double BFACT = 6.0 * G * G *
                                           (DeltaGamma / qtrial - 1.0 / (3.0 * G + m_element->m_HardeningSlope)) /
                                           (NormSn * NormSn);

                            ChMatrixNM<double, 6, 1> devStressVec;
                            devStressVec(0, 0) = devStress.x();
                            devStressVec(1, 0) = devStress.y();
                            devStressVec(3, 0) = devStress.z();
                            devStressVec(2, 0) = 0.0;
                            devStressVec(4, 0) = 0.0;
                            devStressVec(5, 0) = 0.0;

                            // Obtain matrices DEVPRJ and Dep, necessary for Jacobian of plastic internal forces
                            ChMatrixNM<double, 6, 6> FOID;
                            FOID.Reset();

                            FOID(0, 0) = 1.0;
                            FOID(1, 1) = 1.0;
                            FOID(2, 2) = 0.5;
                            FOID(3, 3) = 1.0;
                            FOID(4, 4) = 0.5;
                            FOID(5, 5) = 0.5;

                            ChMatrixNM<double, 6, 1> SOID;
                            SOID(0, 0) = 1.0;
                            SOID(1, 0) = 1.0;
                            SOID(3, 0) = 1.0;
                            SOID(2, 0) = 0.0;
                            SOID(4, 0) = 0.0;
                            SOID(5, 0) = 0.0;

                            for (int i = 0; i < 6; i++) {
                                for (int j = i; j < 6; j++) {
                                    double DEVPRJ_ij = FOID(i, j) - SOID(i, 0) * SOID(j, 0) / 3.0;
                                    Dep(i, j) = AFACT * DEVPRJ_ij + BFACT * devStressVec(i, 0) * devStressVec(j, 0) +
                                                K * SOID(i, 0) * SOID(j, 0);
                                }
                            }

                            for (int j = 0; j < 5; j++) {
                                for (int i = j + 1; i < 6; i++) {
                                    Dep(i, j) = Dep(j, i);
                                }
                            }
                        }
                    } break;

                    case ChElementBrick_9::DruckerPrager: {
                        double EDNInv;  // Inverse of norm of deviatoric Hencky strain
                        // Evaluate norm of deviatoric Hencky strain
                        if (ETDNorm != 0.0) {
                            EDNInv = 1.0 / ETDNorm;
                        } else {
                            EDNInv = 0.0;
                        }

                        ChMatrixNM<double, 6, 1> UniDev;
                        UniDev(0, 0) = EETD.x() * EDNInv;
                        UniDev(1, 0) = EETD.y() * EDNInv;
                        UniDev(3, 0) = EETD.z() * EDNInv;
                        UniDev(2, 0) = 0.0;
                        UniDev(4, 0) = 0.0;
                        UniDev(5, 0) = 0.0;

                        double EETV = LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0);
                        hydroP = K * EETV;
                        devStress.x() = 2.0 * G * (LogStrain(0, 0) - EEVD3);
                        devStress.y() = 2.0 * G * (LogStrain(1, 0) - EEVD3);
                        devStress.z() = 2.0 * G * (LogStrain(2, 0) - EEVD3);
                        // Euclidean natural norm of the second-order tensor
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        J2Rt = NormSn / sqrt(2.0);

                        double phi = m_element->m_FrictionAngle * CH_C_DEG_TO_RAD;    // Friction angle
                        double phi2 = m_element->m_DilatancyAngle * CH_C_DEG_TO_RAD;  // Dilatancy angle
                        double eta;  // Coefficient multiplying hydros. pressure in yield function - function of
                                     // internal friction
                        double gsi;  // Coefficient multiplying 'current' cohesion value in yield function
                        double etab;

                        if (m_element->m_DPHardening == 1) {  // Tension corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 + tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 2) {  // Compression corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 - tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 3) {  // Shear corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = 1.0 / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        }

                        double alpha1;
                        double beta1;
                        if (etab == 0) {
                            etab = 0.000001;
                        }
                        alpha1 = gsi / etab;
                        if (eta == 0) {
                            eta = 0.000001;
                        }
                        beta1 = gsi / eta;

                        // Yield function at trial stage
                        YieldFunc =
                            J2Rt + eta * hydroP -
                            gsi * (m_element->m_YieldStress +
                                   m_element->m_HardeningSlope * m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                        double SQRJ2T = J2Rt;
                        double PT = hydroP;
                        // If YieldFunc > 0.0 there is need for return mapping
                        YieldFlag = 0;
                        if (YieldFunc > 0.0) {
                            YieldFlag = 1;

                            // Initialize for Newton-Raphson
                            double DGamma = 0.0;
                            double Yfunc1 = YieldFunc;  // Initially recalculated

                            for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                // Compute newton residual
                                double DDGamma =
                                    Yfunc1 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                DGamma = DGamma + DDGamma;
                                double EpBar = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DGamma;
                                SQRJ2T = SQRJ2T - G * DGamma;
                                double P = PT - K * etab * DGamma;
                                Yfunc1 = SQRJ2T + eta * P -
                                         gsi * (m_element->m_YieldStress + m_element->m_HardeningSlope * EpBar);
                                if (Yfunc1 < m_element->GetDPYieldTol()) {
                                    break;
                                }
                                if (ii == m_element->GetDPIterationNo() - 1) {
                                    throw ChException(
                                        "Maximum number of iterations reached in Drucker-Prager Newton-Raphson "
                                        "algorithm. Jacobian \n");
                                }
                            }
                            DeltaGamma = DGamma;
                            double Check_DP_Cone = J2Rt - G * DGamma;

                            if (Check_DP_Cone < 0.0) {  // Cone return mapping

                                double Rtrial = hydroP -
                                                beta1 * (m_element->m_YieldStress +
                                                         m_element->m_HardeningSlope *
                                                             m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                                DeltaGamma = Rtrial / (K + alpha1 * beta1 * m_element->m_HardeningSlope);

                                // Stress update
                                StressK_eig(0, 0) = hydroP - K * DeltaGamma;
                                StressK_eig(1, 0) = hydroP - K * DeltaGamma;
                                StressK_eig(2, 0) = hydroP - K * DeltaGamma;
                            } else {
                                // Deviatoric stress
                                devStressUp = (1.0 - G * DeltaGamma / J2Rt) * devStress;
                                // Hydrostatic stress
                                double hydroPUp = hydroP - K * etab * DeltaGamma;
                                // Stress update
                                StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                            }

                            // Calculation for Dep: Plastic contribution to Jacobian of internal forces
                            ChMatrixNM<double, 6, 6> FOID;
                            FOID.Reset();

                            FOID(0, 0) = 1.0;
                            FOID(1, 1) = 1.0;
                            FOID(2, 2) = 0.5;
                            FOID(3, 3) = 1.0;
                            FOID(4, 4) = 0.5;
                            FOID(5, 5) = 0.5;

                            ChMatrixNM<double, 6, 1> SOID;
                            SOID(0, 0) = 1.0;
                            SOID(1, 0) = 1.0;
                            SOID(3, 0) = 1.0;
                            SOID(2, 0) = 0.0;
                            SOID(4, 0) = 0.0;
                            SOID(5, 0) = 0.0;

                            double AFact;
                            if (Check_DP_Cone >= 0.0) {  // Consistent tangent for smooth cone wall return
                                double Aux = 1.0 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                AFact = 2.0 * G * (1.0 - DeltaGamma / (sqrt(2.0) * ETDNorm));
                                double BFact = 2.0 * G * (DeltaGamma / (sqrt(2.0) * ETDNorm) - G * Aux);
                                double CFact = -sqrt(2.0) * G * Aux * K;
                                double DFact = K * (1.0 - K * eta * etab * Aux);

                                for (int ii = 0; ii < 6; ii++) {
                                    for (int jj = 0; jj < 6; jj++) {
                                        Dep(ii, jj) =
                                            AFact * FOID(ii, jj) + BFact * UniDev(ii) +
                                            CFact * (eta * UniDev(ii) * SOID(jj) + etab * SOID(ii) * UniDev(jj)) +
                                            (DFact - AFact / 3.0) * SOID(ii) * SOID(jj);
                                    }
                                }
                            } else {  // Consistent tangent for cone point
                                AFact = K * (1.0 - K / (K + alpha1 * beta1 * m_element->m_HardeningSlope));
                                for (int ii = 0; ii < 6; ii++) {
                                    for (int jj = 0; jj < 6; jj++) {
                                        Dep(ii, jj) = AFact * SOID(ii) * SOID(jj);
                                    }
                                }
                            }
                        }  // End Yield Criteria
                    } break;
                    case ChElementBrick_9::DruckerPrager_Cap: {
                        // Hydrostatic pressure (Cap)
                        double hydroPt;
                        // Current value of yield function (Cap)
                        double YieldFunc_Cap;
                        int FlagYieldType;  //(DP_Cap)
                        int PlasticCount;   //(DP_Cap)
                        double EDNInv;      // Inverse of norm of deviatoric Hencky strain

                        // Evaluate norm of deviatoric Hencky strain
                        if (ETDNorm != 0.0) {
                            EDNInv = 1.0 / ETDNorm;
                        } else {
                            EDNInv = 0.0;
                        }

                        ChMatrixNM<double, 6, 1> UniDev;
                        UniDev(0, 0) = EETD.x() * EDNInv;
                        UniDev(1, 0) = EETD.y() * EDNInv;
                        UniDev(3, 0) = EETD.z() * EDNInv;
                        UniDev(2, 0) = 0.0;
                        UniDev(4, 0) = 0.0;
                        UniDev(5, 0) = 0.0;

                        double EETV = LogStrain(0, 0) + LogStrain(1, 0) + LogStrain(2, 0);
                        hydroP = K * EETV;
                        devStress.x() = 2.0 * G * (LogStrain(0, 0) - EEVD3);
                        devStress.y() = 2.0 * G * (LogStrain(1, 0) - EEVD3);
                        devStress.z() = 2.0 * G * (LogStrain(2, 0) - EEVD3);
                        // Euclidean natural norm of the second-order tensor
                        NormSn =
                            sqrt(devStress.x() * devStress.x() + devStress.y() * devStress.y() + devStress.z() * devStress.z());

                        J2Rt = NormSn / sqrt(2.0);

                        double phi = m_element->m_FrictionAngle * CH_C_DEG_TO_RAD;    // Friction angle
                        double phi2 = m_element->m_DilatancyAngle * CH_C_DEG_TO_RAD;  // Dilatancy angle

                        double eta;  // Coefficient multiplying hydros. pressure in yield function - function of
                                     // internal friction
                        double gsi;  // Coefficient multiplying 'current' cohesion value in yield function
                        double etab;

                        if (m_element->m_DPHardening == 1) {  // Tension corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 + tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 2) {  // Compression corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = (1.0 - tan(phi) / 3.0) / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        } else if (m_element->m_DPHardening == 3) {  // Shear corresponding to Abaqus model
                            eta = tan(phi) / sqrt(3.0);
                            gsi = 1.0 / sqrt(3.0);
                            etab = tan(phi2) / sqrt(3.0);
                        }
                        double alpha1;
                        double beta1;
                        if (etab == 0) {
                            etab = 0.000001;
                        }
                        alpha1 = gsi / etab;
                        if (eta == 0) {
                            eta = 0.000001;
                        }
                        beta1 = gsi / eta;

                        //-------------------------------//
                        // Yield function at trial stage //
                        //-------------------------------//

                        // DP yield function
                        YieldFunc =
                            J2Rt + eta * hydroP -
                            gsi * (m_element->m_YieldStress +
                                   m_element->m_HardeningSlope * m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));

                        // CAP yield function
                        hydroPt = beta1 * m_element->m_YieldStress;
                        double CapM = sqrt(3.0) * eta;

                        double MeanEffP;
                        double Hi;
                        // double alphUp;
                        // ChMatrixDynamic<double> mm_DPVector1 = m_element->m_DPVector1;
                        // ChMatrixDynamic<double> mm_DPVector2 = m_element->m_DPVector2;
                        // int mm_DPVector_size=m_element->m_DPVector_size;
                        // double mm_DPCapBeta=m_element->m_DPCapBeta;

                        // alphUp = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0);
                        // GetLog() << "m_DPVector1" << mm_DPVector1 << "m_DPVector2" << mm_DPVector2 <<"\n";

                        // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                        m_element->ComputeHardening_a(
                            MeanEffP, Hi, m_element->m_Alpha_Plast(m_element->m_InteCounter, 0), m_element->m_DPVector1,
                            m_element->m_DPVector2, m_element->m_DPVector_size);

                        YieldFunc_Cap = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                            (hydroP - hydroPt + MeanEffP) * (hydroP - hydroPt + MeanEffP) +
                                        (sqrt(3.0) * J2Rt / CapM) * (sqrt(3.0) * J2Rt / CapM) - MeanEffP * MeanEffP;

                        double SQRJ2T = J2Rt;
                        double PT = hydroP;
                        double EPBARN = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0);

                        YieldFlag = 0;
                        FlagYieldType = 0;

                        if (YieldFunc > 0.0 || (YieldFunc_Cap > 0.0 && hydroP < hydroPt - MeanEffP)) {
                            YieldFlag = 1;
                            PlasticCount = 1;
                            if (YieldFunc > 0.0) {  // DP smoothed surface return mapping
                                // Initialize for Newton-Raphson
                                double DGamma = 0.0;
                                double Yfunc1 = YieldFunc;  // Initially recalculated

                                for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                    // Compute Newton residual
                                    double DDGamma =
                                        Yfunc1 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                    DGamma = DGamma + DDGamma;
                                    SQRJ2T = SQRJ2T - G * DGamma;
                                    double P = PT - K * etab * DGamma;
                                    double EpBar = m_element->m_Alpha_Plast(m_element->m_InteCounter, 0) + gsi * DGamma;
                                    Yfunc1 = SQRJ2T + eta * P -
                                             gsi * (m_element->m_YieldStress + m_element->m_HardeningSlope * EpBar);
                                    if (std::abs(Yfunc1) < m_element->GetDPYieldTol())
                                        break;

                                    if (ii == m_element->GetDPIterationNo() - 1)
                                        throw ChException(
                                            "Maximum number of iterations reached in Drucker-Prager Surface "
                                            "Newton-Raphson algorithm");
                                }
                                DeltaGamma = DGamma;
                                double Check_DP_Cone = J2Rt - G * DGamma;

                                if (Check_DP_Cone < 0.0) {  // Cone return mapping
                                    FlagYieldType = 2;
                                    double Rtrial = hydroP -
                                                    beta1 * (m_element->m_YieldStress +
                                                             m_element->m_HardeningSlope *
                                                                 m_element->m_Alpha_Plast(m_element->m_InteCounter, 0));
                                    DeltaGamma = Rtrial / (K + alpha1 * beta1 * m_element->m_HardeningSlope);

                                    // Stress update
                                    StressK_eig(0, 0) = hydroP - K * DeltaGamma;
                                    StressK_eig(1, 0) = hydroP - K * DeltaGamma;
                                    StressK_eig(2, 0) = hydroP - K * DeltaGamma;

                                    //// Calculate update elastic logarithmic strain
                                    // LogStrain = StressK_eig;
                                    // LogStrain.MatrScale(1 / (3.0 * K));
                                } else {
                                    if (YieldFunc_Cap <= 0.0 ||
                                        ((hydroP - K * etab * DeltaGamma) >= hydroPt - MeanEffP)) {
                                        FlagYieldType = 1;

                                        // Deviatoric stress
                                        devStressUp = (1.0 - G * DeltaGamma / J2Rt) * devStress;
                                        // Hydrostatic stress
                                        double hydroPUp = hydroP - K * etab * DeltaGamma;
                                        // Stress update
                                        StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                        StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                        StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                        //// Calculate update elastic logarithmic strain
                                        // LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                        // LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                        // LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    }
                                }  // end of // Cone return mapping
                                // Calculation for Dep: Plastic contribution to Jacobian of internal forces
                                ChMatrixNM<double, 6, 6> FOID;
                                FOID.Reset();

                                FOID(0, 0) = 1.0;
                                FOID(1, 1) = 1.0;
                                FOID(2, 2) = 0.5;
                                FOID(3, 3) = 1.0;
                                FOID(4, 4) = 0.5;
                                FOID(5, 5) = 0.5;

                                ChMatrixNM<double, 6, 1> SOID;
                                SOID(0, 0) = 1.0;
                                SOID(1, 0) = 1.0;
                                SOID(3, 0) = 1.0;
                                SOID(2, 0) = 0.0;
                                SOID(4, 0) = 0.0;
                                SOID(5, 0) = 0.0;

                                double AFact;
                                if (Check_DP_Cone >= 0.0) {  // Consistent tangent for smooth cone wall return
                                    double Aux = 1.0 / (G + K * eta * etab + gsi * gsi * m_element->m_HardeningSlope);
                                    AFact = 2.0 * G * (1.0 - DeltaGamma / (sqrt(2.0) * ETDNorm));
                                    double BFact = 2.0 * G * (DeltaGamma / (sqrt(2.0) * ETDNorm) - G * Aux);
                                    double CFact = -sqrt(2.0) * G * Aux * K;
                                    double DFact = K * (1.0 - K * eta * etab * Aux);

                                    for (int ii = 0; ii < 6; ii++) {
                                        for (int jj = 0; jj < 6; jj++) {
                                            Dep(ii, jj) =
                                                AFact * FOID(ii, jj) + BFact * UniDev(ii) +
                                                CFact * (eta * UniDev(ii) * SOID(jj) + etab * SOID(ii) * UniDev(jj)) +
                                                (DFact - AFact / 3.0) * SOID(ii) * SOID(jj);
                                        }
                                    }
                                } else {  // Consistent tangent for cone point
                                    AFact = K * (1.0 - K / (K + alpha1 * beta1 * m_element->m_HardeningSlope));
                                    for (int ii = 0; ii < 6; ii++) {
                                        for (int jj = 0; jj < 6; jj++) {
                                            Dep(ii, jj) = AFact * SOID(ii) * SOID(jj);
                                        }
                                    }
                                }
                            }  // end of // DP smoothed surface return mapping

                            if (YieldFunc_Cap > 0.0 && FlagYieldType == 0) {  // Cap return mapping
                                // Initialize of Newton raphson for Cap surface
                                double DGamma = 0.0;
                                double DGamma_A;
                                double DGamma_B;
                                double EPBAR = EPBARN;
                                double SQRJ2 = SQRJ2T;
                                double P = PT;
                                ChVector<double> devS = devStress;
                                double Acof;
                                double A11;
                                double A12;
                                double A21;
                                double A22;
                                double B11;
                                double B12;
                                double B21;
                                double B22;
                                double Res01;
                                double Res02;
                                double DDGamma;
                                double DALPHA;
                                // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                              m_element->m_DPVector2, m_element->m_DPVector_size);
                                // Evaluate initial 2 residual vectors
                                Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                            (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                        (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) - MeanEffP * MeanEffP;
                                Res02 = EPBAR - EPBARN +
                                        DGamma * 2.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                            (P - hydroPt + MeanEffP);

                                for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                    Acof = P - hydroPt + MeanEffP;
                                    A11 = -12.0 * G / (CapM * CapM + 6.0 * G * DGamma) * (sqrt(3.0) * SQRJ2 / CapM) *
                                          (sqrt(3.0) * SQRJ2 / CapM);
                                    A12 = 2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi) -
                                          2.0 * MeanEffP * Hi;
                                    A21 = 2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta);
                                    A22 = 1.0 +
                                          2.0 * DGamma / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi);

                                    if (A11 * A22 - A12 * A21 == 0.0) {
                                        printf("Cap Return Singular Matrix!!\n");
                                        exit(1);
                                    }

                                    B11 = A22 / (A11 * A22 - A12 * A21);
                                    B12 = -A12 / (A11 * A22 - A12 * A21);
                                    B21 = -A21 / (A11 * A22 - A12 * A21);
                                    B22 = A11 / (A11 * A22 - A12 * A21);

                                    DDGamma = -B11 * Res01 - B12 * Res02;
                                    DALPHA = -B21 * Res01 - B22 * Res02;

                                    DGamma = DGamma + DDGamma;  //! Update Newton increment in terms of DGamma
                                    EPBAR = EPBAR +
                                            DALPHA;  //! Update Newton increment in terms of alpha(hardening parameters)
                                    P = PT + K * (EPBAR - EPBARN);
                                    SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * SQRJ2T;
                                    devS.x() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.x();
                                    devS.y() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.y();
                                    devS.z() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma) * devStress.z();

                                    // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                    m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                  m_element->m_DPVector2, m_element->m_DPVector_size);

                                    Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                            (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                            MeanEffP * MeanEffP;
                                    Res02 = EPBAR - EPBARN +
                                            DGamma * 2.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                                (P - hydroPt + MeanEffP);

                                    if (std::abs(Hi) < m_element->GetDPYieldTol()) {
                                        if (sqrt(Res01 * Res01) < m_element->GetDPYieldTol() &&
                                            sqrt(Res02 * Res02) < m_element->GetDPYieldTol())
                                            break;
                                    } else {
                                        if (sqrt(Res01 * Res01) / std::abs(Hi) < m_element->GetDPYieldTol() &&
                                            sqrt(Res02 * Res02) / std::abs(Hi) < m_element->GetDPYieldTol())
                                            break;
                                    }
                                    if (ii == m_element->GetDPIterationNo() - 1)
                                        throw ChException(
                                            "Maximum number of iterations reached in Drucker-Prager Cap surface "
                                            "Newton-Raphson algorithm");
                                }  // End of Newton raphson

                                DeltaGamma = DGamma;

                                if (P > hydroPt - MeanEffP) {  // Transition return mapping
                                    FlagYieldType = 4;
                                    // Initialize for newton raphson
                                    DGamma_B = 0.0;
                                    double AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B);
                                    DGamma_A = (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                    EPBAR = EPBARN - etab * DGamma_A;
                                    SQRJ2 = SQRJ2T;
                                    P = PT;
                                    devS = devStress;
                                    // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                    m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                  m_element->m_DPVector2, m_element->m_DPVector_size);
                                    // initial resid vector
                                    Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                            (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                            MeanEffP * MeanEffP;
                                    double DGamma_B1;
                                    double Res11;
                                    double DRes01;
                                    for (int ii = 0; ii < m_element->GetDPIterationNo(); ii++) {
                                        // TransNewtonDifference

                                        //** perturbation for DGAMA_B1
                                        DGamma_B1 = DGamma_B + 1e-10;
                                        AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B1);
                                        DGamma_A = (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                        EPBAR = EPBARN - etab * DGamma_A;
                                        // obtain "hardening parameter a"=MeanEffP and "first derivative of a" =Hi
                                        m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                      m_element->m_DPVector2,
                                                                      m_element->m_DPVector_size);
                                        // update stresses
                                        P = PT - K * etab * DGamma_A;
                                        SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B1) *
                                                (1.0 - G * DGamma_A / SQRJ2T) * SQRJ2T;
                                        // Check yield functions
                                        Res11 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                    (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                                (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                                MeanEffP * MeanEffP;
                                        DRes01 = (Res11 - Res01) / 1e-10;

                                        if (DRes01 == 0.0) {
                                            printf("Singular for Transition DP/CAP!!\n");
                                            exit(1);
                                        }

                                        // update 3 parameters (plastic and hardening parameters)
                                        DGamma_B = DGamma_B - 1.0 / DRes01 * Res01;
                                        AA = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B);
                                        DGamma_A = (AA * SQRJ2T + eta * PT - eta * hydroPt) / (AA * G + K * eta * etab);
                                        EPBAR = EPBARN - etab * DGamma_A;  // plastic strain
                                        // Update yield stress and the Hi(slop) at plastic strain from table
                                        m_element->ComputeHardening_a(MeanEffP, Hi, EPBAR, m_element->m_DPVector1,
                                                                      m_element->m_DPVector2,
                                                                      m_element->m_DPVector_size);
                                        // Update stresses
                                        P = PT - K * etab * DGamma_A;
                                        SQRJ2 = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                (1.0 - G * DGamma_A / SQRJ2T) * SQRJ2T;
                                        devS.x() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.x();
                                        devS.y() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.y();
                                        devS.z() = CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) *
                                                 (1.0 - G * DGamma_A / SQRJ2T) * devStress.z();
                                        // Check yield functions
                                        Res01 = (1.0 / (m_element->m_DPCapBeta * m_element->m_DPCapBeta)) *
                                                    (P - hydroPt + MeanEffP) * (P - hydroPt + MeanEffP) +
                                                (sqrt(3.0) * SQRJ2 / CapM) * (sqrt(3.0) * SQRJ2 / CapM) -
                                                MeanEffP * MeanEffP;
                                        if (std::abs(Hi) < m_element->GetDPYieldTol()) {
                                            if (std::abs(Res01) < m_element->GetDPYieldTol())
                                                break;
                                        } else {
                                            if (std::abs(Res01) / std::abs(Hi) < m_element->GetDPYieldTol())
                                                break;
                                        }
                                        if (ii == m_element->GetDPIterationNo() - 1)
                                            throw ChException(
                                                "Hit the max iteration for Transient Cap_surf and DP_surf return "
                                                "mapping");
                                    }  // End of Newton raphson

                                    DeltaGamma = DGamma_A + DGamma_B;

                                    // Update stress
                                    devStressUp = devS;
                                    //(Hydrostatic)
                                    double hydroPUp = P;
                                    // Stress update first
                                    StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                    StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                    StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                    ////calculate updated elastic logarithmic Strain
                                    // LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    // LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    // LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                } else {
                                    FlagYieldType = 3;

                                    // Update stress
                                    devStressUp = devS;
                                    //(Hydrostatic)
                                    double hydroPUp = P;
                                    // Stress update first
                                    StressK_eig(0, 0) = devStressUp.x() + hydroPUp;
                                    StressK_eig(1, 0) = devStressUp.y() + hydroPUp;
                                    StressK_eig(2, 0) = devStressUp.z() + hydroPUp;
                                    ////calculate updated elastic logarithmic Strain
                                    // LogStrain(0, 0) = devStressUp.x() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    // LogStrain(1, 0) = devStressUp.y() / (2.0 * G) + hydroPUp / (3.0 * K);
                                    // LogStrain(2, 0) = devStressUp.z() / (2.0 * G) + hydroPUp / (3.0 * K);
                                }  // end if of transition return mapping
                                // Calculation for Dep: Plastic contribution to Jacobian of internal forces
                                ChMatrixNM<double, 6, 6> FOID;
                                FOID.Reset();

                                FOID(0, 0) = 1.0;
                                FOID(1, 1) = 1.0;
                                FOID(2, 2) = 0.5;
                                FOID(3, 3) = 1.0;
                                FOID(4, 4) = 0.5;
                                FOID(5, 5) = 0.5;

                                ChMatrixNM<double, 6, 1> SOID;
                                SOID(0, 0) = 1.0;
                                SOID(1, 0) = 1.0;
                                SOID(3, 0) = 1.0;
                                SOID(2, 0) = 0.0;
                                SOID(4, 0) = 0.0;
                                SOID(5, 0) = 0.0;

                                double AFact;
                                double BFact;
                                double C1Fact;
                                double C2Fact;
                                double DFact;
                                double C11;
                                double C12;
                                double C21;
                                double C22;
                                if (FlagYieldType == 3) {  //!! Consistent tangent for cap surface
                                    Acof = P - hydroPt + MeanEffP;
                                    A11 = -(-12.0 * G / (CapM * CapM + 6.0 * G * DGamma) * (sqrt(3.0) * SQRJ2 / CapM) *
                                            (sqrt(3.0) * SQRJ2 / CapM));
                                    A12 = -(2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi) -
                                            2.0 * MeanEffP * Hi);
                                    A21 = -(2.0 * Acof / (m_element->m_DPCapBeta * m_element->m_DPCapBeta));
                                    A22 =
                                        -(1.0 +
                                          2.0 * DGamma / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * (K + Hi));

                                    B11 = A22 / (A11 * A22 - A12 * A21);
                                    B12 = -A12 / (A11 * A22 - A12 * A21);
                                    B21 = -A21 / (A11 * A22 - A12 * A21);
                                    B22 = A11 / (A11 * A22 - A12 * A21);

                                    C11 = 2.0 * sqrt(6.0) * G * (CapM * CapM) * sqrt(3.0) * SQRJ2T /
                                          ((CapM * CapM + 6.0 * G * DGamma) * (CapM * CapM + 6.0 * G * DGamma)) * B11;
                                    C12 = 2.0 * Acof * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * B11 +
                                          2.0 * DGamma * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * B12;
                                    C21 = 2.0 * sqrt(6.0) * G * (CapM * CapM) * sqrt(3.0) * SQRJ2T /
                                          ((CapM * CapM + 6.0 * G * DGamma) * (CapM * CapM + 6.0 * G * DGamma)) * B21;
                                    C22 = 2.0 * Acof * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * B21 +
                                          2.0 * DGamma * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) * B22;

                                    AFact = 2.0 * G * (CapM * CapM) / ((CapM * CapM) + 6.0 * G * DGamma);
                                    BFact = -12.0 * (G * G) * (CapM * CapM) * ETDNorm * C11 /
                                            (((CapM * CapM) + 6.0 * G * DGamma) * ((CapM * CapM) + 6.0 * G * DGamma));
                                    C1Fact = -12.0 * (G * G) * (CapM * CapM) * ETDNorm * C12 /
                                             (((CapM * CapM) + 6.0 * G * DGamma) * ((CapM * CapM) + 6.0 * G * DGamma));
                                    C2Fact = K * C21;
                                    DFact = K * (1.0 + C22);

                                    for (int ii = 0; ii < 6; ii++) {
                                        for (int jj = 0; jj < 6; jj++) {
                                            Dep(ii, jj) = AFact * FOID(ii, jj) + BFact * UniDev(ii) * UniDev(jj) +
                                                          C1Fact * UniDev(ii) * SOID(jj) +
                                                          C2Fact * SOID(ii) * UniDev(jj) +
                                                          (DFact - AFact / 3.0) * SOID(ii) * SOID(jj);
                                        }
                                    }
                                }
                                if (FlagYieldType == 4) {  // Consistent tangent for elastic modulus
                                    Acof = G * DGamma_A / SQRJ2T - 1.0;
                                    double Bcof = CapM * CapM + 6.0 * G * DGamma_B;

                                    A11 = -(-K * eta * etab - G * CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B));
                                    A12 = -(2.0 * sqrt(3.0) * G * (CapM * CapM) * sqrt(3.0) * SQRJ2T * Acof /
                                            (Bcof * Bcof));
                                    A21 = -(2.0 * sqrt(3.0) * G * (CapM * CapM) * sqrt(3.0) * SQRJ2T * Acof /
                                                (Bcof * Bcof) -
                                            2.0 * K * etab * (P - hydroPt + MeanEffP) /
                                                (m_element->m_DPCapBeta * m_element->m_DPCapBeta));
                                    A22 = -(-12.0 * G * (CapM * CapM) * ((sqrt(3.0) * SQRJ2T) * (sqrt(3.0) * SQRJ2T)) *
                                            (Acof * Acof) / (Bcof * Bcof * Bcof));

                                    B11 = A22 / (A11 * A22 - A12 * A21);
                                    B12 = -A12 / (A11 * A22 - A12 * A21);
                                    B21 = -A21 / (A11 * A22 - A12 * A21);
                                    B22 = A11 / (A11 * A22 - A12 * A21);

                                    C11 = B11 * sqrt(2.0) * G * (CapM * CapM) / (CapM * CapM + 6.0 * G * DGamma_B) -
                                          B12 * 2.0 * sqrt(2.0) * G * (CapM * CapM) /
                                              ((CapM * CapM + 6.0 * G * DGamma_B) * (CapM * CapM + 6.0 * G * DGamma_B));
                                    C12 = B11 * K * eta +
                                          B12 * 2.0 * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                              (P - hydroPt + MeanEffP);
                                    C21 = B21 * sqrt(2.0) * G * (CapM * CapM) / (CapM * CapM + 6.0 * G * DGamma_B) -
                                          B22 * 2.0 * sqrt(2.0) * G * (CapM * CapM) /
                                              ((CapM * CapM + 6.0 * G * DGamma_B) * (CapM * CapM + 6.0 * G * DGamma_B));
                                    C22 = B21 * K * eta +
                                          B12 * 2.0 * K / (m_element->m_DPCapBeta * m_element->m_DPCapBeta) *
                                              (P - hydroPt + MeanEffP);

                                    AFact = 2.0 * G * CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B);
                                    BFact = -sqrt(2.0) * G * CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) * C11 -
                                            12.0 * G * G * CapM * CapM / ((CapM * CapM + 6.0 * G * DGamma_B) *
                                                                          (CapM * CapM + 6.0 * G * DGamma_B)) *
                                                (ETDNorm - 1.0 / sqrt(2.0) * DGamma_A) * C21;
                                    C1Fact = -sqrt(2.0) * G * CapM * CapM / (CapM * CapM + 6.0 * G * DGamma_B) * C12 -
                                             12.0 * G * G * CapM * CapM / ((CapM * CapM + 6.0 * G * DGamma_B) *
                                                                           (CapM * CapM + 6.0 * G * DGamma_B)) *
                                                 (ETDNorm - 1.0 / sqrt(2.0) * DGamma_A) * C22;
                                    C2Fact = -K * etab * C11;
                                    DFact = K * (1.0 - etab * C12);

                                    for (int ii = 0; ii < 6; ii++) {
                                        for (int jj = 0; jj < 6; jj++) {
                                            Dep(ii, jj) = AFact * FOID(ii, jj) + BFact * UniDev(ii) * UniDev(jj) +
                                                          C1Fact * UniDev(ii) * SOID(jj) +
                                                          C2Fact * SOID(ii) * UniDev(jj) +
                                                          (DFact - AFact / 3.0) * SOID(ii) * SOID(jj);
                                        }
                                    }
                                    // Elastic De
                                    for (int ii = 0; ii < 6; ii++) {
                                        for (int jj = 0; jj < 6; jj++) {
                                            Dep(ii, jj) =
                                                2.0 * G * FOID(ii, jj) + (K - 2.0 * G / 3.0) * SOID(ii) * SOID(jj);
                                        }
                                    }
                                }

                            }  // end if Cap return mapping

                        }     // end if of yield criteria
                    } break;  // end of Case DruckerPrager_Cap
                }

            }  // End of Plastic Deformation

            // Obtain stress tensor from MMX matrices - either elastic or plastic
            ChMatrixNM<double, 3, 3> StressK;
            MM1.MatrScale(StressK_eig(0, 0));
            MM2.MatrScale(StressK_eig(1, 0));
            MM3.MatrScale(StressK_eig(2, 0));
            StressK = MM1 + MM2 + MM3;

            // Influence of damping
            ChMatrixNM<double, 6, 1> DEPS;
            DEPS.Reset();
            for (int ii = 0; ii < 33; ii++) {
                DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
                DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
                DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
                DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
                DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
                DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
            }
            // Obtain stress from structural damping
            ChMatrixNM<double, 6, 1> Stress_damp;
            Stress_damp.MatrMultiply(E_eps, DEPS);
            Stress_damp.MatrScale(m_element->m_Alpha);

            // Final six stress components
            ChMatrixNM<double, 6, 1> Stress;
            Stress(0, 0) = StressK(0, 0) + Stress_damp(0, 0);
            Stress(1, 0) = StressK(1, 1) + Stress_damp(1, 0);
            Stress(2, 0) = 0.5 * (StressK(1, 0) + StressK(0, 1)) + Stress_damp(2, 0);
            Stress(3, 0) = StressK(2, 2) + Stress_damp(3, 0);
            Stress(4, 0) = 0.5 * (StressK(2, 0) + StressK(0, 2)) + Stress_damp(4, 0);
            Stress(5, 0) = 0.5 * (StressK(2, 1) + StressK(1, 2)) + Stress_damp(5, 0);

            // Declaration and computation of Sigm, to be removed
            ChMatrixNM<double, 9, 9> Sigm;
            Sigm.Reset();

            Sigm(0, 0) = Stress(0, 0);  // XX
            Sigm(1, 1) = Stress(0, 0);
            Sigm(2, 2) = Stress(0, 0);

            Sigm(0, 3) = Stress(2, 0);  // XY
            Sigm(1, 4) = Stress(2, 0);
            Sigm(2, 5) = Stress(2, 0);

            Sigm(0, 6) = Stress(4, 0);  // XZ
            Sigm(1, 7) = Stress(4, 0);
            Sigm(2, 8) = Stress(4, 0);

            Sigm(3, 0) = Stress(2, 0);  // XY
            Sigm(4, 1) = Stress(2, 0);
            Sigm(5, 2) = Stress(2, 0);

            Sigm(3, 3) = Stress(1, 0);  // YY
            Sigm(4, 4) = Stress(1, 0);
            Sigm(5, 5) = Stress(1, 0);

            Sigm(3, 6) = Stress(5, 0);  // YZ
            Sigm(4, 7) = Stress(5, 0);
            Sigm(5, 8) = Stress(5, 0);

            Sigm(6, 0) = Stress(4, 0);  // XZ
            Sigm(7, 1) = Stress(4, 0);
            Sigm(8, 2) = Stress(4, 0);

            Sigm(6, 3) = Stress(5, 0);  // YZ
            Sigm(7, 4) = Stress(5, 0);
            Sigm(8, 5) = Stress(5, 0);

            Sigm(6, 6) = Stress(3, 0);  // ZZ
            Sigm(7, 7) = Stress(3, 0);
            Sigm(8, 8) = Stress(3, 0);

            // Gd is the total deformation gradient differentiated by the coordinates (9 components diff. by 33
            // coordinates)
            ChMatrixNM<double, 9, 33> Gd;
            Gd.Reset();

            for (int ii = 0; ii < 11; ii++) {
                double Temp1 = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
                double Temp2 = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
                double Temp3 = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
                Gd(0, 3 * (ii)) = Temp1;
                Gd(1, 3 * (ii) + 1) = Temp1;
                Gd(2, 3 * (ii) + 2) = Temp1;

                Gd(3, 3 * (ii)) = Temp2;
                Gd(4, 3 * (ii) + 1) = Temp2;
                Gd(5, 3 * (ii) + 2) = Temp2;

                Gd(6, 3 * (ii)) = Temp3;
                Gd(7, 3 * (ii) + 1) = Temp3;
                Gd(8, 3 * (ii) + 2) = Temp3;
            }

            // Jacobian of internal forces (excluding the EAS contribution).
            ChMatrixNM<double, 33, 6> temp336;
            ChMatrixNM<double, 33, 9> temp339;
            if (m_element->m_Plasticity && YieldFlag == 1) {
                temp336.MatrTMultiply(strainD, Dep);  // Plastic contribution to the Jacobian of internal forces
            } else {
                temp336.MatrTMultiply(strainD, E_eps);  // Elastic contribution to the Jacobian of internal forces
            }

            temp339.MatrTMultiply(Gd, Sigm);  // Stress contribution to the Jacobian of internal forces

            m_KTE1.MatrMultiply(temp336, strainD);
            m_KTE2.MatrMultiply(temp339, Gd);

            result = m_KTE1 * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + m_KTE2 * m_Kfactor;
            result.MatrScale(detJ * m_element->m_GaussScaling);

            m_element->m_InteCounter++;
        } break;
    }
}

// Compute the Jacobian of the internal forces
void ChElementBrick_9::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    m_JacobianMatrix.Reset();
    m_InteCounter = 0;
    MyJacobianBrick9 formula(this, Kfactor, Rfactor);
    ChMatrixNM<double, 33, 33> result;
    result.Reset();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 33>>(result,   // result of integration
                                                          formula,  // integrand formula
                                                          -1, 1,    // x limits
                                                          -1, 1,    // y limits
                                                          -1, 1,    // z limits
                                                          2         // order of integration
                                                          );
    // Accumulate Jacobian
    m_JacobianMatrix += result;
}

// -----------------------------------------------------------------------------
// Interface to implicit integrators
// -----------------------------------------------------------------------------

// Update element at new time
void ChElementBrick_9::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector (column matrix) with the current states at the nodes of
// the element, with proper ordering.
void ChElementBrick_9::GetStateBlock(ChMatrixDynamic<>& mD) {
    for (int i = 0; i < 8; i++) {
        mD.PasteVector(m_nodes[i]->GetPos(), 3 * i, 0);
    }
    mD.PasteVector(m_central_node->GetCurvatureXX(), 24, 0);
    mD.PasteVector(m_central_node->GetCurvatureYY(), 27, 0);
    mD.PasteVector(m_central_node->GetCurvatureZZ(), 30, 0);
}

// Return the mass matrix.
void ChElementBrick_9::ComputeMmatrixGlobal(ChMatrix<>& M) {
    M = m_MassMatrix;
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * (M) + Kfactor * (K) + Rfactor * (R)
void ChElementBrick_9::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 33) && (H.GetColumns() == 33));

    // Calculate the linear combination Kfactor*(K) + Rfactor*(R)
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*(M) into H
    for (int i = 0; i < 33; i++)
        for (int j = 0; j < 33; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// -----------------------------------------------------------------------------
// Implementation of interface to ChLoadableUVW
// -----------------------------------------------------------------------------

// Get all the DOFs packed in a single vector (position part).
void ChElementBrick_9::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    for (int i = 0; i < 8; i++) {
        mD.PasteVector(m_nodes[i]->GetPos(), block_offset + 3 * i, 0);
    }
    mD.PasteVector(m_central_node->GetCurvatureXX(), block_offset + 24, 0);
    mD.PasteVector(m_central_node->GetCurvatureYY(), block_offset + 27, 0);
    mD.PasteVector(m_central_node->GetCurvatureZZ(), block_offset + 30, 0);
}

// Get all the DOFs packed in a single vector (speed part).
void ChElementBrick_9::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    for (int i = 0; i < 8; i++) {
        mD.PasteVector(this->m_nodes[i]->GetPos_dt(), block_offset + 3 * i, 0);
    }
    mD.PasteVector(m_central_node->GetCurvatureXX_dt(), block_offset + 24, 0);
    mD.PasteVector(m_central_node->GetCurvatureYY_dt(), block_offset + 27, 0);
    mD.PasteVector(m_central_node->GetCurvatureZZ_dt(), block_offset + 30, 0);
}

void ChElementBrick_9::LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv)  {
    for (int i = 0; i < 8; i++) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x  + 3 * i  , x_new, x, off_v  + 3 * i  , Dv);
    }
    this->m_central_node->NodeIntStateIncrement(off_x  + 24  , x_new, x, off_v  + 24 , Dv);
}

// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
void ChElementBrick_9::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < 8; ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
    }
    mvars.push_back(&m_central_node->Variables());
}

// Evaluate N'*F , where N is some type of shape function evaluated at (U,V,W).
// Here, U,V,W are coordinates of the volume, each ranging in -1..+1
// F is a load, N'*F is the resulting generalized load
// Returns also det(J) with J=(dx/du,..), that might be useful in gauss quadrature.
void ChElementBrick_9::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det(J) here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 11> N;
    ShapeFunctions(N, U, V, W);

    detJ = Calc_detJ0(U, V, W);
    detJ *= m_dimensions.x() * m_dimensions.y() * m_dimensions.z() / 8;

    ChVector<> Fv = F.ClipVector(0, 0);
    for (int i = 0; i < 11; i++) {
        Qi.PasteVector(N(i) * Fv, 3 * i, 0);
    }
}

void ChElementBrick_9::ComputeStrainD_Brick9(ChMatrixNM<double, 6, 33>& strainD,
                                             ChMatrixNM<double, 1, 11> Nx,
                                             ChMatrixNM<double, 1, 11> Ny,
                                             ChMatrixNM<double, 1, 11> Nz,
                                             ChMatrixNM<double, 3, 3> FI,
                                             ChMatrixNM<double, 3, 3> J0I) {
    strainD.Reset();
    
    double Tempx = FI(0, 0) * J0I(0, 0) + FI(1, 0) * J0I(0, 1) + FI(2, 0) * J0I(0, 2);
    double Tempy = FI(0, 0) * J0I(1, 0) + FI(1, 0) * J0I(1, 1) + FI(2, 0) * J0I(1, 2);
    double Tempz = FI(0, 0) * J0I(2, 0) + FI(1, 0) * J0I(2, 1) + FI(2, 0) * J0I(2, 2);
    double Tempx1 = FI(0, 1) * J0I(0, 0) + FI(1, 1) * J0I(0, 1) + FI(2, 1) * J0I(0, 2);
    double Tempy1 = FI(0, 1) * J0I(1, 0) + FI(1, 1) * J0I(1, 1) + FI(2, 1) * J0I(1, 2);
    double Tempz1 = FI(0, 1) * J0I(2, 0) + FI(1, 1) * J0I(2, 1) + FI(2, 1) * J0I(2, 2);
    double Tempx2 = FI(0, 2) * J0I(0, 0) + FI(1, 2) * J0I(0, 1) + FI(2, 2) * J0I(0, 2);
    double Tempy2 = FI(0, 2) * J0I(1, 0) + FI(1, 2) * J0I(1, 1) + FI(2, 2) * J0I(1, 2);
    double Tempz2 = FI(0, 2) * J0I(2, 0) + FI(1, 2) * J0I(2, 1) + FI(2, 2) * J0I(2, 2);

    strainD(0, 0) = Nx(0, 0) * (Tempx) + Ny(0, 0) * (Tempy) + Nz(0, 0) * (Tempz);
    strainD(0, 3) = Nx(0, 1) * (Tempx) + Ny(0, 1) * (Tempy) + Nz(0, 1) * (Tempz);
    strainD(0, 6) = Nx(0, 2) * (Tempx) + Ny(0, 2) * (Tempy) + Nz(0, 2) * (Tempz);
    strainD(0, 9) = Nx(0, 3) * (Tempx) + Ny(0, 3) * (Tempy) + Nz(0, 3) * (Tempz);
    strainD(0, 12) = Nx(0, 4) * (Tempx) + Ny(0, 4) * (Tempy) + Nz(0, 4) * (Tempz);
    strainD(0, 15) = Nx(0, 5) * (Tempx) + Ny(0, 5) * (Tempy) + Nz(0, 5) * (Tempz);
    strainD(0, 18) = Nx(0, 6) * (Tempx) + Ny(0, 6) * (Tempy) + Nz(0, 6) * (Tempz);
    strainD(0, 21) = Nx(0, 7) * (Tempx) + Ny(0, 7) * (Tempy) + Nz(0, 7) * (Tempz);
    strainD(0, 24) = Nx(0, 8) * (Tempx) + Ny(0, 8) * (Tempy) + Nz(0, 8) * (Tempz);
    strainD(0, 27) = Nx(0, 9) * (Tempx) + Ny(0, 9) * (Tempy) + Nz(0, 9) * (Tempz);
    strainD(0, 30) = Nx(0, 10) * (Tempx) + Ny(0, 10) * (Tempy) + Nz(0, 10) * (Tempz);
    strainD(1, 1) = Nx(0, 0) * (Tempx1) + Ny(0, 0) * (Tempy1) + Nz(0, 0) * (Tempz1);
    strainD(1, 4) = Nx(0, 1) * (Tempx1) + Ny(0, 1) * (Tempy1) + Nz(0, 1) * (Tempz1);
    strainD(1, 7) = Nx(0, 2) * (Tempx1) + Ny(0, 2) * (Tempy1) + Nz(0, 2) * (Tempz1);
    strainD(1, 10) = Nx(0, 3) * (Tempx1) + Ny(0, 3) * (Tempy1) + Nz(0, 3) * (Tempz1);
    strainD(1, 13) = Nx(0, 4) * (Tempx1) + Ny(0, 4) * (Tempy1) + Nz(0, 4) * (Tempz1);
    strainD(1, 16) = Nx(0, 5) * (Tempx1) + Ny(0, 5) * (Tempy1) + Nz(0, 5) * (Tempz1);
    strainD(1, 19) = Nx(0, 6) * (Tempx1) + Ny(0, 6) * (Tempy1) + Nz(0, 6) * (Tempz1);
    strainD(1, 22) = Nx(0, 7) * (Tempx1) + Ny(0, 7) * (Tempy1) + Nz(0, 7) * (Tempz1);
    strainD(1, 25) = Nx(0, 8) * (Tempx1) + Ny(0, 8) * (Tempy1) + Nz(0, 8) * (Tempz1);
    strainD(1, 28) = Nx(0, 9) * (Tempx1) + Ny(0, 9) * (Tempy1) + Nz(0, 9) * (Tempz1);
    strainD(1, 31) = Nx(0, 10) * (Tempx1) + Ny(0, 10) * (Tempy1) + Nz(0, 10) * (Tempz1);
    strainD(2, 0) = Nx(0, 0) * (Tempx1) + Ny(0, 0) * (Tempy1) + Nz(0, 0) * (Tempz1);
    strainD(2, 1) = Nx(0, 0) * (Tempx) + Ny(0, 0) * (Tempy) + Nz(0, 0) * (Tempz);
    strainD(2, 3) = Nx(0, 1) * (Tempx1) + Ny(0, 1) * (Tempy1) + Nz(0, 1) * (Tempz1);
    strainD(2, 4) = Nx(0, 1) * (Tempx) + Ny(0, 1) * (Tempy) + Nz(0, 1) * (Tempz);
    strainD(2, 6) = Nx(0, 2) * (Tempx1) + Ny(0, 2) * (Tempy1) + Nz(0, 2) * (Tempz1);
    strainD(2, 7) = Nx(0, 2) * (Tempx) + Ny(0, 2) * (Tempy) + Nz(0, 2) * (Tempz);
    strainD(2, 9) = Nx(0, 3) * (Tempx1) + Ny(0, 3) * (Tempy1) + Nz(0, 3) * (Tempz1);
    strainD(2, 10) = Nx(0, 3) * (Tempx) + Ny(0, 3) * (Tempy) + Nz(0, 3) * (Tempz);
    strainD(2, 12) = Nx(0, 4) * (Tempx1) + Ny(0, 4) * (Tempy1) + Nz(0, 4) * (Tempz1);
    strainD(2, 13) = Nx(0, 4) * (Tempx) + Ny(0, 4) * (Tempy) + Nz(0, 4) * (Tempz);
    strainD(2, 15) = Nx(0, 5) * (Tempx1) + Ny(0, 5) * (Tempy1) + Nz(0, 5) * (Tempz1);
    strainD(2, 16) = Nx(0, 5) * (Tempx) + Ny(0, 5) * (Tempy) + Nz(0, 5) * (Tempz);
    strainD(2, 18) = Nx(0, 6) * (Tempx1) + Ny(0, 6) * (Tempy1) + Nz(0, 6) * (Tempz1);
    strainD(2, 19) = Nx(0, 6) * (Tempx) + Ny(0, 6) * (Tempy) + Nz(0, 6) * (Tempz);
    strainD(2, 21) = Nx(0, 7) * (Tempx1) + Ny(0, 7) * (Tempy1) + Nz(0, 7) * (Tempz1);
    strainD(2, 22) = Nx(0, 7) * (Tempx) + Ny(0, 7) * (Tempy) + Nz(0, 7) * (Tempz);
    strainD(2, 24) = Nx(0, 8) * (Tempx1) + Ny(0, 8) * (Tempy1) + Nz(0, 8) * (Tempz1);
    strainD(2, 25) = Nx(0, 8) * (Tempx) + Ny(0, 8) * (Tempy) + Nz(0, 8) * (Tempz);
    strainD(2, 27) = Nx(0, 9) * (Tempx1) + Ny(0, 9) * (Tempy1) + Nz(0, 9) * (Tempz1);
    strainD(2, 28) = Nx(0, 9) * (Tempx) + Ny(0, 9) * (Tempy) + Nz(0, 9) * (Tempz);
    strainD(2, 30) = Nx(0, 10) * (Tempx1) + Ny(0, 10) * (Tempy1) + Nz(0, 10) * (Tempz1);
    strainD(2, 31) = Nx(0, 10) * (Tempx) + Ny(0, 10) * (Tempy) + Nz(0, 10) * (Tempz);
    strainD(3, 2) = Nx(0, 0) * (Tempx2) + Ny(0, 0) * (Tempy2) + Nz(0, 0) * (Tempz2);
    strainD(3, 5) = Nx(0, 1) * (Tempx2) + Ny(0, 1) * (Tempy2) + Nz(0, 1) * (Tempz2);
    strainD(3, 8) = Nx(0, 2) * (Tempx2) + Ny(0, 2) * (Tempy2) + Nz(0, 2) * (Tempz2);
    strainD(3, 11) = Nx(0, 3) * (Tempx2) + Ny(0, 3) * (Tempy2) + Nz(0, 3) * (Tempz2);
    strainD(3, 14) = Nx(0, 4) * (Tempx2) + Ny(0, 4) * (Tempy2) + Nz(0, 4) * (Tempz2);
    strainD(3, 17) = Nx(0, 5) * (Tempx2) + Ny(0, 5) * (Tempy2) + Nz(0, 5) * (Tempz2);
    strainD(3, 20) = Nx(0, 6) * (Tempx2) + Ny(0, 6) * (Tempy2) + Nz(0, 6) * (Tempz2);
    strainD(3, 23) = Nx(0, 7) * (Tempx2) + Ny(0, 7) * (Tempy2) + Nz(0, 7) * (Tempz2);
    strainD(3, 26) = Nx(0, 8) * (Tempx2) + Ny(0, 8) * (Tempy2) + Nz(0, 8) * (Tempz2);
    strainD(3, 29) = Nx(0, 9) * (Tempx2) + Ny(0, 9) * (Tempy2) + Nz(0, 9) * (Tempz2);
    strainD(3, 32) = Nx(0, 10) * (Tempx2) + Ny(0, 10) * (Tempy2) + Nz(0, 10) * (Tempz2);
    strainD(4, 0) = Nx(0, 0) * (Tempx2) + Ny(0, 0) * (Tempy2) + Nz(0, 0) * (Tempz2);
    strainD(4, 2) = Nx(0, 0) * (Tempx) + Ny(0, 0) * (Tempy) + Nz(0, 0) * (Tempz);
    strainD(4, 3) = Nx(0, 1) * (Tempx2) + Ny(0, 1) * (Tempy2) + Nz(0, 1) * (Tempz2);
    strainD(4, 5) = Nx(0, 1) * (Tempx) + Ny(0, 1) * (Tempy) + Nz(0, 1) * (Tempz);
    strainD(4, 6) = Nx(0, 2) * (Tempx2) + Ny(0, 2) * (Tempy2) + Nz(0, 2) * (Tempz2);
    strainD(4, 8) = Nx(0, 2) * (Tempx) + Ny(0, 2) * (Tempy) + Nz(0, 2) * (Tempz);
    strainD(4, 9) = Nx(0, 3) * (Tempx2) + Ny(0, 3) * (Tempy2) + Nz(0, 3) * (Tempz2);
    strainD(4, 11) = Nx(0, 3) * (Tempx) + Ny(0, 3) * (Tempy) + Nz(0, 3) * (Tempz);
    strainD(4, 12) = Nx(0, 4) * (Tempx2) + Ny(0, 4) * (Tempy2) + Nz(0, 4) * (Tempz2);
    strainD(4, 14) = Nx(0, 4) * (Tempx) + Ny(0, 4) * (Tempy) + Nz(0, 4) * (Tempz);
    strainD(4, 15) = Nx(0, 5) * (Tempx2) + Ny(0, 5) * (Tempy2) + Nz(0, 5) * (Tempz2);
    strainD(4, 17) = Nx(0, 5) * (Tempx) + Ny(0, 5) * (Tempy) + Nz(0, 5) * (Tempz);
    strainD(4, 18) = Nx(0, 6) * (Tempx2) + Ny(0, 6) * (Tempy2) + Nz(0, 6) * (Tempz2);
    strainD(4, 20) = Nx(0, 6) * (Tempx) + Ny(0, 6) * (Tempy) + Nz(0, 6) * (Tempz);
    strainD(4, 21) = Nx(0, 7) * (Tempx2) + Ny(0, 7) * (Tempy2) + Nz(0, 7) * (Tempz2);
    strainD(4, 23) = Nx(0, 7) * (Tempx) + Ny(0, 7) * (Tempy) + Nz(0, 7) * (Tempz);
    strainD(4, 24) = Nx(0, 8) * (Tempx2) + Ny(0, 8) * (Tempy2) + Nz(0, 8) * (Tempz2);
    strainD(4, 26) = Nx(0, 8) * (Tempx) + Ny(0, 8) * (Tempy) + Nz(0, 8) * (Tempz);
    strainD(4, 27) = Nx(0, 9) * (Tempx2) + Ny(0, 9) * (Tempy2) + Nz(0, 9) * (Tempz2);
    strainD(4, 29) = Nx(0, 9) * (Tempx) + Ny(0, 9) * (Tempy) + Nz(0, 9) * (Tempz);
    strainD(4, 30) = Nx(0, 10) * (Tempx2) + Ny(0, 10) * (Tempy2) + Nz(0, 10) * (Tempz2);
    strainD(4, 32) = Nx(0, 10) * (Tempx) + Ny(0, 10) * (Tempy) + Nz(0, 10) * (Tempz);
    strainD(5, 1) = Nx(0, 0) * (Tempx2) + Ny(0, 0) * (Tempy2) + Nz(0, 0) * (Tempz2);
    strainD(5, 2) = Nx(0, 0) * (Tempx1) + Ny(0, 0) * (Tempy1) + Nz(0, 0) * (Tempz1);
    strainD(5, 4) = Nx(0, 1) * (Tempx2) + Ny(0, 1) * (Tempy2) + Nz(0, 1) * (Tempz2);
    strainD(5, 5) = Nx(0, 1) * (Tempx1) + Ny(0, 1) * (Tempy1) + Nz(0, 1) * (Tempz1);
    strainD(5, 7) = Nx(0, 2) * (Tempx2) + Ny(0, 2) * (Tempy2) + Nz(0, 2) * (Tempz2);
    strainD(5, 8) = Nx(0, 2) * (Tempx1) + Ny(0, 2) * (Tempy1) + Nz(0, 2) * (Tempz1);
    strainD(5, 10) = Nx(0, 3) * (Tempx2) + Ny(0, 3) * (Tempy2) + Nz(0, 3) * (Tempz2);
    strainD(5, 11) = Nx(0, 3) * (Tempx1) + Ny(0, 3) * (Tempy1) + Nz(0, 3) * (Tempz1);
    strainD(5, 13) = Nx(0, 4) * (Tempx2) + Ny(0, 4) * (Tempy2) + Nz(0, 4) * (Tempz2);
    strainD(5, 14) = Nx(0, 4) * (Tempx1) + Ny(0, 4) * (Tempy1) + Nz(0, 4) * (Tempz1);
    strainD(5, 16) = Nx(0, 5) * (Tempx2) + Ny(0, 5) * (Tempy2) + Nz(0, 5) * (Tempz2);
    strainD(5, 17) = Nx(0, 5) * (Tempx1) + Ny(0, 5) * (Tempy1) + Nz(0, 5) * (Tempz1);
    strainD(5, 19) = Nx(0, 6) * (Tempx2) + Ny(0, 6) * (Tempy2) + Nz(0, 6) * (Tempz2);
    strainD(5, 20) = Nx(0, 6) * (Tempx1) + Ny(0, 6) * (Tempy1) + Nz(0, 6) * (Tempz1);
    strainD(5, 22) = Nx(0, 7) * (Tempx2) + Ny(0, 7) * (Tempy2) + Nz(0, 7) * (Tempz2);
    strainD(5, 23) = Nx(0, 7) * (Tempx1) + Ny(0, 7) * (Tempy1) + Nz(0, 7) * (Tempz1);
    strainD(5, 25) = Nx(0, 8) * (Tempx2) + Ny(0, 8) * (Tempy2) + Nz(0, 8) * (Tempz2);
    strainD(5, 26) = Nx(0, 8) * (Tempx1) + Ny(0, 8) * (Tempy1) + Nz(0, 8) * (Tempz1);
    strainD(5, 28) = Nx(0, 9) * (Tempx2) + Ny(0, 9) * (Tempy2) + Nz(0, 9) * (Tempz2);
    strainD(5, 29) = Nx(0, 9) * (Tempx1) + Ny(0, 9) * (Tempy1) + Nz(0, 9) * (Tempz1);
    strainD(5, 31) = Nx(0, 10) * (Tempx2) + Ny(0, 10) * (Tempy2) + Nz(0, 10) * (Tempz2);
    strainD(5, 32) = Nx(0, 10) * (Tempx1) + Ny(0, 10) * (Tempy1) + Nz(0, 10) * (Tempz1);
}

double ChElementBrick_9::Calc_detJ0(double x,
                                    double y,
                                    double z,
                                    ChMatrixNM<double, 1, 11>& Nx,
                                    ChMatrixNM<double, 1, 11>& Ny,
                                    ChMatrixNM<double, 1, 11>& Nz,
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

double ChElementBrick_9::Calc_detJ0(double x, double y, double z) {
    ChMatrixNM<double, 1, 11> Nx;
    ChMatrixNM<double, 1, 11> Ny;
    ChMatrixNM<double, 1, 11> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;

    return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
}

void ChElementBrick_9::CalcCoordMatrix(ChMatrixNM<double, 11, 3>& d) {
    for (int i = 0; i < 8; i++) {
        const ChVector<>& pos = m_nodes[i]->GetPos();
        d(i, 0) = pos.x();
        d(i, 1) = pos.y();
        d(i, 2) = pos.z();
    }

    const ChVector<>& rxx = m_central_node->GetCurvatureXX();
    const ChVector<>& ryy = m_central_node->GetCurvatureYY();
    const ChVector<>& rzz = m_central_node->GetCurvatureZZ();

    d(8, 0) = rxx.x();
    d(8, 1) = rxx.y();
    d(8, 2) = rxx.z();

    d(9, 0) = ryy.x();
    d(9, 1) = ryy.y();
    d(9, 2) = ryy.z();

    d(10, 0) = rzz.x();
    d(10, 1) = rzz.y();
    d(10, 2) = rzz.z();
}

void ChElementBrick_9::CalcCoordDerivMatrix(ChMatrixNM<double, 33, 1>& dt) {
    for (int i = 0; i < 8; i++) {
        const ChVector<>& vel = m_nodes[i]->GetPos_dt();
        dt(3 * i, 0) = vel.x();
        dt(3 * i + 1, 0) = vel.y();
        dt(3 * i + 2, 0) = vel.z();
    }

    dt(24, 0) = m_central_node->GetCurvatureXX_dt().x();
    dt(25, 0) = m_central_node->GetCurvatureXX_dt().y();
    dt(26, 0) = m_central_node->GetCurvatureXX_dt().z();
    dt(27, 0) = m_central_node->GetCurvatureYY_dt().x();
    dt(28, 0) = m_central_node->GetCurvatureYY_dt().y();
    dt(29, 0) = m_central_node->GetCurvatureYY_dt().z();
    dt(30, 0) = m_central_node->GetCurvatureZZ_dt().x();
    dt(31, 0) = m_central_node->GetCurvatureZZ_dt().y();
    dt(32, 0) = m_central_node->GetCurvatureZZ_dt().z();
}
void ChElementBrick_9::ComputeHardening_a(double& MeanEffP,
                                          double& Hi,
                                          double alphUp,
                                          ChVectorDynamic<double> m_DPVector1,
                                          ChVectorDynamic<double> m_DPVector2,
                                          int m_DPVector_size) {
    // Given a value of alphUp return a value of MeanEffP based on interpolation
    // within a table of m_DPVector2 values(ytab) corresponding to the m_DPVector1 values
    // contained in the array xtab.The subroutine assumes that the
    // values in xtab increase monotonically.
    int i1;
    int flag;
    double wx;
    double x;
    flag = 0;
    x = alphUp;
    if (x < m_DPVector1(0)) {  // is below the table range
        flag = 2;
    } else {
        for (i1 = 0; i1 < m_DPVector_size - 1; i1++) {
            if (x <= m_DPVector1(i1 + 1)) {
                break;
            }
        }
        if (x > m_DPVector1(m_DPVector_size - 1)) {  // is above the table range
            flag = 1;
        }
    }

    if (flag == 0) {  // Inter range
        wx = (x - m_DPVector1(i1)) / (m_DPVector1(i1 + 1) - m_DPVector1(i1));
        MeanEffP = (1.0 - wx) * m_DPVector2(i1) + wx * m_DPVector2(i1 + 1);
        Hi = (m_DPVector2(i1 + 1) - m_DPVector2(i1)) / (m_DPVector1(i1 + 1) - m_DPVector1(i1));
    }
    if (flag == 1) {  // outer range above
        MeanEffP =
            (m_DPVector2(i1 + 1) - m_DPVector2(i1)) / (m_DPVector1(i1 + 1) - m_DPVector1(i1)) * (x - m_DPVector1(i1)) +
            m_DPVector2(i1);
        Hi = (m_DPVector2(i1 + 1) - m_DPVector2(i1)) / (m_DPVector1(i1 + 1) - m_DPVector1(i1));
    }
    if (flag == 2) {  // outer range below
        MeanEffP = (m_DPVector2(1) - m_DPVector2(0)) / (m_DPVector1(1) - m_DPVector1(0)) * (x - m_DPVector1(0)) +
                   m_DPVector2(0);
        Hi = (m_DPVector2(1) - m_DPVector2(0)) / (m_DPVector1(1) - m_DPVector1(0));
    }
}

}  // end namespace fea
}  // end namespace chrono
