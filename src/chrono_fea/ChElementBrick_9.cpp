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
// Authors: Radu Serban
// =============================================================================
// Brick element with 9 nodes (central node for curvature)
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono_fea/ChElementBrick_9.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

ChElementBrick_9::ChElementBrick_9() : m_gravity_on(false) {
    m_nodes.resize(8);
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

    std::vector<ChLcpVariables*> mvars;
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

    CalcCoordMatrix(m_d0);
    //// TODO any other initializations here?
}

// Initial element setup
void ChElementBrick_9::SetupInitial(ChSystem* system) {
    //// TODO any other initializations go here

    ComputeGravityForce(system->Get_G_acc());
    ComputeMassMatrix();
}

// -----------------------------------------------------------------------------
// Calculation of shape functions and their derivatives
// -----------------------------------------------------------------------------

void ChElementBrick_9::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    N(0) = 0.125 * (1 - x) * (1 - y) * (1 - z);
    N(1) = 0.125 * (1 + x) * (1 - y) * (1 - z);
    N(2) = 0.125 * (1 - x) * (1 + y) * (1 - z);
    N(3) = 0.125 * (1 + x) * (1 + y) * (1 - z);
    N(4) = 0.125 * (1 - x) * (1 - y) * (1 + z);
    N(5) = 0.125 * (1 + x) * (1 - y) * (1 + z);
    N(6) = 0.125 * (1 - x) * (1 + y) * (1 + z);
    N(7) = 0.125 * (1 + x) * (1 + y) * (1 + z);
    //
    N(8) = 0.5 * (x * x - 1);
    N(9) = 0.5 * (y * y - 1);
    N(10) = 0.5 * (z * z - 1);
}

void ChElementBrick_9::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double factor = 2 / GetDimensions().x;

    Nx(0) = 0.125 * factor * (-1) * (1 - y) * (1 - z);
    Nx(1) = 0.125 * factor * (+1) * (1 - y) * (1 - z);
    Nx(2) = 0.125 * factor * (-1) * (1 + y) * (1 - z);
    Nx(3) = 0.125 * factor * (+1) * (1 + y) * (1 - z);
    Nx(4) = 0.125 * factor * (-1) * (1 - y) * (1 + z);
    Nx(5) = 0.125 * factor * (+1) * (1 - y) * (1 + z);
    Nx(6) = 0.125 * factor * (-1) * (1 + y) * (1 + z);
    Nx(7) = 0.125 * factor * (+1) * (1 + y) * (1 + z);
    //
    Nx(8) = 0.5 * factor * (2 * x);
    Nx(9) = 0;
    Nx(10) = 0;
}

void ChElementBrick_9::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double factor = 2 / GetDimensions().y;

    Ny(0) = 0.125 * factor * (1 - x) * (-1) * (1 - z);
    Ny(1) = 0.125 * factor * (1 + x) * (-1) * (1 - z);
    Ny(2) = 0.125 * factor * (1 - x) * (+1) * (1 - z);
    Ny(3) = 0.125 * factor * (1 + x) * (+1) * (1 - z);
    Ny(4) = 0.125 * factor * (1 - x) * (-1) * (1 + z);
    Ny(5) = 0.125 * factor * (1 + x) * (-1) * (1 + z);
    Ny(6) = 0.125 * factor * (1 - x) * (+1) * (1 + z);
    Ny(7) = 0.125 * factor * (1 + x) * (+1) * (1 + z);
    //
    Ny(8) = 0;
    Ny(9) = 0.5 * factor * (2 * y);
    Ny(10) = 0;
}

void ChElementBrick_9::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    double factor = 2 / GetDimensions().z;
    
    Nz(0) = 0.125 * factor * (1 - x) * (1 - y) * (-1);
    Nz(1) = 0.125 * factor * (1 + x) * (1 - y) * (-1);
    Nz(2) = 0.125 * factor * (1 - x) * (1 + y) * (-1);
    Nz(3) = 0.125 * factor * (1 + x) * (1 + y) * (-1);
    Nz(4) = 0.125 * factor * (1 - x) * (1 - y) * (+1);
    Nz(5) = 0.125 * factor * (1 + x) * (1 - y) * (+1);
    Nz(6) = 0.125 * factor * (1 - x) * (1 + y) * (+1);
    Nz(7) = 0.125 * factor * (1 + x) * (1 + y) * (+1);
    //
    Nz(8) = 0;
    Nz(9) = 0;
    Nz(10) = 0.5 * factor * (2 * z);
}

// -----------------------------------------------------------------------------
// Calculation of the mass matrix
// -----------------------------------------------------------------------------

// Private class for quadrature of the mass matrix.
class MyMassBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
  public:
    MyMassBrick9(ChElementBrick_9* element) : m_element(element) {
        //// TODO
    }

  private:
    virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;

    ChElementBrick_9* m_element;
    //// TODO
};

void MyMassBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);
    //// TODO
}

// Compute the mass matrix of the element.
void ChElementBrick_9::ComputeMassMatrix() {
    m_MassMatrix.Reset();

    MyMassBrick9 myformula(this);

    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 33>>(m_MassMatrix,  // result of integration will go there
                                                          myformula,     // formula to integrate
                                                          -1, 1,         // limits in x direction
                                                          -1, 1,         // limits in y direction
                                                          -1, 1,         // limits in z direction
                                                          2              // order of integration
                                                          );

    m_MassMatrix *= m_material->Get_density();
}

// -----------------------------------------------------------------------------
// Calculation of gravitational forces
// -----------------------------------------------------------------------------

// Private class for quadrature of gravitational forces.
class MyGravityBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 1>> {
  public:
    MyGravityBrick9(ChElementBrick_9* element, const ChVector<>& gacc) : m_element(element), m_gacc(gacc) {
        //// TODO
    }

  private:
    virtual void Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) override;

    ChElementBrick_9* m_element;
    ChVector<> m_gacc;
    //// TODO
};

// Evaluate integrand at the specified point
void MyGravityBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);
    //// TODO
}

// Compute the gravitational forces.
void ChElementBrick_9::ComputeGravityForce(const ChVector<>& g_acc) {
    m_GravForce.Reset();

    MyGravityBrick9 myformula(this, g_acc);
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
class MyForceBrick9 : public ChIntegrable3D<ChMatrixNM<double, 999, 1>> {
  public:
    MyForceBrick9(ChElementBrick_9* element) : m_element(element) {
        //// TODO
    }

  private:
    virtual void Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) override;

    ChElementBrick_9* m_element;
    //// TODO
};

// Evaluate integrand at the specified point
void MyForceBrick9::Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);
    //// TODO
}

// Compute internal forces and load them in the Fi vector.
void ChElementBrick_9::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    Fi.Reset();

    //// TODO

    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------
// Calculation of the Jacobian of internal forces
// -----------------------------------------------------------------------------

// Private class for quadrature of the Jacobian of internal forces
class MyJacobianBrick9 : public ChIntegrable3D<ChMatrixNM<double, 999, 1>> {
  public:
    MyJacobianBrick9(ChElementBrick_9* element,  // Associated element
                     double Kfactor,             // Scaling coefficient for stiffness component
                     double Rfactor              // Scaling coefficient for damping component
                     )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {
        //// TODO
    }

  private:
    virtual void Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) override;

    ChElementBrick_9* m_element;
    double m_Kfactor;
    double m_Rfactor;
    //// TODO
};

// Evaluate integrand at the specified point
void MyJacobianBrick9::Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 11> N;
    m_element->ShapeFunctions(N, x, y, z);
    //// TODO
}

// Compute the Jacobian of the internal forces
void ChElementBrick_9::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    m_JacobianMatrix.Reset();
    //// TODO
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
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementBrick_9::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 33) && (H.GetColumns() == 33));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 33; i++)
        for (int j = 0; j < 33; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// -----------------------------------------------------------------------------
// Implementation of interface to ChLoadableUVW
// -----------------------------------------------------------------------------

// Get all the DOFs packed in a single vector (position part).
void ChElementBrick_9::LoadableGetStateBlock_x(int block_offset, ChVectorDynamic<>& mD) {
    for (int i = 0; i < 8; i++) {
        mD.PasteVector(m_nodes[i]->GetPos(), block_offset + 3 * i, 0);
    }
    mD.PasteVector(m_central_node->GetCurvatureXX(), block_offset + 24, 0);
    mD.PasteVector(m_central_node->GetCurvatureYY(), block_offset + 27, 0);
    mD.PasteVector(m_central_node->GetCurvatureZZ(), block_offset + 30, 0);
}

// Get all the DOFs packed in a single vector (speed part).
void ChElementBrick_9::LoadableGetStateBlock_w(int block_offset, ChVectorDynamic<>& mD) {
    for (int i = 0; i < 8; i++) {
        mD.PasteVector(this->m_nodes[i]->GetPos_dt(), block_offset + 3 * i, 0);
    }
    mD.PasteVector(m_central_node->GetCurvatureXX_dt(), block_offset + 24, 0);
    mD.PasteVector(m_central_node->GetCurvatureYY_dt(), block_offset + 27, 0);
    mD.PasteVector(m_central_node->GetCurvatureZZ_dt(), block_offset + 30, 0);
}

// Get the pointers to the contained ChLcpVariables, appending to the mvars vector.
void ChElementBrick_9::LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
    for (int i = 0; i < 8; ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
    }
    mvars.push_back(&m_central_node->Variables());
}

// Evaluate N'*F , where N is some type of shape function evaluated at (U,V,W).
// Here, U,V,W are coordinates of the volume, each ranging in -1..+1
// F is a load, N'*F is the resulting generalized load
// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
void ChElementBrick_9::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 11> N;
    ShapeFunctions(N, U, V, W);

    detJ = Calc_detJ0(U, V, W);
    detJ *= m_dimensions.x * m_dimensions.y * m_dimensions.z / 8;

    ChVector<> Fv = F.ClipVector(0, 0);
    for (int i = 0; i < 11; i++) {
        Qi.PasteVector(N(i) * Fv, 3 * i, 0);
    }
}

// -----------------------------------------------------------------------------
// Functions for internal computations
// -----------------------------------------------------------------------------

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
        d(i, 0) = pos.x;
        d(i, 1) = pos.y;
        d(i, 2) = pos.z;
    }

    const ChVector<>& rxx = m_central_node->GetCurvatureXX();
    const ChVector<>& ryy = m_central_node->GetCurvatureYY();
    const ChVector<>& rzz = m_central_node->GetCurvatureZZ();

    d(8, 0) = rxx.x;
    d(8, 1) = rxx.y;
    d(8, 2) = rxx.z;

    d(9, 0) = ryy.x;
    d(9, 1) = ryy.y;
    d(9, 2) = ryy.z;

    d(10, 0) = rzz.x;
    d(10, 1) = rzz.y;
    d(10, 2) = rzz.z;
}

}  // end namespace fea
}  // end namespace chrono
