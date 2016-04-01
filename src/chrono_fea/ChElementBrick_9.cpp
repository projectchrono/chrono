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
ChElementBrick_9::ChElementBrick_9() {
    m_nodes.resize(8);
    //// TODO
}
// -----------------------------------------------------------------------------
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
    ////mvars.push_back(&m_central_node->Variables());

    Kmatr.SetVariables(mvars);
}

// -----------------------------------------------------------------------------

void ChElementBrick_9::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    //// TODO
}

void ChElementBrick_9::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    //// TODO
}

void ChElementBrick_9::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    //// TODO
}

void ChElementBrick_9::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    //// TODO
}

// ----------------------------------------------------------------------------

void ChElementBrick_9::Update() {
    ChElementGeneric::Update();
    //// TODO
}

// -----------------------------------------------------------------------------

void ChElementBrick_9::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), 3, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), 6, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), 9, 0);
    mD.PasteVector(m_nodes[4]->GetPos(), 12, 0);
    mD.PasteVector(m_nodes[5]->GetPos(), 15, 0);
    mD.PasteVector(m_nodes[6]->GetPos(), 18, 0);
    mD.PasteVector(m_nodes[7]->GetPos(), 21, 0);
    //// TODO
}

// -----------------------------------------------------------------------------

// Private class for quadrature of the mass matrix.
class MyMassBrick9 : public ChIntegrable3D<ChMatrixNM<double, 33, 33>> {
  public:
    MyMassBrick9();  ///< Constructor
    MyMassBrick9(ChElementBrick_9* element) : m_element(element) {
        //// TODO
    }
    ~MyMassBrick9() {}

  private:
    virtual void Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) override;

    ChElementBrick_9* m_element;
    //// TODO
};

void MyMassBrick9::Evaluate(ChMatrixNM<double, 33, 33>& result, const double x, const double y, const double z) {
    //// TODO
}

// Compute the mass matrix of the element.
void ChElementBrick_9::ComputeMassMatrix() {
    //// TODO
}

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

void MyGravityBrick9::Evaluate(ChMatrixNM<double, 33, 1>& result, const double x, const double y, const double z) {
    //// TODO
}

// Compute the gravitational forces.
void ChElementBrick_9::ComputeGravityForce(const ChVector<>& g_acc) {
    m_GravForce.Reset();

    MyGravityBrick9 myformula1(this, g_acc);
    ChQuadrature::Integrate3D<ChMatrixNM<double, 33, 1>>(m_GravForce,  // result of integration will go there
                                                         myformula1,   // formula to integrate
                                                         -1, 1,        // limits in x direction
                                                         -1, 1,        // limits in y direction
                                                         -1, 1,        // limits in z direction
                                                         2             // order of integration
                                                         );

    m_GravForce *= m_material->Get_density();
}

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

void MyForceBrick9::Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) {
    //// TODO
}

// Compute internal forces and load them in the Fi vector.
void ChElementBrick_9::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    //// TODO
}

// -----------------------------------------------------------------------------

// Private class for quadrature of the Jacobian of internal forces
class MyJacobianBrick9 : public ChIntegrable3D<ChMatrixNM<double, 999, 1>> {
  public:
    MyJacobianBrick9(ChElementBrick_9* element,  // Containing element
                     double Kfactor,             // Scaling coefficient for stiffness component
                     double Rfactor              // Scaling coefficient for damping component
                     )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor) {
        //// TODO
    }

  private:
    ChElementBrick_9* m_element;
    double m_Kfactor;
    double m_Rfactor;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) override;
};

void MyJacobianBrick9::Evaluate(ChMatrixNM<double, 999, 1>& result, const double x, const double y, const double z) {
    //// TODO
}

void ChElementBrick_9::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    //// TODO
}

// -----------------------------------------------------------------------------

void ChElementBrick_9::SetupInitial(ChSystem* system) {
    ComputeGravityForce(system->Get_G_acc());
    ComputeMassMatrix();
    //// TODO more here?
}

// -----------------------------------------------------------------------------

void ChElementBrick_9::ComputeMmatrixGlobal(ChMatrix<>& M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------

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

void ChElementBrick_9::LoadableGetVariables(std::vector<ChLcpVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i)
        mvars.push_back(&this->m_nodes[i]->Variables());

    //// TODO
}

// -----------------------------------------------------------------------------

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
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
