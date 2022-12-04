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
// Authors: Michael Taylor, Antonio Recuero, Radu Serban
// =============================================================================
// Fully Parameterized ANCF brick element with 8 nodes (96DOF). A Description of this element can be found in:
// Olshevskiy, A., Dmitrochenko, O., & Kim, C. W. (2014). Three-dimensional solid brick element using slopes in the
// absolute nodal coordinate formulation. Journal of Computational and Nonlinear Dynamics, 9(2).
// =============================================================================
// The "Continuous Integration" style calculation for the generalized internal force is based on modifications to
// (including a new analytical Jacobian):  Gerstmayr, J., Shabana, A.A.: Efficient integration of the elastic forces and
// thin three-dimensional beam elements in the absolute nodal coordinate formulation.In: Proceedings of the Multibody
// Dynamics Eccomas thematic Conference, Madrid(2005).
//
// The "Pre-Integration" style calculation is based on modifications
// to Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody system composed of
// composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.
//
// A report covering the detailed mathematics and implementation both of these generalized internal force calculations
// and their Jacobians can be found in: Taylor, M.: Technical Report TR-2020-09 Efficient CPU Based Calculations of the
// Generalized Internal Forces and Jacobian of the Generalized Internal Forces for ANCF Continuum Mechanics Elements
// with Linear Viscoelastic Materials, Simulation Based Engineering Lab, University of Wisconsin-Madison; 2021.
// =============================================================================
// This element class has been templatized by the number of Gauss quadrature points to use for the generalized internal
// force calculations and its Jacobian with the recommended values as the default.  Using fewer than 3 Gauss quadrature
// will likely result in numerical issues with the element.
// =============================================================================

#include "chrono/fea/ChElementHexaANCF_3843.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementHexaANCF_3843::ChElementHexaANCF_3843()
    : m_method(IntFrcMethod::ContInt),
      m_lenX(0),
      m_lenY(0),
      m_lenZ(0),
      m_Alpha(0),
      m_damping_enabled(false) {
    m_nodes.resize(8);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementHexaANCF_3843::SetNodes(std::shared_ptr<ChNodeFEAxyzDDD> nodeA,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeB,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeC,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeD,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeE,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeF,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeG,
                                      std::shared_ptr<ChNodeFEAxyzDDD> nodeH) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);
    assert(nodeD);
    assert(nodeE);
    assert(nodeF);
    assert(nodeG);
    assert(nodeH);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;
    m_nodes[4] = nodeE;
    m_nodes[5] = nodeF;
    m_nodes[6] = nodeG;
    m_nodes[7] = nodeH;

    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[0]->Variables_DD());
    mvars.push_back(&m_nodes[0]->Variables_DDD());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables_DD());
    mvars.push_back(&m_nodes[1]->Variables_DDD());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[2]->Variables_D());
    mvars.push_back(&m_nodes[2]->Variables_DD());
    mvars.push_back(&m_nodes[2]->Variables_DDD());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[3]->Variables_D());
    mvars.push_back(&m_nodes[3]->Variables_DD());
    mvars.push_back(&m_nodes[3]->Variables_DDD());
    mvars.push_back(&m_nodes[4]->Variables());
    mvars.push_back(&m_nodes[4]->Variables_D());
    mvars.push_back(&m_nodes[4]->Variables_DD());
    mvars.push_back(&m_nodes[4]->Variables_DDD());
    mvars.push_back(&m_nodes[5]->Variables());
    mvars.push_back(&m_nodes[5]->Variables_D());
    mvars.push_back(&m_nodes[5]->Variables_DD());
    mvars.push_back(&m_nodes[5]->Variables_DDD());
    mvars.push_back(&m_nodes[6]->Variables());
    mvars.push_back(&m_nodes[6]->Variables_D());
    mvars.push_back(&m_nodes[6]->Variables_DD());
    mvars.push_back(&m_nodes[6]->Variables_DDD());
    mvars.push_back(&m_nodes[7]->Variables());
    mvars.push_back(&m_nodes[7]->Variables_D());
    mvars.push_back(&m_nodes[7]->Variables_DD());
    mvars.push_back(&m_nodes[7]->Variables_DDD());

    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    // These values define the reference configuration of the element
    CalcCoordMatrix(m_ebar0);

    // Check to see if SetupInitial has already been called (i.e. at least one set of precomputed matrices has been
    // populated).  If so, the precomputed matrices will need to be re-generated.  If not, this will be handled once
    // SetupInitial is called.
    if (m_SD.size() + m_O1.size() > 0) {
        PrecomputeInternalForceMatricesWeights();
    }
}

// -----------------------------------------------------------------------------
// Element Settings
// -----------------------------------------------------------------------------

// Specify the element dimensions.

void ChElementHexaANCF_3843::SetDimensions(double lenX, double lenY, double lenZ) {
    m_lenX = lenX;
    m_lenY = lenY;
    m_lenZ = lenZ;

    // Check to see if SetupInitial has already been called (i.e. at least one set of precomputed matrices has been
    // populated).  If so, the precomputed matrices will need to be re-generated.  If not, this will be handled once
    // SetupInitial is called.
    if (m_SD.size() + m_O1.size() > 0) {
        PrecomputeInternalForceMatricesWeights();
    }
}

// Specify the element material.

void ChElementHexaANCF_3843::SetMaterial(std::shared_ptr<ChMaterialHexaANCF> brick_mat) {
    m_material = brick_mat;

    // Check to see if the Pre-Integration method is selected and if the precomputed matrices have already been
    // generated and thus need to be regenerated.  The Continuous Integration precomputed matrices do not include any
    // material properties.
    if (m_method == IntFrcMethod::PreInt) {
        if (m_O1.size() > 0) {
            PrecomputeInternalForceMatricesWeights();
        }
    }
}

// Set the value for the single term structural damping coefficient.

void ChElementHexaANCF_3843::SetAlphaDamp(double a) {
    m_Alpha = a;
    if (std::abs(m_Alpha) > 1e-10)
        m_damping_enabled = true;
    else
        m_damping_enabled = false;
}

// Change the method used to compute the generalized internal force vector and its Jacobian.

void ChElementHexaANCF_3843::SetIntFrcCalcMethod(IntFrcMethod method) {
    m_method = method;

    // Check to see if SetupInitial has already been called (i.e. at least one set of precomputed matrices has been
    // populated).  If so, the precomputed matrices will need to be generated if they do not already exist or
    // regenerated if they have to ensure they are in sync with any other element changes.
    if (m_SD.size() + m_O1.size() > 0) {
        PrecomputeInternalForceMatricesWeights();
    }
}

// -----------------------------------------------------------------------------
// Evaluate Strains and Stresses
// -----------------------------------------------------------------------------
// These functions are designed for single function calls.  If these values are needed at the same points in the element
// through out the simulation, then the adjusted normalized shape function derivative matrix (Sxi_D) for each query
// point should be cached and saved to increase the execution speed
// -----------------------------------------------------------------------------

// Get the Green-Lagrange strain tensor at the normalized element coordinates (xi, eta, zeta) [-1...1]

ChMatrix33<> ChElementHexaANCF_3843::GetGreenLagrangeStrain(const double xi, const double eta, const double zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    ChMatrix33<double> J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_ebar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    ChMatrix33<> I3x3;
    I3x3.setIdentity();
    return 0.5 * (F.transpose() * F - I3x3);
}

// Get the 2nd Piola-Kirchoff stress tensor at the normalized element coordinates (xi, eta, zeta) [-1...1] at the
// current state of the element.

ChMatrix33<> ChElementHexaANCF_3843::GetPK2Stress(const double xi, const double eta, const double zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    ChMatrix33<double> J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_ebar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    // Calculate the Green-Lagrange strain tensor at the current point in Voigt notation
    ChVectorN<double, 6> epsilon_combined;
    epsilon_combined(0) = 0.5 * (F.col(0).dot(F.col(0)) - 1);
    epsilon_combined(1) = 0.5 * (F.col(1).dot(F.col(1)) - 1);
    epsilon_combined(2) = 0.5 * (F.col(2).dot(F.col(2)) - 1);
    epsilon_combined(3) = F.col(1).dot(F.col(2));
    epsilon_combined(4) = F.col(0).dot(F.col(2));
    epsilon_combined(5) = F.col(0).dot(F.col(1));

    if (m_damping_enabled) {
        Matrix3xN ebardot;  // Element coordinate time derivatives in matrix form
        CalcCoordDerivMatrix(ebardot);

        // Calculate the time derivative of the Deformation Gradient at the current point
        ChMatrixNM_col<double, 3, 3> Fdot = ebardot * Sxi_D;

        // Calculate the time derivative of the Green-Lagrange strain tensor in Voigt notation
        // and combine it with epsilon assuming a Linear Kelvin-Voigt Viscoelastic material model
        epsilon_combined(0) += m_Alpha * F.col(0).dot(Fdot.col(0));
        epsilon_combined(1) += m_Alpha * F.col(1).dot(Fdot.col(1));
        epsilon_combined(2) += m_Alpha * F.col(2).dot(Fdot.col(2));
        epsilon_combined(3) += m_Alpha * (F.col(1).dot(Fdot.col(2)) + Fdot.col(1).dot(F.col(2)));
        epsilon_combined(4) += m_Alpha * (F.col(0).dot(Fdot.col(2)) + Fdot.col(0).dot(F.col(2)));
        epsilon_combined(5) += m_Alpha * (F.col(0).dot(Fdot.col(1)) + Fdot.col(0).dot(F.col(1)));
    }

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    ChVectorN<double, 6> sigmaPK2 = D * epsilon_combined;  // 2nd Piola Kirchhoff Stress tensor in Voigt notation

    ChMatrix33<> SPK2;
    SPK2(0, 0) = sigmaPK2(0);
    SPK2(1, 1) = sigmaPK2(1);
    SPK2(2, 2) = sigmaPK2(2);
    SPK2(1, 2) = sigmaPK2(3);
    SPK2(2, 1) = sigmaPK2(3);
    SPK2(0, 2) = sigmaPK2(4);
    SPK2(2, 0) = sigmaPK2(4);
    SPK2(0, 1) = sigmaPK2(5);
    SPK2(1, 0) = sigmaPK2(5);

    return SPK2;
}

// Get the von Mises stress value at the normalized element coordinates (xi, eta, zeta) [-1...1] at the current
// state of the element.

double ChElementHexaANCF_3843::GetVonMissesStress(const double xi, const double eta, const double zeta) {
    MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    ChMatrix33<double> J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
    J_0xi.noalias() = m_ebar0 * Sxi_D;

    Sxi_D = Sxi_D * J_0xi.inverse();  // Adjust the shape function derivative matrix to account for the potentially
                                      // distorted reference configuration

    Matrix3xN e_bar;  // Element coordinates in matrix form
    CalcCoordMatrix(e_bar);

    // Calculate the Deformation Gradient at the current point
    ChMatrixNM_col<double, 3, 3> F = e_bar * Sxi_D;

    // Calculate the Green-Lagrange strain tensor at the current point in Voigt notation
    ChVectorN<double, 6> epsilon_combined;
    epsilon_combined(0) = 0.5 * (F.col(0).dot(F.col(0)) - 1);
    epsilon_combined(1) = 0.5 * (F.col(1).dot(F.col(1)) - 1);
    epsilon_combined(2) = 0.5 * (F.col(2).dot(F.col(2)) - 1);
    epsilon_combined(3) = F.col(1).dot(F.col(2));
    epsilon_combined(4) = F.col(0).dot(F.col(2));
    epsilon_combined(5) = F.col(0).dot(F.col(1));

    if (m_damping_enabled) {
        Matrix3xN ebardot;  // Element coordinate time derivatives in matrix form
        CalcCoordDerivMatrix(ebardot);

        // Calculate the time derivative of the Deformation Gradient at the current point
        ChMatrixNM_col<double, 3, 3> Fdot = ebardot * Sxi_D;

        // Calculate the time derivative of the Green-Lagrange strain tensor in Voigt notation
        // and combine it with epsilon assuming a Linear Kelvin-Voigt Viscoelastic material model
        epsilon_combined(0) += m_Alpha * F.col(0).dot(Fdot.col(0));
        epsilon_combined(1) += m_Alpha * F.col(1).dot(Fdot.col(1));
        epsilon_combined(2) += m_Alpha * F.col(2).dot(Fdot.col(2));
        epsilon_combined(3) += m_Alpha * (F.col(1).dot(Fdot.col(2)) + Fdot.col(1).dot(F.col(2)));
        epsilon_combined(4) += m_Alpha * (F.col(0).dot(Fdot.col(2)) + Fdot.col(0).dot(F.col(2)));
        epsilon_combined(5) += m_Alpha * (F.col(0).dot(Fdot.col(1)) + Fdot.col(0).dot(F.col(1)));
    }

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    ChVectorN<double, 6> sigmaPK2 = D * epsilon_combined;  // 2nd Piola Kirchhoff Stress tensor in Voigt notation

    ChMatrixNM<double, 3, 3> SPK2;  // 2nd Piola Kirchhoff Stress tensor
    SPK2(0, 0) = sigmaPK2(0);
    SPK2(1, 1) = sigmaPK2(1);
    SPK2(2, 2) = sigmaPK2(2);
    SPK2(1, 2) = sigmaPK2(3);
    SPK2(2, 1) = sigmaPK2(3);
    SPK2(0, 2) = sigmaPK2(4);
    SPK2(2, 0) = sigmaPK2(4);
    SPK2(0, 1) = sigmaPK2(5);
    SPK2(1, 0) = sigmaPK2(5);

    // Convert from 2ndPK Stress to Cauchy Stress
    ChMatrix33<double> S = (F * SPK2 * F.transpose()) / F.determinant();
    double SVonMises =
        sqrt(0.5 * ((S(0, 0) - S(1, 1)) * (S(0, 0) - S(1, 1)) + (S(1, 1) - S(2, 2)) * (S(1, 1) - S(2, 2)) +
                    (S(2, 2) - S(0, 0)) * (S(2, 2) - S(0, 0))) +
             3 * (S(1, 2) * S(1, 2) + S(2, 0) * S(2, 0) + S(0, 1) * S(0, 1)));

    return (SVonMises);
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.

void ChElementHexaANCF_3843::SetupInitial(ChSystem* system) {
    m_element_dof = 0;
    for (int i = 0; i < 8; i++) {
        m_element_dof += m_nodes[i]->GetNdofX();
    }

    m_full_dof = (m_element_dof == 8 * 12);

    if (!m_full_dof) {
        m_mapping_dof.resize(m_element_dof);
        int dof = 0;
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < m_nodes[i]->GetNdofX(); j++)
                m_mapping_dof(dof++) = i * 12 + j;
        }
    }

    // Store the initial nodal coordinates. These values define the reference configuration of the element.
    CalcCoordMatrix(m_ebar0);

    // Compute and store the constant mass matrix and the matrix used to multiply the acceleration due to gravity to get
    // the generalized gravitational force vector for the element
    ComputeMassMatrixAndGravityForce();

    // Compute any required matrices and vectors for the generalized internal force and Jacobian calculations
    PrecomputeInternalForceMatricesWeights();
}

// Fill the D vector with the current field values at the element nodes.

void ChElementHexaANCF_3843::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(6, 3) = m_nodes[0]->GetDD().eigen();
    mD.segment(9, 3) = m_nodes[0]->GetDDD().eigen();

    mD.segment(12, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(15, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(18, 3) = m_nodes[1]->GetDD().eigen();
    mD.segment(21, 3) = m_nodes[1]->GetDDD().eigen();

    mD.segment(24, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(27, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(30, 3) = m_nodes[2]->GetDD().eigen();
    mD.segment(33, 3) = m_nodes[2]->GetDDD().eigen();

    mD.segment(36, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(39, 3) = m_nodes[3]->GetD().eigen();
    mD.segment(42, 3) = m_nodes[3]->GetDD().eigen();
    mD.segment(45, 3) = m_nodes[3]->GetDDD().eigen();

    mD.segment(48, 3) = m_nodes[4]->GetPos().eigen();
    mD.segment(51, 3) = m_nodes[4]->GetD().eigen();
    mD.segment(54, 3) = m_nodes[4]->GetDD().eigen();
    mD.segment(57, 3) = m_nodes[4]->GetDDD().eigen();

    mD.segment(60, 3) = m_nodes[5]->GetPos().eigen();
    mD.segment(63, 3) = m_nodes[5]->GetD().eigen();
    mD.segment(66, 3) = m_nodes[5]->GetDD().eigen();
    mD.segment(69, 3) = m_nodes[5]->GetDDD().eigen();

    mD.segment(72, 3) = m_nodes[6]->GetPos().eigen();
    mD.segment(75, 3) = m_nodes[6]->GetD().eigen();
    mD.segment(78, 3) = m_nodes[6]->GetDD().eigen();
    mD.segment(81, 3) = m_nodes[6]->GetDDD().eigen();

    mD.segment(84, 3) = m_nodes[7]->GetPos().eigen();
    mD.segment(87, 3) = m_nodes[7]->GetD().eigen();
    mD.segment(90, 3) = m_nodes[7]->GetDD().eigen();
    mD.segment(93, 3) = m_nodes[7]->GetDDD().eigen();
}

// State update.

void ChElementHexaANCF_3843::Update() {
    ChElementGeneric::Update();
}

// Return the mass matrix in full sparse form.

void ChElementHexaANCF_3843::ComputeMmatrixGlobal(ChMatrixRef M) {
    M.setZero();

    // Mass Matrix is Stored in Compact Upper Triangular Form
    // Expand it out into its Full Sparse Symmetric Form
    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            M(3 * i, 3 * j) = m_MassMatrix(idx);
            M(3 * i + 1, 3 * j + 1) = m_MassMatrix(idx);
            M(3 * i + 2, 3 * j + 2) = m_MassMatrix(idx);
            if (i != j) {
                M(3 * j, 3 * i) = m_MassMatrix(idx);
                M(3 * j + 1, 3 * i + 1) = m_MassMatrix(idx);
                M(3 * j + 2, 3 * i + 2) = m_MassMatrix(idx);
            }
            idx++;
        }
    }
}

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass

void ChElementHexaANCF_3843::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0) + m_MassMatrix(4) + m_MassMatrix(8) + m_MassMatrix(12) +
                               m_MassMatrix(16) + m_MassMatrix(20) + m_MassMatrix(24) + m_MassMatrix(28);
    m_nodes[1]->m_TotalMass += m_MassMatrix(4) + m_MassMatrix(122) + m_MassMatrix(126) + m_MassMatrix(130) +
                               m_MassMatrix(134) + m_MassMatrix(138) + m_MassMatrix(142) + m_MassMatrix(146);
    m_nodes[2]->m_TotalMass += m_MassMatrix(8) + m_MassMatrix(126) + m_MassMatrix(228) + m_MassMatrix(232) +
                               m_MassMatrix(236) + m_MassMatrix(240) + m_MassMatrix(244) + m_MassMatrix(248);
    m_nodes[3]->m_TotalMass += m_MassMatrix(12) + m_MassMatrix(130) + m_MassMatrix(232) + m_MassMatrix(318) +
                               m_MassMatrix(322) + m_MassMatrix(326) + m_MassMatrix(330) + m_MassMatrix(334);
    m_nodes[4]->m_TotalMass += m_MassMatrix(16) + m_MassMatrix(134) + m_MassMatrix(236) + m_MassMatrix(322) +
                               m_MassMatrix(392) + m_MassMatrix(396) + m_MassMatrix(400) + m_MassMatrix(404);
    m_nodes[5]->m_TotalMass += m_MassMatrix(20) + m_MassMatrix(138) + m_MassMatrix(240) + m_MassMatrix(326) +
                               m_MassMatrix(396) + m_MassMatrix(450) + m_MassMatrix(454) + m_MassMatrix(458);
    m_nodes[6]->m_TotalMass += m_MassMatrix(24) + m_MassMatrix(142) + m_MassMatrix(244) + m_MassMatrix(330) +
                               m_MassMatrix(400) + m_MassMatrix(454) + m_MassMatrix(492) + m_MassMatrix(496);
    m_nodes[7]->m_TotalMass += m_MassMatrix(28) + m_MassMatrix(146) + m_MassMatrix(248) + m_MassMatrix(334) +
                               m_MassMatrix(404) + m_MassMatrix(458) + m_MassMatrix(496) + m_MassMatrix(518);
}

// Compute the generalized internal force vector for the current nodal coordinates and set the value in the Fi vector.

void ChElementHexaANCF_3843::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == GetNdofs());

    if (m_method == IntFrcMethod::ContInt) {
        if (m_damping_enabled) {  // If linear Kelvin-Voigt viscoelastic material model is enabled
            ComputeInternalForcesContIntDamping(Fi);
        } else {
            ComputeInternalForcesContIntNoDamping(Fi);
        }
    } else {
        ComputeInternalForcesContIntPreInt(Fi);
    }
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]

void ChElementHexaANCF_3843::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == GetNdofs()) && (H.cols() == GetNdofs()));

    if (m_method == IntFrcMethod::ContInt) {
        if (m_damping_enabled) {  // If linear Kelvin-Voigt viscoelastic material model is enabled
            ComputeInternalJacobianContIntDamping(H, -Kfactor, -Rfactor, Mfactor);
        } else {
            ComputeInternalJacobianContIntNoDamping(H, -Kfactor, Mfactor);
        }
    } else {
        ComputeInternalJacobianPreInt(H, Kfactor, Rfactor, Mfactor);
    }
}

// Compute the generalized force vector due to gravity using the efficient ANCF specific method
void ChElementHexaANCF_3843::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    assert(Fg.size() == GetNdofs());

    // Calculate and add the generalized force due to gravity to the generalized internal force vector for the element.
    // The generalized force due to gravity could be computed once prior to the start of the simulation if gravity was
    // assumed constant throughout the entire simulation.  However, this implementation assumes that the acceleration
    // due to gravity, while a constant for the entire system, can change from step to step which could be useful for
    // gravity loaded units tests as an example.  The generalized force due to gravity is calculated in compact matrix
    // form and is pre-mapped to the desired vector format
    Eigen::Map<MatrixNx3> GravForceCompact(Fg.data(), NSF, 3);
    GravForceCompact = m_GravForceScale * G_acc.eigen().transpose();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void ChElementHexaANCF_3843::EvaluateElementFrame(const double xi,
                                                  const double eta,
                                                  const double zeta,
                                                  ChVector<>& point,
                                                  ChQuaternion<>& rot) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta);
    VectorN Sxi_xi_compact;
    Calc_Sxi_xi_compact(Sxi_xi_compact, xi, eta, zeta);
    VectorN Sxi_eta_compact;
    Calc_Sxi_eta_compact(Sxi_eta_compact, xi, eta, zeta);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;

    // Since ANCF does not use rotations, calculate an approximate
    // rotation based off the position vector gradients
    ChVector<double> Xdir = e_bar * Sxi_xi_compact * 2 / m_lenX;
    ChVector<double> Ydir = e_bar * Sxi_eta_compact * 2 / m_lenY;

    // Since the position vector gradients are not in general orthogonal,
    // set the Dx direction tangent to the brick xi axis and
    // compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization, guided by the brick eta axis
    ChMatrix33<> msect;
    msect.Set_A_Xdir(Xdir, Ydir);

    rot = msect.Get_A_quaternion();
}

void ChElementHexaANCF_3843::EvaluateElementPoint(const double xi,
                                                  const double eta,
                                                  const double zeta,
                                                  ChVector<>& point) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;
}

void ChElementHexaANCF_3843::EvaluateElementVel(double xi, double eta, const double zeta, ChVector<>& Result) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta);

    Matrix3xN e_bardot;
    CalcCoordDerivMatrix(e_bardot);

    // rdot = S*edot written in compact form
    Result = e_bardot * Sxi_compact;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).

void ChElementHexaANCF_3843::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[0]->GetDDD().eigen();

    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[1]->GetDD().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[1]->GetDDD().eigen();

    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[2]->GetDD().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[2]->GetDDD().eigen();

    mD.segment(block_offset + 36, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[3]->GetD().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[3]->GetDD().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[3]->GetDDD().eigen();

    mD.segment(block_offset + 48, 3) = m_nodes[4]->GetPos().eigen();
    mD.segment(block_offset + 51, 3) = m_nodes[4]->GetD().eigen();
    mD.segment(block_offset + 54, 3) = m_nodes[4]->GetDD().eigen();
    mD.segment(block_offset + 57, 3) = m_nodes[4]->GetDDD().eigen();

    mD.segment(block_offset + 60, 3) = m_nodes[5]->GetPos().eigen();
    mD.segment(block_offset + 63, 3) = m_nodes[5]->GetD().eigen();
    mD.segment(block_offset + 66, 3) = m_nodes[5]->GetDD().eigen();
    mD.segment(block_offset + 69, 3) = m_nodes[5]->GetDDD().eigen();

    mD.segment(block_offset + 72, 3) = m_nodes[6]->GetPos().eigen();
    mD.segment(block_offset + 75, 3) = m_nodes[6]->GetD().eigen();
    mD.segment(block_offset + 78, 3) = m_nodes[6]->GetDD().eigen();
    mD.segment(block_offset + 81, 3) = m_nodes[6]->GetDDD().eigen();

    mD.segment(block_offset + 84, 3) = m_nodes[7]->GetPos().eigen();
    mD.segment(block_offset + 87, 3) = m_nodes[7]->GetD().eigen();
    mD.segment(block_offset + 90, 3) = m_nodes[7]->GetDD().eigen();
    mD.segment(block_offset + 93, 3) = m_nodes[7]->GetDDD().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).

void ChElementHexaANCF_3843::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD_dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[0]->GetDDD_dt().eigen();

    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetD_dt().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[1]->GetDD_dt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[1]->GetDDD_dt().eigen();

    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[2]->GetD_dt().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[2]->GetDD_dt().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[2]->GetDDD_dt().eigen();

    mD.segment(block_offset + 36, 3) = m_nodes[3]->GetPos_dt().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[3]->GetD_dt().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[3]->GetDD_dt().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[3]->GetDDD_dt().eigen();

    mD.segment(block_offset + 48, 3) = m_nodes[4]->GetPos_dt().eigen();
    mD.segment(block_offset + 51, 3) = m_nodes[4]->GetD_dt().eigen();
    mD.segment(block_offset + 54, 3) = m_nodes[4]->GetDD_dt().eigen();
    mD.segment(block_offset + 57, 3) = m_nodes[4]->GetDDD_dt().eigen();

    mD.segment(block_offset + 60, 3) = m_nodes[5]->GetPos_dt().eigen();
    mD.segment(block_offset + 63, 3) = m_nodes[5]->GetD_dt().eigen();
    mD.segment(block_offset + 66, 3) = m_nodes[5]->GetDD_dt().eigen();
    mD.segment(block_offset + 69, 3) = m_nodes[5]->GetDDD_dt().eigen();

    mD.segment(block_offset + 72, 3) = m_nodes[6]->GetPos_dt().eigen();
    mD.segment(block_offset + 75, 3) = m_nodes[6]->GetD_dt().eigen();
    mD.segment(block_offset + 78, 3) = m_nodes[6]->GetDD_dt().eigen();
    mD.segment(block_offset + 81, 3) = m_nodes[6]->GetDDD_dt().eigen();

    mD.segment(block_offset + 84, 3) = m_nodes[7]->GetPos_dt().eigen();
    mD.segment(block_offset + 87, 3) = m_nodes[7]->GetD_dt().eigen();
    mD.segment(block_offset + 90, 3) = m_nodes[7]->GetDD_dt().eigen();
    mD.segment(block_offset + 93, 3) = m_nodes[7]->GetDDD_dt().eigen();
}

/// Increment all DOFs using a delta.

void ChElementHexaANCF_3843::LoadableStateIncrement(const unsigned int off_x,
                                                    ChState& x_new,
                                                    const ChState& x,
                                                    const unsigned int off_v,
                                                    const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 12, x_new, x, off_v + 12, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 24, x_new, x, off_v + 24, Dv);
    m_nodes[3]->NodeIntStateIncrement(off_x + 36, x_new, x, off_v + 36, Dv);
    m_nodes[4]->NodeIntStateIncrement(off_x + 48, x_new, x, off_v + 48, Dv);
    m_nodes[5]->NodeIntStateIncrement(off_x + 60, x_new, x, off_v + 60, Dv);
    m_nodes[6]->NodeIntStateIncrement(off_x + 72, x_new, x, off_v + 72, Dv);
    m_nodes[7]->NodeIntStateIncrement(off_x + 84, x_new, x, off_v + 84, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.

void ChElementHexaANCF_3843::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
        mvars.push_back(&m_nodes[i]->Variables_DD());
        mvars.push_back(&m_nodes[i]->Variables_DDD());
    }
}

// Evaluate N'*F, which is the projection of the applied point force and moment at the coordinates (xi,eta,zeta)
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.

void ChElementHexaANCF_3843::ComputeNF(
    const double xi,             // parametric coordinate in volume
    const double eta,            // parametric coordinate in volume
    const double zeta,           // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    // Compute the generalized force vector for the applied force component using the compact form of the shape
    // functions.  This requires a reshaping of the calculated matrix to get it into the correct vector order (just a
    // reinterpretation of the data since the matrix is in row-major format)
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, eta, zeta);
    MatrixNx3 QiCompact;

    QiCompact = Sxi_compact * F.segment(0, 3).transpose();

    Eigen::Map<Vector3N> QiReshaped(QiCompact.data(), QiCompact.size());
    Qi = QiReshaped;

    // Compute the generalized force vector for the applied moment component
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    ChMatrix33<double> J_Cxi;
    ChMatrix33<double> J_Cxi_Inv;

    J_Cxi.noalias() = e_bar * Sxi_D;
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    VectorN G_A = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 0) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 0) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 0);
    VectorN G_B = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 1) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 1) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 1);
    VectorN G_C = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 2) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 2) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 2);

    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    for (unsigned int i = 0; i < NSF; i++) {
        Qi(3 * i) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi((3 * i) + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi((3 * i) + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }

    // Compute the element Jacobian between the current configuration and the normalized configuration
    // This is different than the element Jacobian between the reference configuration and the normalized
    //  configuration used in the internal force calculations.  For this calculation, this is the ratio between the
    //  actual differential volume and the normalized differential volume.  The determinate of the element Jacobian is
    //  used to calculate this volume ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.determinant();
}

// Return the element density (needed for ChLoaderVolumeGravity).

double ChElementHexaANCF_3843::GetDensity() {
    return GetMaterial()->Get_rho();
}

// -----------------------------------------------------------------------------
// Mass Matrix & Generalized Force Due to Gravity Calculation
// -----------------------------------------------------------------------------

void ChElementHexaANCF_3843::ComputeMassMatrixAndGravityForce() {
    // For this element, the mass matrix integrand is of order 7 in xi, 7 in eta, and 7 in zeta.
    // 4 GQ Points are needed in the xi, eta, and zeta directions for exact integration of the element's mass matrix,
    // even if the reference configuration is not straight. Since the major pieces of the generalized force due to
    // gravity can also be used to calculate the mass matrix, these calculations are performed at the same time.  Only
    // the matrix that scales the acceleration due to gravity is calculated at this time so that any changes to the
    // acceleration due to gravity in the system are correctly accounted for in the generalized internal force
    // calculation.

    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi_eta_zeta = 3;  // 4 Point Gauss-Quadrature;

    // Mass Matrix in its compact matrix form.  Since the mass matrix is symmetric, just the upper diagonal entries will
    // be stored.
    ChMatrixNM<double, NSF, NSF> MassMatrixCompactSquare;

    // Set these to zeros since they will be incremented as the vector/matrix is calculated
    MassMatrixCompactSquare.setZero();
    m_GravForceScale.setZero();

    double rho = GetMaterial()->Get_rho();  // Density of the material for the element

    // Sum the contribution to the mass matrix and generalized force due to gravity at the current point
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi_eta_zeta][it_xi] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_xi];
                double eta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_zeta];
                double det_J_0xi = Calc_det_J_0xi(xi, eta, zeta);  // determinant of the element Jacobian (volume ratio)

                VectorN Sxi_compact;  // Vector of the Unique Normalized Shape Functions
                Calc_Sxi_compact(Sxi_compact, xi, eta, zeta);

                m_GravForceScale += (GQ_weight * rho * det_J_0xi) * Sxi_compact;
                MassMatrixCompactSquare += (GQ_weight * rho * det_J_0xi) * Sxi_compact * Sxi_compact.transpose();
            }
        }
    }

    // Store just the unique entries in the Mass Matrix in Compact Upper Triangular Form
    // since the full Mass Matrix is both sparse and symmetric
    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            m_MassMatrix(idx) = MassMatrixCompactSquare(i, j);
            idx++;
        }
    }
}

// Precalculate constant matrices and scalars for the internal force calculations

void ChElementHexaANCF_3843::PrecomputeInternalForceMatricesWeights() {
    if (m_method == IntFrcMethod::ContInt)
        PrecomputeInternalForceMatricesWeightsContInt();
    else
        PrecomputeInternalForceMatricesWeightsPreInt();
}

// Precalculate constant matrices for the internal force calculations when using the "Continuous Integration" style
// method

void ChElementHexaANCF_3843::PrecomputeInternalForceMatricesWeightsContInt() {
    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi_eta_zeta = NP - 1;  // Gauss-Quadrature table index for xi, eta, and zeta

    m_SD.resize(NSF, 3 * NIP);
    m_kGQ.resize(NIP, 1);

    ChMatrixNM<double, NSF, 3> SD_precompute_D;

    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi_eta_zeta][it_xi] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_xi];
                double eta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_zeta];
                auto index =
                    it_zeta + it_eta * GQTable->Lroots[GQ_idx_xi_eta_zeta].size() +
                    it_xi * GQTable->Lroots[GQ_idx_xi_eta_zeta].size() * GQTable->Lroots[GQ_idx_xi_eta_zeta].size();
                ChMatrix33<double>
                    J_0xi;         // Element Jacobian between the reference configuration and normalized configuration
                MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives

                Calc_Sxi_D(Sxi_D, xi, eta, zeta);
                J_0xi.noalias() = m_ebar0 * Sxi_D;

                // Adjust the shape function derivative matrix to account for a potentially deformed reference state
                SD_precompute_D = Sxi_D * J_0xi.inverse();
                m_kGQ(index) = -J_0xi.determinant() * GQ_weight;

                // Group all of the columns together in blocks across the entire element
                m_SD.col(index) = SD_precompute_D.col(0);
                m_SD.col(index + NIP) = SD_precompute_D.col(1);
                m_SD.col(index + 2 * NIP) = SD_precompute_D.col(2);

                index++;
            }
        }
    }
}

// Precalculate constant matrices for the internal force calculations when using the "Pre-Integration" style
// method

void ChElementHexaANCF_3843::PrecomputeInternalForceMatricesWeightsPreInt() {
    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi_eta_zeta = NP - 1;  // Gauss-Quadrature table index for xi, eta, and zeta

    m_O1.resize(NSF * NSF, NSF * NSF);
    m_O2.resize(NSF * NSF, NSF * NSF);
    m_K3Compact.resize(NSF, NSF);
    m_K13Compact.resize(NSF, NSF);

    m_K13Compact.setZero();
    m_K3Compact.setZero();
    m_O1.setZero();

    // =============================================================================
    // Get the stiffness tensor in 6x6 matrix form
    // =============================================================================

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    // =============================================================================
    // Pull the required entries from the 4th order stiffness tensor that are needed for K3.  Note that D is written
    // in Voigt notation rather than a 4th other tensor so the correct entries from that matrix need to be pulled.
    // D11 = slice (matrix) of the 4th order tensor with last two subscripts = 1)
    // D22 = slice (matrix) of the 4th order tensor with last two subscripts = 2)
    // D33 = slice (matrix) of the 4th order tensor with last two subscripts = 3)
    // =============================================================================
    ChMatrixNM<double, 3, 3> D11;
    ChMatrixNM<double, 3, 3> D22;
    ChMatrixNM<double, 3, 3> D33;
    D11(0, 0) = D(0, 0);
    D11(1, 1) = D(1, 0);
    D11(2, 2) = D(2, 0);
    D11(1, 0) = D(5, 0);
    D11(0, 1) = D(5, 0);
    D11(2, 0) = D(4, 0);
    D11(0, 2) = D(4, 0);
    D11(2, 1) = D(3, 0);
    D11(1, 2) = D(3, 0);

    D22(0, 0) = D(0, 1);
    D22(1, 1) = D(1, 1);
    D22(2, 2) = D(2, 1);
    D22(1, 0) = D(5, 1);
    D22(0, 1) = D(5, 1);
    D22(2, 0) = D(4, 1);
    D22(0, 2) = D(4, 1);
    D22(2, 1) = D(3, 1);
    D22(1, 2) = D(3, 1);

    D33(0, 0) = D(0, 2);
    D33(1, 1) = D(1, 2);
    D33(2, 2) = D(2, 2);
    D33(1, 0) = D(5, 2);
    D33(0, 1) = D(5, 2);
    D33(2, 0) = D(4, 2);
    D33(0, 2) = D(4, 2);
    D33(2, 1) = D(3, 2);
    D33(1, 2) = D(3, 2);

    // Setup the stiffness tensor in block matrix form to make it easier to iterate through all 4 subscripts
    ChMatrixNM<double, 9, 9> D_block;
    D_block << D(0, 0), D(0, 5), D(0, 4), D(0, 5), D(0, 1), D(0, 3), D(0, 4), D(0, 3), D(0, 2), D(5, 0), D(5, 5),
        D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3), D(5, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(5, 0), D(5, 5), D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3),
        D(5, 2), D(1, 0), D(1, 5), D(1, 4), D(1, 5), D(1, 1), D(1, 3), D(1, 4), D(1, 3), D(1, 2), D(3, 0), D(3, 5),
        D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3), D(3, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(3, 0), D(3, 5), D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3),
        D(3, 2), D(2, 0), D(2, 5), D(2, 4), D(2, 5), D(2, 1), D(2, 3), D(2, 4), D(2, 3), D(2, 2);

    // Loop over each Gauss quadrature point and sum the contribution to each constant matrix that will be used for
    // the internal force calculations
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_xi_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi_eta_zeta][it_xi] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_xi_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_xi];
                double eta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_xi_eta_zeta][it_zeta];

                ChMatrix33<double>
                    J_0xi;         // Element Jacobian between the reference configuration and normalized configuration
                MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives

                Calc_Sxi_D(Sxi_D, xi, eta, zeta);
                J_0xi.noalias() = m_ebar0 * Sxi_D;

                MatrixNx3c Sxi_D_0xi = Sxi_D * J_0xi.inverse();
                double GQWeight_det_J_0xi = -J_0xi.determinant() * GQ_weight;

                m_K3Compact += GQWeight_det_J_0xi * 0.5 *
                               (Sxi_D_0xi * D11 * Sxi_D_0xi.transpose() + Sxi_D_0xi * D22 * Sxi_D_0xi.transpose() +
                                Sxi_D_0xi * D33 * Sxi_D_0xi.transpose());

                MatrixNxN scale;
                for (unsigned int n = 0; n < 3; n++) {
                    for (unsigned int c = 0; c < 3; c++) {
                        scale = Sxi_D_0xi * D_block.block<3, 3>(3 * n, 3 * c) * Sxi_D_0xi.transpose();
                        scale *= GQWeight_det_J_0xi;

                        MatrixNxN Sxi_D_0xi_n_Sxi_D_0xi_c_transpose =
                            Sxi_D_0xi.template block<NSF, 1>(0, n) * Sxi_D_0xi.template block<NSF, 1>(0, c).transpose();
                        for (unsigned int f = 0; f < NSF; f++) {
                            for (unsigned int t = 0; t < NSF; t++) {
                                m_O1.block<NSF, NSF>(NSF * t, NSF * f) +=
                                    scale(t, f) * Sxi_D_0xi_n_Sxi_D_0xi_c_transpose;
                            }
                        }
                    }
                }
            }
        }
    }

    // Since O2 is just a reordered version of O1, wait until O1 is completely calculated and then generate O2 from it
    for (unsigned int f = 0; f < NSF; f++) {
        for (unsigned int t = 0; t < NSF; t++) {
            m_O2.block<NSF, NSF>(NSF * t, NSF * f) = m_O1.block<NSF, NSF>(NSF * t, NSF * f).transpose();
        }
    }
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

void ChElementHexaANCF_3843::ComputeInternalForcesContIntDamping(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Continuous Integration" style of method assuming a
    // linear viscoelastic material model (single term damping model).  For this style of method, the generalized
    // internal force vector is integrated across the volume of the element every time this calculation is performed.
    // For this element, this is likely more efficient than the "Pre-Integration" style calculation method.  Note that
    // the integrand for the generalize internal force vector for a straight and normalized element is of order : 12 in
    // xi, 12 in eta, and 12 in zeta. This requires GQ 7 points along the xi, eta, and zeta directions for "Full
    // Integration". However, very similar results can be obtained with fewer GQ point in each direction, resulting in
    // significantly fewer calculations.  Based on testing, this could be as low as 4x4x4

    MatrixNx6 ebar_ebardot;
    CalcCombinedCoordMatrix(ebar_ebardot);

    // =============================================================================
    // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
    // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
    // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31  F11dot  F21dot  F31dot ]
    // FC = [F12  F22  F32  F12dot  F22dot  F32dot ]
    //      [F13  F23  F33  F13dot  F23dot  F33dot ]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 6> FC = m_SD.transpose() * ebar_ebardot;

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // Note that the scaled time derivatives of the Green-Lagrange strain are added to make the later calculation of
    // the 2nd Piola-Kirchoff stresses more efficient.  The combined result is then scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point (m_kGQ) again for efficiency.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*(E11+alpha*E11dot)
    //                  = kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP E_BlockDamping = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5));
    VectorNIP E1_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 2));
    E1_Block.array() -= 1;
    E1_Block *= 0.5;
    E1_Block += m_Alpha * E_BlockDamping;
    E1_Block.array() *= m_kGQ.array();

    // Each entry in E2 = kGQ*(E22+alpha*E22dot)
    //                  = kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5));
    VectorNIP E2_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E2_Block.array() -= 1;
    E2_Block *= 0.5;
    E2_Block += m_Alpha * E_BlockDamping;
    E2_Block.array() *= m_kGQ.array();

    // Each entry in E3 = kGQ*(E33+alpha*E33dot)
    //                  = kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping.noalias() =
        FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
        FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
        FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E3_Block = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E3_Block.array() -= 1;
    E3_Block *= 0.5;
    E3_Block += m_Alpha * E_BlockDamping;
    E3_Block.array() *= m_kGQ.array();

    // Each entry in E4 = kGQ*(2*(E23+alpha*E23dot))
    //                  = kGQ*((F12*F13+F22*F23+F32*F33)
    //                    +alpha*(F12dot*F13+F22dot*F23+F32dot*F33 + F12*F13dot+F22*F23dot+F32*F33dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5)) +
                               FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E4_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E4_Block += m_Alpha * E_BlockDamping;
    E4_Block.array() *= m_kGQ.array();

    // Each entry in E5 = kGQ*(2*(E13+alpha*E13dot))
    //                  = kGQ*((F11*F13+F21*F23+F31*F33)
    //                    +alpha*(F11dot*F13+F21dot*F23+F31dot*F33 + F11*F13dot+F21*F23dot+F31*F33dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5)) +
                               FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E5_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E5_Block += m_Alpha * E_BlockDamping;
    E5_Block.array() *= m_kGQ.array();

    // Each entry in E6 = kGQ*(2*(E12+alpha*E12dot))
    //                  = kGQ*((F11*F12+F21*F22+F31*F32)
    //                    +alpha*(F11dot*F12+F21dot*F22+F31dot*F32 + F11*F12dot+F21*F22dot+F31*F32dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5)) +
                               FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5));
    VectorNIP E6_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E6_Block += m_Alpha * E_BlockDamping;
    E6_Block.array() *= m_kGQ.array();

    // =============================================================================
    // Get the stiffness tensor in 6x6 matrix form
    // =============================================================================

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have been scaled have already been
    // combined with their scaled time derivatives and minus the Gauss quadrature weight times the element Jacobian at
    // the corresponding Gauss point to make the calculation of the 2nd Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP SPK2_1_Block = D(0, 0) * E1_Block + D(0, 1) * E2_Block + D(0, 2) * E3_Block + D(0, 3) * E4_Block +
                             D(0, 4) * E5_Block + D(0, 5) * E6_Block;
    VectorNIP SPK2_2_Block = D(1, 0) * E1_Block + D(1, 1) * E2_Block + D(1, 2) * E3_Block + D(1, 3) * E4_Block +
                             D(1, 4) * E5_Block + D(1, 5) * E6_Block;
    VectorNIP SPK2_3_Block = D(2, 0) * E1_Block + D(2, 1) * E2_Block + D(2, 2) * E3_Block + D(2, 3) * E4_Block +
                             D(2, 4) * E5_Block + D(2, 5) * E6_Block;
    VectorNIP SPK2_4_Block = D(3, 0) * E1_Block + D(3, 1) * E2_Block + D(3, 2) * E3_Block + D(3, 3) * E4_Block +
                             D(3, 4) * E5_Block + D(3, 5) * E6_Block;
    VectorNIP SPK2_5_Block = D(4, 0) * E1_Block + D(4, 1) * E2_Block + D(4, 2) * E3_Block + D(4, 3) * E4_Block +
                             D(4, 4) * E5_Block + D(4, 5) * E6_Block;
    VectorNIP SPK2_6_Block = D(5, 0) * E1_Block + D(5, 1) * E2_Block + D(5, 2) * E3_Block + D(5, 3) * E4_Block +
                             D(5, 4) * E5_Block + D(5, 5) * E6_Block;

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ]
    //         = [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ]
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> P_Block;

    P_Block.template block<NIP, 1>(0, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_5_Block);
    P_Block.template block<NIP, 1>(0, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_5_Block);
    P_Block.template block<NIP, 1>(0, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_5_Block);

    P_Block.template block<NIP, 1>(NIP, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_4_Block);
    P_Block.template block<NIP, 1>(NIP, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_4_Block);
    P_Block.template block<NIP, 1>(NIP, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_4_Block);

    P_Block.template block<NIP, 1>(2 * NIP, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_3_Block);
    P_Block.template block<NIP, 1>(2 * NIP, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_3_Block);
    P_Block.template block<NIP, 1>(2 * NIP, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_3_Block);

    // =============================================================================
    // Multiply the scaled first Piola-Kirchoff stresses by the shape function derivative matrix to get the generalized
    // force vector in matrix form (in the correct order if its calculated in row-major memory layout)
    // =============================================================================

    MatrixNx3 QiCompact = m_SD * P_Block;

    // =============================================================================
    // Reshape the compact matrix form of the generalized internal force vector (stored using a row-major memory layout)
    // into its actual column vector format.  This is done by mathematically stacking the transpose of each row on top
    // of each other forming the column vector.  Due to the memory organization this is simply a reinterpretation of the
    // data
    // =============================================================================

    Eigen::Map<Vector3N> QiReshaped(QiCompact.data(), QiCompact.size());
    Fi = QiReshaped;
}

void ChElementHexaANCF_3843::ComputeInternalForcesContIntNoDamping(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Continuous Integration" style of method assuming a
    // linear material model (no damping).  For this style of method, the generalized internal force vector is
    // integrated across the volume of the element every time this calculation is performed.  For this element, this is
    // likely more efficient than the "Pre-Integration" style calculation method.  Note that the integrand for the
    // generalize internal force vector for a straight and normalized element is of order : 12 in xi, 12 in eta, and 12
    // in zeta. This requires GQ 7 points along the xi, eta, and zeta directions for "Full Integration". However, very
    // similar results can be obtained with fewer GQ point in each direction, resulting in significantly fewer
    // calculations.  Based on testing, this could be as low as 4x4x4

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // =============================================================================
    // Calculate the deformation gradient for all Gauss quadrature points in a single matrix multiplication.  Note
    // that since the shape function derivative matrix is ordered by columns, the resulting deformation gradient
    // will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31 ]
    // FC = [F12  F22  F32 ]
    //      [F13  F23  F33 ]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> FC = m_SD.transpose() * e_bar.transpose();

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // The result is then scaled by minus the Gauss quadrature weight times the element Jacobian at the
    // corresponding Gauss point (m_kGQ) for efficiency.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*E11 = kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP E1_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 2));
    E1_Block.array() -= 1;
    E1_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E2 = kGQ*E22 = kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP E2_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E2_Block.array() -= 1;
    E2_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E3 = kGQ*E33 = kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP E3_Block = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E3_Block.array() -= 1;
    E3_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E4 = kGQ*2*E23 = kGQ*(F12*F13+F22*F23+F32*F33)
    VectorNIP E4_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E4_Block.array() *= m_kGQ.array();

    // Each entry in E5 = kGQ*2*E13 = kGQ*(F11*F13+F21*F23+F31*F33)
    VectorNIP E5_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E5_Block.array() *= m_kGQ.array();

    // Each entry in E6 = kGQ*2*E12 = (F11*F12+F21*F22+F31*F32)
    VectorNIP E6_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E6_Block.array() *= m_kGQ.array();

    // =============================================================================
    // Get the stiffness tensor in 6x6 matrix form
    // =============================================================================

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have been scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point to make the calculation of the 2nd
    // Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP SPK2_1_Block = D(0, 0) * E1_Block + D(0, 1) * E2_Block + D(0, 2) * E3_Block + D(0, 3) * E4_Block +
                             D(0, 4) * E5_Block + D(0, 5) * E6_Block;
    VectorNIP SPK2_2_Block = D(1, 0) * E1_Block + D(1, 1) * E2_Block + D(1, 2) * E3_Block + D(1, 3) * E4_Block +
                             D(1, 4) * E5_Block + D(1, 5) * E6_Block;
    VectorNIP SPK2_3_Block = D(2, 0) * E1_Block + D(2, 1) * E2_Block + D(2, 2) * E3_Block + D(2, 3) * E4_Block +
                             D(2, 4) * E5_Block + D(2, 5) * E6_Block;
    VectorNIP SPK2_4_Block = D(3, 0) * E1_Block + D(3, 1) * E2_Block + D(3, 2) * E3_Block + D(3, 3) * E4_Block +
                             D(3, 4) * E5_Block + D(3, 5) * E6_Block;
    VectorNIP SPK2_5_Block = D(4, 0) * E1_Block + D(4, 1) * E2_Block + D(4, 2) * E3_Block + D(4, 3) * E4_Block +
                             D(4, 4) * E5_Block + D(4, 5) * E6_Block;
    VectorNIP SPK2_6_Block = D(5, 0) * E1_Block + D(5, 1) * E2_Block + D(5, 2) * E3_Block + D(5, 3) * E4_Block +
                             D(5, 4) * E5_Block + D(5, 5) * E6_Block;

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ]
    //         = [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ]
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> P_Block;

    P_Block.template block<NIP, 1>(0, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_5_Block);
    P_Block.template block<NIP, 1>(0, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_5_Block);
    P_Block.template block<NIP, 1>(0, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_1_Block) +
                                           FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_6_Block) +
                                           FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_5_Block);

    P_Block.template block<NIP, 1>(NIP, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_4_Block);
    P_Block.template block<NIP, 1>(NIP, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_4_Block);
    P_Block.template block<NIP, 1>(NIP, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_6_Block) +
                                             FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_2_Block) +
                                             FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_4_Block);

    P_Block.template block<NIP, 1>(2 * NIP, 0) = FC.template block<NIP, 1>(0, 0).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 0).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(SPK2_3_Block);
    P_Block.template block<NIP, 1>(2 * NIP, 1) = FC.template block<NIP, 1>(0, 1).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 1).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(SPK2_3_Block);
    P_Block.template block<NIP, 1>(2 * NIP, 2) = FC.template block<NIP, 1>(0, 2).cwiseProduct(SPK2_5_Block) +
                                                 FC.template block<NIP, 1>(NIP, 2).cwiseProduct(SPK2_4_Block) +
                                                 FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(SPK2_3_Block);

    // =============================================================================
    // Multiply the scaled first Piola-Kirchoff stresses by the shape function derivative matrix to get the generalized
    // force vector in matrix form (in the correct order if its calculated in row-major memory layout)
    // =============================================================================

    MatrixNx3 QiCompact = m_SD * P_Block;

    // =============================================================================
    // Reshape the compact matrix form of the generalized internal force vector (stored using a row-major memory layout)
    // into its actual column vector format.  This is done by mathematically stacking the transpose of each row on top
    // of each other forming the column vector.  Due to the memory organization this is simply a reinterpretation of the
    // data
    // =============================================================================

    Eigen::Map<Vector3N> QiReshaped(QiCompact.data(), QiCompact.size());
    Fi = QiReshaped;
}

void ChElementHexaANCF_3843::ComputeInternalForcesContIntPreInt(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Pre-Integration" style of method assuming a
    // linear viscoelastic material model (single term damping model).  For this style of method, the components of the
    // generalized internal force vector and its Jacobian that need to be integrated across the volume are calculated
    // once prior to the start of the simulation.  For this method, the in-simulation calculations are independent of
    // the number of Gauss quadrature points used throughout the entire element.

    Matrix3xN ebar;
    Matrix3xN ebardot;

    CalcCoordMatrix(ebar);
    CalcCoordDerivMatrix(ebardot);

    // Calculate PI1 which is a combined form of the nodal coordinates.  It is calculated in matrix form and then later
    // reshaped into vector format (through a simple reinterpretation of the data)
    MatrixNxN PI1_matrix = 0.5 * ebar.transpose() * ebar;

    // If damping is enabled adjust PI1 to account for the extra terms.  This is the only modification required to
    // include damping in the generalized internal force calculation
    if (m_damping_enabled) {
        PI1_matrix += m_Alpha * ebardot.transpose() * ebar;
    }

    MatrixNxN K1_matrix;

    // Setup the reshaped/reinterpreted/mapped forms of PI1 and K1 to make the calculation of K1 simpler
    Eigen::Map<ChVectorN<double, NSF * NSF>> PI1(PI1_matrix.data(), PI1_matrix.size());
    Eigen::Map<ChVectorN<double, NSF * NSF>> K1_vec(K1_matrix.data(), K1_matrix.size());

    // Calculate the matrix K1 in mapped vector form and the resulting matrix will be in the correct form to combine
    // with K3
    K1_vec.noalias() = m_O1 * PI1;

    // Store the combined sum of K1 and K3 since it will be used again in the Jacobian calculation
    m_K13Compact.noalias() = K1_matrix - m_K3Compact;

    // Multiply the combined K1 and K3 matrix by the nodal coordinates in compact form and then remap it into the
    // required vector order that is the generalized internal force vector
    MatrixNx3 QiCompactLiu = m_K13Compact * ebar.transpose();
    Eigen::Map<Vector3N> QiReshapedLiu(QiCompactLiu.data(), QiCompactLiu.size());

    Fi = QiReshapedLiu;
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

void ChElementHexaANCF_3843::ComputeInternalJacobianContIntDamping(ChMatrixRef& H,
                                                                   double Kfactor,
                                                                   double Rfactor,
                                                                   double Mfactor) {
    // Calculate the Jacobian of the generalize internal force vector using the "Continuous Integration" style of method
    // assuming a linear viscoelastic material model (single term damping model).  For this style of method, the
    // Jacobian of the generalized internal force vector is integrated across the volume of the element every time this
    // calculation is performed. For this element, this is likely more efficient than the "Pre-Integration" style
    // calculation method.

    MatrixNx6 ebar_ebardot;
    CalcCombinedCoordMatrix(ebar_ebardot);

    // No values from the generalized internal force vector are cached for reuse in the Jacobian.  Instead these
    // quantities are recalculated again during the Jacobian calculations.  This both simplifies the code and speeds
    // up the generalized internal force calculation while only having a minimal impact on the performance of the
    // Jacobian calculation speed.  The Jacobian calculation is performed in two major pieces.  First is the
    // calculation of the potentially non-symmetric and non-sparse component.  The second pieces is the symmetric
    // and sparse component.

    // =============================================================================
    // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
    // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
    // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31  F11dot  F21dot  F31dot ]
    // FC = [F12  F22  F32  F12dot  F22dot  F32dot ]
    //      [F13  F23  F33  F13dot  F23dot  F33dot ]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 6> FC = m_SD.transpose() * ebar_ebardot;

    //==============================================================================
    //==============================================================================
    // Calculate the potentially non-symmetric and non-sparse component of the Jacobian matrix
    //==============================================================================
    //==============================================================================

    // =============================================================================
    // Calculate the partial derivative of the Green-Largrange strains with respect to the nodal coordinates
    // (transposed).  This calculation is performed in blocks across all the Gauss quadrature points at the same
    // time.  This value should be store in row major memory layout to align with the access patterns for
    // calculating this matrix.
    // PE = [(d epsilon1/d e)GQPnt1' (d epsilon1/d e)GQPnt2' ... (d epsilon1/d e)GQPntNIP'...
    //       (d epsilon2/d e)GQPnt1' (d epsilon2/d e)GQPnt2' ... (d epsilon2/d e)GQPntNIP'...
    //       (d epsilon3/d e)GQPnt1' (d epsilon3/d e)GQPnt2' ... (d epsilon3/d e)GQPntNIP'...
    //       (d epsilon4/d e)GQPnt1' (d epsilon4/d e)GQPnt2' ... (d epsilon4/d e)GQPntNIP'...
    //       (d epsilon5/d e)GQPnt1' (d epsilon5/d e)GQPnt2' ... (d epsilon5/d e)GQPntNIP'...
    //       (d epsilon6/d e)GQPnt1' (d epsilon6/d e)GQPnt2' ... (d epsilon6/d e)GQPntNIP']
    // Note that each partial derivative block shown is placed to the left of the previous block.
    // The explanation of the calculation above is just too long to write it all on a single line.
    // =============================================================================

    ChMatrixDynamic<> PE;
    PE.resize(3 * NSF, 6 * NIP);

    for (auto i = 0; i < NSF; i++) {
        PE.block<1, NIP>(3 * i, 0 * NIP) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 1 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(1 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(1 * NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0 * NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0 * NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(1 * NIP, 0).transpose());

        PE.block<1, NIP>((3 * i) + 1, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose());

        PE.block<1, NIP>((3 * i) + 2, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose());
    }

    // =============================================================================
    // Combine the deformation gradient, time derivative of the deformation gradient, damping coefficient, Gauss
    // quadrature weighting times the element Jacobian, and Jacobian component scale factors into a scaled block
    // deformation gradient matrix Calculate the deformation gradient and time derivative of the deformation
    // gradient for all Gauss quadrature points in a single matrix multiplication.  Note that the resulting combined
    // deformation gradient block matrix will be ordered by block matrix (column vectors) components Note that the
    // indices of the components are in transposed order
    //            [kGQ*(Kfactor+alpha*Rfactor)*F11+alpha*Rfactor*F11dot ... similar for F21 & F31 blocks]
    // FCscaled = [kGQ*(Kfactor+alpha*Rfactor)*F12+alpha*Rfactor*F12dot ... similar for F22 & F32 blocks]
    //            [kGQ*(Kfactor+alpha*Rfactor)*F13+alpha*Rfactor*F13dot ... similar for F23 & F33 blocks]
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> FCscaled = (Kfactor + m_Alpha * Rfactor) * FC.template block<3 * NIP, 3>(0, 0) +
                                               (m_Alpha * Kfactor) * FC.template block<3 * NIP, 3>(0, 3);

    for (auto i = 0; i < 3; i++) {
        FCscaled.template block<NIP, 1>(0, i).array() *= m_kGQ.array();
        FCscaled.template block<NIP, 1>(NIP, i).array() *= m_kGQ.array();
        FCscaled.template block<NIP, 1>(2 * NIP, i).array() *= m_kGQ.array();
    }

    // =============================================================================
    // Calculate the combination of the scaled partial derivative of the Green-Largrange strains with respect to the
    // nodal coordinates, the scaled partial derivative of the time derivative of the Green-Largrange strains with
    // respect to the nodal coordinates, the scaled partial derivative of the Green-Largrange strains with respect
    // to the time derivative of the nodal coordinates, and the other parameters to correctly integrate across the
    // volume of the element.  This calculation is performed in blocks across all the Gauss quadrature points at the
    // same time.  This value should be store in row major memory layout to align with the access patterns for
    // calculating this matrix.
    // =============================================================================

    ChMatrixDynamic<> Scaled_Combined_PE;
    Scaled_Combined_PE.resize(3 * NSF, 6 * NIP);

    for (auto i = 0; i < NSF; i++) {
        Scaled_Combined_PE.block<1, NIP>(3 * i, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose());

        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose());

        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose());
    }

    // =============================================================================
    // Get the stiffness tensor in 6x6 matrix form
    // =============================================================================

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    // =============================================================================
    // Multiply the scaled and combined partial derivative block matrix by the stiffness matrix for each individual
    // Gauss quadrature point
    // =============================================================================

    ChMatrixDynamic<> DScaled_Combined_PE;
    DScaled_Combined_PE.resize(3 * NSF, 6 * NIP);

    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 0) =
        D(0, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(0, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(0, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(0, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(0, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(0, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, NIP) =
        D(1, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(1, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(1, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(1, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(1, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(1, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 2 * NIP) =
        D(2, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(2, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(2, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(2, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(2, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(2, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 3 * NIP) =
        D(3, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(3, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(3, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(3, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(3, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(3, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 4 * NIP) =
        D(4, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(4, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(4, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(4, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(4, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(4, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 5 * NIP) =
        D(5, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(5, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(5, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(5, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(5, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(5, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);

    // =============================================================================
    // Multiply the partial derivative block matrix by the final scaled and combined partial derivative block matrix
    // to obtain the potentially non-symmetric and non-sparse component of the Jacobian matrix
    // =============================================================================

    H = PE * DScaled_Combined_PE.transpose();

    //==============================================================================
    //==============================================================================
    // Calculate the sparse and symmetric component of the Jacobian matrix
    //==============================================================================
    //==============================================================================

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // Note that the scaled time derivatives of the Green-Lagrange strain are added to make the later calculation of
    // the 2nd Piola-Kirchoff stresses more efficient.  The combined result is then scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point (m_kGQ) again for efficiency.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*(E11+alpha*E11dot)
    //                  = kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP E_BlockDamping = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5));
    VectorNIP E1_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 2));
    E1_Block.array() -= 1;
    E1_Block *= 0.5;
    E1_Block += m_Alpha * E_BlockDamping;
    E1_Block.array() *= m_kGQ.array();

    // Each entry in E2 = kGQ*(E22+alpha*E22dot)
    //                  = kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5));
    VectorNIP E2_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E2_Block.array() -= 1;
    E2_Block *= 0.5;
    E2_Block += m_Alpha * E_BlockDamping;
    E2_Block.array() *= m_kGQ.array();

    // Each entry in E3 = kGQ*(E33+alpha*E33dot)
    //                  = kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping.noalias() =
        FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
        FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
        FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E3_Block = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E3_Block.array() -= 1;
    E3_Block *= 0.5;
    E3_Block += m_Alpha * E_BlockDamping;
    E3_Block.array() *= m_kGQ.array();

    // Each entry in E4 = kGQ*(2*(E23+alpha*E23dot))
    //                  = kGQ*((F12*F13+F22*F23+F32*F33)
    //                    +alpha*(F12dot*F13+F22dot*F23+F32dot*F33 + F12*F13dot+F22*F23dot+F32*F33dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5)) +
                               FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E4_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E4_Block += m_Alpha * E_BlockDamping;
    E4_Block.array() *= m_kGQ.array();

    // Each entry in E5 = kGQ*(2*(E13+alpha*E13dot))
    //                  = kGQ*((F11*F13+F21*F23+F31*F33)
    //                    +alpha*(F11dot*F13+F21dot*F23+F31dot*F33 + F11*F13dot+F21*F23dot+F31*F33dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5)) +
                               FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 5));
    VectorNIP E5_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E5_Block += m_Alpha * E_BlockDamping;
    E5_Block.array() *= m_kGQ.array();

    // Each entry in E6 = kGQ*(2*(E12+alpha*E12dot))
    //                  = kGQ*((F11*F12+F21*F22+F31*F32)
    //                    +alpha*(F11dot*F12+F21dot*F22+F31dot*F32 + F11*F12dot+F21*F22dot+F31*F32dot))
    E_BlockDamping.noalias() = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(0, 3)) +
                               FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(0, 4)) +
                               FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(0, 5)) +
                               FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 3)) +
                               FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 4)) +
                               FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 5));
    VectorNIP E6_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E6_Block += m_Alpha * E_BlockDamping;
    E6_Block.array() *= m_kGQ.array();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have been scaled and already been
    // combined with their scaled time derivatives and minus the Gauss quadrature weight times the element Jacobian at
    // the corresponding Gauss point to make the calculation of the 2nd Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP SPK2_1_Block = D(0, 0) * E1_Block + D(0, 1) * E2_Block + D(0, 2) * E3_Block + D(0, 3) * E4_Block +
                             D(0, 4) * E5_Block + D(0, 5) * E6_Block;
    VectorNIP SPK2_2_Block = D(1, 0) * E1_Block + D(1, 1) * E2_Block + D(1, 2) * E3_Block + D(1, 3) * E4_Block +
                             D(1, 4) * E5_Block + D(1, 5) * E6_Block;
    VectorNIP SPK2_3_Block = D(2, 0) * E1_Block + D(2, 1) * E2_Block + D(2, 2) * E3_Block + D(2, 3) * E4_Block +
                             D(2, 4) * E5_Block + D(2, 5) * E6_Block;
    VectorNIP SPK2_4_Block = D(3, 0) * E1_Block + D(3, 1) * E2_Block + D(3, 2) * E3_Block + D(3, 3) * E4_Block +
                             D(3, 4) * E5_Block + D(3, 5) * E6_Block;
    VectorNIP SPK2_5_Block = D(4, 0) * E1_Block + D(4, 1) * E2_Block + D(4, 2) * E3_Block + D(4, 3) * E4_Block +
                             D(4, 4) * E5_Block + D(4, 5) * E6_Block;
    VectorNIP SPK2_6_Block = D(5, 0) * E1_Block + D(5, 1) * E2_Block + D(5, 2) * E3_Block + D(5, 3) * E4_Block +
                             D(5, 4) * E5_Block + D(5, 5) * E6_Block;

    // =============================================================================
    // Multiply the shape function derivative matrix by the 2nd Piola-Kirchoff stresses for each corresponding Gauss
    // quadrature point
    // =============================================================================

    ChMatrixNM<double, NSF, 3 * NIP> S_scaled_SD;

    for (auto i = 0; i < NSF; i++) {
        S_scaled_SD.template block<1, NIP>(i, 0) =
            SPK2_1_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_6_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_5_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));

        S_scaled_SD.template block<1, NIP>(i, NIP) =
            SPK2_6_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_2_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_4_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));

        S_scaled_SD.template block<1, NIP>(i, 2 * NIP) =
            SPK2_5_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_4_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_3_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));
    }

    // =============================================================================
    // Calculate just the non-sparse upper triangular entires of the sparse and symmetric component of the Jacobian
    // matrix, combine this with the scaled mass matrix, and then expand them out to full size by summing the
    // contribution into the correct locations of the full sized Jacobian matrix
    // =============================================================================

    // Add in the Mass Matrix Which is Stored in Compact Upper Triangular Form
    ChVectorN<double, (NSF * (NSF + 1)) / 2> ScaledMassMatrix = Mfactor * m_MassMatrix;

    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            double d = Kfactor * m_SD.row(i) * S_scaled_SD.row(j).transpose();
            d += ScaledMassMatrix(idx);

            H(3 * i, 3 * j) += d;
            H(3 * i + 1, 3 * j + 1) += d;
            H(3 * i + 2, 3 * j + 2) += d;
            if (i != j) {
                H(3 * j, 3 * i) += d;
                H(3 * j + 1, 3 * i + 1) += d;
                H(3 * j + 2, 3 * i + 2) += d;
            }
            idx++;
        }
    }
}

void ChElementHexaANCF_3843::ComputeInternalJacobianContIntNoDamping(ChMatrixRef& H, double Kfactor, double Mfactor) {
    // Calculate the Jacobian of the generalize internal force vector using the "Continuous Integration" style of method
    // assuming a linear material model (no damping).  For this style of method, the Jacobian of the generalized
    // internal force vector is integrated across the volume of the element every time this calculation is performed.
    // For this element, this is likely more efficient than the "Pre-Integration" style calculation method.

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // No values from the generalized internal force vector are cached for reuse in the Jacobian.  Instead these
    // quantities are recalculated again during the Jacobian calculations.  This both simplifies the code and speeds
    // up the generalized internal force calculation while only having a minimal impact on the performance of the
    // Jacobian calculation speed.  The Jacobian calculation is performed in two major pieces.  First is the
    // calculation of the potentially non-symmetric and non-sparse component.  The second pieces is the symmetric
    // and sparse component.

    // =============================================================================
    // Calculate the deformation gradient for all Gauss quadrature points in a single matrix multiplication.  Note
    // that since the shape function derivative matrix is ordered by columns, the resulting deformation gradient
    // will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31 ]
    // FC = [F12  F22  F32 ]
    //      [F13  F23  F33 ]
    // =============================================================================
    ChMatrixNM_col<double, 3 * NIP, 3> FC = m_SD.transpose() * e_bar.transpose();

    //==============================================================================
    //==============================================================================
    // Calculate the potentially non-symmetric and non-sparse component of the Jacobian matrix
    //==============================================================================
    //==============================================================================

    // =============================================================================
    // Calculate the partial derivative of the Green-Largrange strains with respect to the nodal coordinates
    // (transposed).  This calculation is performed in blocks across all the Gauss quadrature points at the same
    // time.  This value should be store in row major memory layout to align with the access patterns for
    // calculating this matrix.
    // PE = [(d epsilon1/d e)GQPnt1' (d epsilon1/d e)GQPnt2' ... (d epsilon1/d e)GQPntNIP'...
    //       (d epsilon2/d e)GQPnt1' (d epsilon2/d e)GQPnt2' ... (d epsilon2/d e)GQPntNIP'...
    //       (d epsilon3/d e)GQPnt1' (d epsilon3/d e)GQPnt2' ... (d epsilon3/d e)GQPntNIP'...
    //       (d epsilon4/d e)GQPnt1' (d epsilon4/d e)GQPnt2' ... (d epsilon4/d e)GQPntNIP'...
    //       (d epsilon5/d e)GQPnt1' (d epsilon5/d e)GQPnt2' ... (d epsilon5/d e)GQPntNIP'...
    //       (d epsilon6/d e)GQPnt1' (d epsilon6/d e)GQPnt2' ... (d epsilon6/d e)GQPntNIP']
    // Note that each partial derivative block shown is placed to the left of the previous block.
    // The explanation of the calculation above is just too long to write it all on a single line.
    // =============================================================================
    ChMatrixDynamic<> PE;
    PE.resize(3 * NSF, 6 * NIP);

    for (auto i = 0; i < NSF; i++) {
        PE.block<1, NIP>(3 * i, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 0).transpose());
        PE.block<1, NIP>(3 * i, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0).transpose());
        PE.block<1, NIP>(3 * i, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 0).transpose());

        PE.block<1, NIP>((3 * i) + 1, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1).transpose());
        PE.block<1, NIP>((3 * i) + 1, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 1).transpose());

        PE.block<1, NIP>((3 * i) + 2, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2).transpose());
        PE.block<1, NIP>((3 * i) + 2, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FC.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FC.template block<NIP, 1>(NIP, 2).transpose());
    }

    // =============================================================================
    // Scale the block deformation gradient matrix by the Kfactor weighting factor for the Jacobian and multiply
    // each Gauss quadrature component by its Gauss quadrature weight times the element Jacobian (kGQ)
    // =============================================================================
    ChMatrixNM_col<double, 3 * NIP, 3> FCscaled = Kfactor * FC;

    for (auto i = 0; i < 3; i++) {
        FCscaled.template block<NIP, 1>(0, i).array() *= m_kGQ.array();
        FCscaled.template block<NIP, 1>(NIP, i).array() *= m_kGQ.array();
        FCscaled.template block<NIP, 1>(2 * NIP, i).array() *= m_kGQ.array();
    }

    // =============================================================================
    // Calculate the scaled partial derivative of the Green-Largrange strains with respect to the nodal coordinates
    // and the other parameters to correctly integrate across the volume of the element.  This calculation is
    // performed in blocks across all the Gauss quadrature points at the same time.  This value should be store in
    // row major memory layout to align with the access patterns for calculating this matrix.
    // =============================================================================

    ChMatrixDynamic<> Scaled_Combined_PE;
    Scaled_Combined_PE.resize(3 * NSF, 6 * NIP);

    for (auto i = 0; i < NSF; i++) {
        Scaled_Combined_PE.block<1, NIP>(3 * i, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 0).transpose());
        Scaled_Combined_PE.block<1, NIP>(3 * i, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 0).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 0).transpose());

        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 1).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 1, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 1).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 1).transpose());

        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 0) =
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 2 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 3 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose()) +
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 4 * NIP) =
            m_SD.block<1, NIP>(i, 2 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(2 * NIP, 2).transpose());
        Scaled_Combined_PE.block<1, NIP>((3 * i) + 2, 5 * NIP) =
            m_SD.block<1, NIP>(i, 1 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(0, 2).transpose()) +
            m_SD.block<1, NIP>(i, 0 * NIP).cwiseProduct(FCscaled.template block<NIP, 1>(NIP, 2).transpose());
    }

    // =============================================================================
    // Get the stiffness tensor in 6x6 matrix form
    // =============================================================================

    const ChMatrixNM<double, 6, 6>& D = GetMaterial()->Get_D();

    // =============================================================================
    // Multiply the scaled and combined partial derivative block matrix by the stiffness matrix for each individual
    // Gauss quadrature point
    // =============================================================================
    ChMatrixDynamic<> DScaled_Combined_PE;
    DScaled_Combined_PE.resize(3 * NSF, 6 * NIP);

    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 0) =
        D(0, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(0, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(0, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(0, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(0, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(0, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, NIP) =
        D(1, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(1, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(1, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(1, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(1, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(1, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 2 * NIP) =
        D(2, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(2, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(2, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(2, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(2, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(2, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 3 * NIP) =
        D(3, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(3, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(3, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(3, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(3, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(3, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 4 * NIP) =
        D(4, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(4, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(4, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(4, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(4, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(4, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);
    DScaled_Combined_PE.template block<3 * NSF, NIP>(0, 5 * NIP) =
        D(5, 0) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 0) +
        D(5, 1) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, NIP) +
        D(5, 2) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 2 * NIP) +
        D(5, 3) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 3 * NIP) +
        D(5, 4) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 4 * NIP) +
        D(5, 5) * Scaled_Combined_PE.block<3 * NSF, NIP>(0, 5 * NIP);

    // =============================================================================
    // Multiply the partial derivative block matrix by the final scaled and combined partial derivative block matrix
    // to obtain the potentially non-symmetric and non-sparse component of the Jacobian matrix
    // =============================================================================

    H = PE * DScaled_Combined_PE.transpose();

    //==============================================================================
    //==============================================================================
    // Calculate the sparse and symmetric component of the Jacobian matrix
    //==============================================================================
    //==============================================================================

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // The result is then scaled by minus the Gauss quadrature weight times the element Jacobian at the
    // corresponding Gauss point (m_kGQ) for efficiency.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*E11 = kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP E1_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(0, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(0, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(0, 2));
    E1_Block.array() -= 1;
    E1_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E2 = kGQ*E22 = kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP E2_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E2_Block.array() -= 1;
    E2_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E3 = kGQ*E33 = kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP E3_Block = FC.template block<NIP, 1>(2 * NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(2 * NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(2 * NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E3_Block.array() -= 1;
    E3_Block.array() *= 0.5 * m_kGQ.array();

    // Each entry in E4 = kGQ*2*E23 = kGQ*(F12*F13+F22*F23+F32*F33)
    VectorNIP E4_Block = FC.template block<NIP, 1>(NIP, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(NIP, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(NIP, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E4_Block.array() *= m_kGQ.array();

    // Each entry in E5 = kGQ*2*E13 = kGQ*(F11*F13+F21*F23+F31*F33)
    VectorNIP E5_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(2 * NIP, 2));
    E5_Block.array() *= m_kGQ.array();

    // Each entry in E6 = kGQ*2*E12 = (F11*F12+F21*F22+F31*F32)
    VectorNIP E6_Block = FC.template block<NIP, 1>(0, 0).cwiseProduct(FC.template block<NIP, 1>(NIP, 0)) +
                         FC.template block<NIP, 1>(0, 1).cwiseProduct(FC.template block<NIP, 1>(NIP, 1)) +
                         FC.template block<NIP, 1>(0, 2).cwiseProduct(FC.template block<NIP, 1>(NIP, 2));
    E6_Block.array() *= m_kGQ.array();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have been scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point to make the calculation of the 2nd
    // Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP SPK2_1_Block = D(0, 0) * E1_Block + D(0, 1) * E2_Block + D(0, 2) * E3_Block + D(0, 3) * E4_Block +
                             D(0, 4) * E5_Block + D(0, 5) * E6_Block;
    VectorNIP SPK2_2_Block = D(1, 0) * E1_Block + D(1, 1) * E2_Block + D(1, 2) * E3_Block + D(1, 3) * E4_Block +
                             D(1, 4) * E5_Block + D(1, 5) * E6_Block;
    VectorNIP SPK2_3_Block = D(2, 0) * E1_Block + D(2, 1) * E2_Block + D(2, 2) * E3_Block + D(2, 3) * E4_Block +
                             D(2, 4) * E5_Block + D(2, 5) * E6_Block;
    VectorNIP SPK2_4_Block = D(3, 0) * E1_Block + D(3, 1) * E2_Block + D(3, 2) * E3_Block + D(3, 3) * E4_Block +
                             D(3, 4) * E5_Block + D(3, 5) * E6_Block;
    VectorNIP SPK2_5_Block = D(4, 0) * E1_Block + D(4, 1) * E2_Block + D(4, 2) * E3_Block + D(4, 3) * E4_Block +
                             D(4, 4) * E5_Block + D(4, 5) * E6_Block;
    VectorNIP SPK2_6_Block = D(5, 0) * E1_Block + D(5, 1) * E2_Block + D(5, 2) * E3_Block + D(5, 3) * E4_Block +
                             D(5, 4) * E5_Block + D(5, 5) * E6_Block;

    // =============================================================================
    // Multiply the shape function derivative matrix by the 2nd Piola-Kirchoff stresses for each corresponding Gauss
    // quadrature point
    // =============================================================================

    ChMatrixNM<double, NSF, 3 * NIP> S_scaled_SD;

    for (auto i = 0; i < NSF; i++) {
        S_scaled_SD.template block<1, NIP>(i, 0) =
            SPK2_1_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_6_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_5_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));

        S_scaled_SD.template block<1, NIP>(i, NIP) =
            SPK2_6_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_2_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_4_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));

        S_scaled_SD.template block<1, NIP>(i, 2 * NIP) =
            SPK2_5_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 0 * NIP)) +
            SPK2_4_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 1 * NIP)) +
            SPK2_3_Block.transpose().cwiseProduct(m_SD.block<1, NIP>(i, 2 * NIP));
    }

    // =============================================================================
    // Calculate just the non-sparse upper triangular entires of the sparse and symmetric component of the Jacobian
    // matrix, combine this with the scaled mass matrix, and then expand them out to full size by summing the
    // contribution into the correct locations of the full sized Jacobian matrix
    // =============================================================================

    // Add in the Mass Matrix Which is Stored in Compact Upper Triangular Form
    ChVectorN<double, (NSF * (NSF + 1)) / 2> ScaledMassMatrix = Mfactor * m_MassMatrix;

    unsigned int idx = 0;
    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = i; j < NSF; j++) {
            double d = Kfactor * m_SD.row(i) * S_scaled_SD.row(j).transpose();
            d += ScaledMassMatrix(idx);

            H(3 * i, 3 * j) += d;
            H(3 * i + 1, 3 * j + 1) += d;
            H(3 * i + 2, 3 * j + 2) += d;
            if (i != j) {
                H(3 * j, 3 * i) += d;
                H(3 * j + 1, 3 * i + 1) += d;
                H(3 * j + 2, 3 * i + 2) += d;
            }
            idx++;
        }
    }
}

void ChElementHexaANCF_3843::ComputeInternalJacobianPreInt(ChMatrixRef& H,
                                                           double Kfactor,
                                                           double Rfactor,
                                                           double Mfactor) {
    // Calculate the Jacobian of the generalize internal force vector using the "Pre-Integration" style of method
    // assuming a linear viscoelastic material model (single term damping model).  For this style of method, the
    // components of the generalized internal force vector and its Jacobian that need to be integrated across the volume
    // are calculated once prior to the start of the simulation.  For this method, the in-simulation calculations are
    // independent of the number of Gauss quadrature points used throughout the entire element.  Since computationally
    // expensive quantities are required for both the generalized internal force vector and its Jacobian, these values
    // were cached for reuse during this Jacobian calculation.

    Matrix3xN e_bar;
    Matrix3xN e_bar_dot;

    CalcCoordMatrix(e_bar);
    CalcCoordDerivMatrix(e_bar_dot);

    // Build the [9 x NSF^2] matrix containing the combined scaled nodal coordinates and their time derivatives.
    Matrix3xN temp = (Kfactor + m_Alpha * Rfactor) * e_bar + (m_Alpha * Kfactor) * e_bar_dot;
    ChMatrixNM<double, 1, NSF> tempRow0 = temp.template block<1, NSF>(0, 0);
    ChMatrixNM<double, 1, NSF> tempRow1 = temp.template block<1, NSF>(1, 0);
    ChMatrixNM<double, 1, NSF> tempRow2 = temp.template block<1, NSF>(2, 0);
    ChMatrixNM_col<double, 9, NSF * NSF> PI2;

    for (unsigned int v = 0; v < NSF; v++) {
        PI2.template block<3, NSF>(0, NSF * v) = e_bar.template block<3, 1>(0, v) * tempRow0;
        PI2.template block<3, NSF>(3, NSF * v) = e_bar.template block<3, 1>(0, v) * tempRow1;
        PI2.template block<3, NSF>(6, NSF * v) = e_bar.template block<3, 1>(0, v) * tempRow2;
    }

    // Calculate the matrix containing the dense part of the Jacobian matrix in a reordered form. This is then reordered
    // from its [9 x NSF^2] form into its required [3*NSF x 3*NSF] form
    ChMatrixDynamic_col<> K2 = -PI2 * m_O2;

    for (unsigned int k = 0; k < NSF; k++) {
        for (unsigned int f = 0; f < NSF; f++) {
            H.block<3, 1>(3 * k, 3 * f) = K2.template block<3, 1>(0, NSF * f + k);
            H.block<3, 1>(3 * k, 3 * f + 1) = K2.template block<3, 1>(3, NSF * f + k);
            H.block<3, 1>(3 * k, 3 * f + 2) = K2.template block<3, 1>(6, NSF * f + k);
        }
    }

    // Add in the sparse (blocks times the 3x3 identity matrix) component of the Jacobian that was already calculated as
    // part of the generalized internal force calculations as well as the Mass Matrix which is stored in compact upper
    // triangular form
    MatrixNxN K_K13Compact = -Kfactor * m_K13Compact;
    ChVectorN<double, (NSF * (NSF + 1)) / 2> ScaledMassMatrix = Mfactor * m_MassMatrix;

    for (unsigned int i = 0; i < NSF; i++) {
        for (unsigned int j = 0; j < NSF; j++) {
            unsigned int idx;
            // Convert from a (i,j) index to a linear index into the Mass Matrix in Compact Upper Triangular Form
            // https://math.stackexchange.com/questions/2134011/conversion-of-upper-triangle-linear-index-from-index-on-symmetrical-array
            if (j >= i) {
                idx = (NSF * (NSF - 1) - (NSF - i) * (NSF - i - 1)) / 2 + j;
            } else {
                idx = (NSF * (NSF - 1) - (NSF - j) * (NSF - j - 1)) / 2 + i;
            }

            double d = ScaledMassMatrix(idx) + K_K13Compact(i, j);
            H(3 * i, 3 * j) += d;
            H(3 * i + 1, 3 * j + 1) += d;
            H(3 * i + 2, 3 * j + 2) += d;
        }
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

// Nx1 Vector Form of the Normalized Shape Functions
// [s1; s2; s3; ...]

void ChElementHexaANCF_3843::Calc_Sxi_compact(VectorN& Sxi_compact, double xi, double eta, double zeta) {
    Sxi_compact(0) =
        0.0625 * (zeta - 1) * (xi - 1) * (eta - 1) * (eta * eta + eta + xi * xi + xi + zeta * zeta + zeta - 2);
    Sxi_compact(1) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta - 1) * (eta - 1);
    Sxi_compact(2) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta - 1) * (xi - 1);
    Sxi_compact(3) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi - 1) * (eta - 1);
    Sxi_compact(4) =
        -0.0625 * (zeta - 1) * (xi + 1) * (eta - 1) * (eta * eta + eta + xi * xi - xi + zeta * zeta + zeta - 2);
    Sxi_compact(5) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta - 1) * (eta - 1);
    Sxi_compact(6) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta - 1) * (xi + 1);
    Sxi_compact(7) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi + 1) * (eta - 1);
    Sxi_compact(8) =
        0.0625 * (zeta - 1) * (xi + 1) * (eta + 1) * (eta * eta - eta + xi * xi - xi + zeta * zeta + zeta - 2);
    Sxi_compact(9) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta - 1) * (eta + 1);
    Sxi_compact(10) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta - 1) * (xi + 1);
    Sxi_compact(11) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi + 1) * (eta + 1);
    Sxi_compact(12) =
        -0.0625 * (zeta - 1) * (xi - 1) * (eta + 1) * (eta * eta - eta + xi * xi + xi + zeta * zeta + zeta - 2);
    Sxi_compact(13) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta - 1) * (eta + 1);
    Sxi_compact(14) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta - 1) * (xi - 1);
    Sxi_compact(15) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi - 1) * (eta + 1);
    Sxi_compact(16) =
        -0.0625 * (zeta + 1) * (xi - 1) * (eta - 1) * (eta * eta + eta + xi * xi + xi + zeta * zeta - zeta - 2);
    Sxi_compact(17) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta + 1) * (eta - 1);
    Sxi_compact(18) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta + 1) * (xi - 1);
    Sxi_compact(19) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi - 1) * (eta - 1);
    Sxi_compact(20) =
        0.0625 * (zeta + 1) * (xi + 1) * (eta - 1) * (eta * eta + eta + xi * xi - xi + zeta * zeta - zeta - 2);
    Sxi_compact(21) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta + 1) * (eta - 1);
    Sxi_compact(22) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta + 1) * (xi + 1);
    Sxi_compact(23) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi + 1) * (eta - 1);
    Sxi_compact(24) =
        -0.0625 * (zeta + 1) * (xi + 1) * (eta + 1) * (eta * eta - eta + xi * xi - xi + zeta * zeta - zeta - 2);
    Sxi_compact(25) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta + 1) * (eta + 1);
    Sxi_compact(26) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta + 1) * (xi + 1);
    Sxi_compact(27) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi + 1) * (eta + 1);
    Sxi_compact(28) =
        0.0625 * (zeta + 1) * (xi - 1) * (eta + 1) * (eta * eta - eta + xi * xi + xi + zeta * zeta - zeta - 2);
    Sxi_compact(29) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta + 1) * (eta + 1);
    Sxi_compact(30) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta + 1) * (xi - 1);
    Sxi_compact(31) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi - 1) * (eta + 1);
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to xi
// [s1; s2; s3; ...]

void ChElementHexaANCF_3843::Calc_Sxi_xi_compact(VectorN& Sxi_xi_compact, double xi, double eta, double zeta) {
    Sxi_xi_compact(0) = 0.0625 * (zeta - 1) * (eta - 1) * (eta * eta + eta + 3 * xi * xi + zeta * zeta + zeta - 3);
    Sxi_xi_compact(1) = 0.03125 * m_lenX * (3 * xi + 1) * (xi - 1) * (zeta - 1) * (eta - 1);
    Sxi_xi_compact(2) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta - 1);
    Sxi_xi_compact(3) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (eta - 1);
    Sxi_xi_compact(4) = -0.0625 * (zeta - 1) * (eta - 1) * (eta * eta + eta + 3 * xi * xi + zeta * zeta + zeta - 3);
    Sxi_xi_compact(5) = 0.03125 * m_lenX * (xi + 1) * (3 * xi - 1) * (zeta - 1) * (eta - 1);
    Sxi_xi_compact(6) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta - 1);
    Sxi_xi_compact(7) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (eta - 1);
    Sxi_xi_compact(8) = 0.0625 * (zeta - 1) * (eta + 1) * (eta * eta - eta + 3 * xi * xi + zeta * zeta + zeta - 3);
    Sxi_xi_compact(9) = -0.03125 * m_lenX * (xi + 1) * (3 * xi - 1) * (zeta - 1) * (eta + 1);
    Sxi_xi_compact(10) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta - 1);
    Sxi_xi_compact(11) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (eta + 1);
    Sxi_xi_compact(12) = -0.0625 * (zeta - 1) * (eta + 1) * (eta * eta - eta + 3 * xi * xi + zeta * zeta + zeta - 3);
    Sxi_xi_compact(13) = -0.03125 * m_lenX * (3 * xi + 1) * (xi - 1) * (zeta - 1) * (eta + 1);
    Sxi_xi_compact(14) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta - 1);
    Sxi_xi_compact(15) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (eta + 1);
    Sxi_xi_compact(16) = -0.0625 * (zeta + 1) * (eta - 1) * (eta * eta + eta + 3 * xi * xi + zeta * zeta - zeta - 3);
    Sxi_xi_compact(17) = -0.03125 * m_lenX * (3 * xi + 1) * (xi - 1) * (zeta + 1) * (eta - 1);
    Sxi_xi_compact(18) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta + 1);
    Sxi_xi_compact(19) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (eta - 1);
    Sxi_xi_compact(20) = 0.0625 * (zeta + 1) * (eta - 1) * (eta * eta + eta + 3 * xi * xi + zeta * zeta - zeta - 3);
    Sxi_xi_compact(21) = -0.03125 * m_lenX * (xi + 1) * (3 * xi - 1) * (zeta + 1) * (eta - 1);
    Sxi_xi_compact(22) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (zeta + 1);
    Sxi_xi_compact(23) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (eta - 1);
    Sxi_xi_compact(24) = -0.0625 * (zeta + 1) * (eta + 1) * (eta * eta - eta + 3 * xi * xi + zeta * zeta - zeta - 3);
    Sxi_xi_compact(25) = 0.03125 * m_lenX * (xi + 1) * (3 * xi - 1) * (zeta + 1) * (eta + 1);
    Sxi_xi_compact(26) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta + 1);
    Sxi_xi_compact(27) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (eta + 1);
    Sxi_xi_compact(28) = 0.0625 * (zeta + 1) * (eta + 1) * (eta * eta - eta + 3 * xi * xi + zeta * zeta - zeta - 3);
    Sxi_xi_compact(29) = 0.03125 * m_lenX * (3 * xi + 1) * (xi - 1) * (zeta + 1) * (eta + 1);
    Sxi_xi_compact(30) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (zeta + 1);
    Sxi_xi_compact(31) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (eta + 1);
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to eta
// [s1; s2; s3; ...]

void ChElementHexaANCF_3843::Calc_Sxi_eta_compact(VectorN& Sxi_eta_compact, double xi, double eta, double zeta) {
    Sxi_eta_compact(0) = 0.0625 * (zeta - 1) * (xi - 1) * (3 * eta * eta + xi * xi + xi + zeta * zeta + zeta - 3);
    Sxi_eta_compact(1) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta - 1);
    Sxi_eta_compact(2) = 0.03125 * m_lenY * (3 * eta + 1) * (eta - 1) * (zeta - 1) * (xi - 1);
    Sxi_eta_compact(3) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi - 1);
    Sxi_eta_compact(4) = -0.0625 * (zeta - 1) * (xi + 1) * (3 * eta * eta + xi * xi - xi + zeta * zeta + zeta - 3);
    Sxi_eta_compact(5) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta - 1);
    Sxi_eta_compact(6) = -0.03125 * m_lenY * (3 * eta + 1) * (eta - 1) * (zeta - 1) * (xi + 1);
    Sxi_eta_compact(7) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi + 1);
    Sxi_eta_compact(8) = 0.0625 * (zeta - 1) * (xi + 1) * (3 * eta * eta + xi * xi - xi + zeta * zeta + zeta - 3);
    Sxi_eta_compact(9) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta - 1);
    Sxi_eta_compact(10) = -0.03125 * m_lenY * (eta + 1) * (3 * eta - 1) * (zeta - 1) * (xi + 1);
    Sxi_eta_compact(11) = 0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi + 1);
    Sxi_eta_compact(12) = -0.0625 * (zeta - 1) * (xi - 1) * (3 * eta * eta + xi * xi + xi + zeta * zeta + zeta - 3);
    Sxi_eta_compact(13) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta - 1);
    Sxi_eta_compact(14) = 0.03125 * m_lenY * (eta + 1) * (3 * eta - 1) * (zeta - 1) * (xi - 1);
    Sxi_eta_compact(15) = -0.03125 * m_lenZ * (zeta + 1) * (zeta - 1) * (zeta - 1) * (xi - 1);
    Sxi_eta_compact(16) = -0.0625 * (zeta + 1) * (xi - 1) * (3 * eta * eta + xi * xi + xi + zeta * zeta - zeta - 3);
    Sxi_eta_compact(17) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta + 1);
    Sxi_eta_compact(18) = -0.03125 * m_lenY * (3 * eta + 1) * (eta - 1) * (zeta + 1) * (xi - 1);
    Sxi_eta_compact(19) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi - 1);
    Sxi_eta_compact(20) = 0.0625 * (zeta + 1) * (xi + 1) * (3 * eta * eta + xi * xi - xi + zeta * zeta - zeta - 3);
    Sxi_eta_compact(21) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta + 1);
    Sxi_eta_compact(22) = 0.03125 * m_lenY * (3 * eta + 1) * (eta - 1) * (zeta + 1) * (xi + 1);
    Sxi_eta_compact(23) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi + 1);
    Sxi_eta_compact(24) = -0.0625 * (zeta + 1) * (xi + 1) * (3 * eta * eta + xi * xi - xi + zeta * zeta - zeta - 3);
    Sxi_eta_compact(25) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (zeta + 1);
    Sxi_eta_compact(26) = 0.03125 * m_lenY * (eta + 1) * (3 * eta - 1) * (zeta + 1) * (xi + 1);
    Sxi_eta_compact(27) = 0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi + 1);
    Sxi_eta_compact(28) = 0.0625 * (zeta + 1) * (xi - 1) * (3 * eta * eta + xi * xi + xi + zeta * zeta - zeta - 3);
    Sxi_eta_compact(29) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (zeta + 1);
    Sxi_eta_compact(30) = -0.03125 * m_lenY * (eta + 1) * (3 * eta - 1) * (zeta + 1) * (xi - 1);
    Sxi_eta_compact(31) = -0.03125 * m_lenZ * (zeta - 1) * (zeta + 1) * (zeta + 1) * (xi - 1);
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to zeta
// [s1; s2; s3; ...]

void ChElementHexaANCF_3843::Calc_Sxi_zeta_compact(VectorN& Sxi_zeta_compact, double xi, double eta, double zeta) {
    Sxi_zeta_compact(0) = 0.0625 * (xi - 1) * (eta - 1) * (eta * eta + eta + xi * xi + xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(1) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta - 1);
    Sxi_zeta_compact(2) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi - 1);
    Sxi_zeta_compact(3) = 0.03125 * m_lenZ * (3 * zeta + 1) * (zeta - 1) * (xi - 1) * (eta - 1);
    Sxi_zeta_compact(4) = -0.0625 * (xi + 1) * (eta - 1) * (eta * eta + eta + xi * xi - xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(5) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta - 1);
    Sxi_zeta_compact(6) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi + 1);
    Sxi_zeta_compact(7) = -0.03125 * m_lenZ * (3 * zeta + 1) * (zeta - 1) * (xi + 1) * (eta - 1);
    Sxi_zeta_compact(8) = 0.0625 * (xi + 1) * (eta + 1) * (eta * eta - eta + xi * xi - xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(9) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta + 1);
    Sxi_zeta_compact(10) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi + 1);
    Sxi_zeta_compact(11) = 0.03125 * m_lenZ * (3 * zeta + 1) * (zeta - 1) * (xi + 1) * (eta + 1);
    Sxi_zeta_compact(12) = -0.0625 * (xi - 1) * (eta + 1) * (eta * eta - eta + xi * xi + xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(13) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta + 1);
    Sxi_zeta_compact(14) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi - 1);
    Sxi_zeta_compact(15) = -0.03125 * m_lenZ * (3 * zeta + 1) * (zeta - 1) * (xi - 1) * (eta + 1);
    Sxi_zeta_compact(16) = -0.0625 * (xi - 1) * (eta - 1) * (eta * eta + eta + xi * xi + xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(17) = -0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta - 1);
    Sxi_zeta_compact(18) = -0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi - 1);
    Sxi_zeta_compact(19) = 0.03125 * m_lenZ * (zeta + 1) * (3 * zeta - 1) * (xi - 1) * (eta - 1);
    Sxi_zeta_compact(20) = 0.0625 * (xi + 1) * (eta - 1) * (eta * eta + eta + xi * xi - xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(21) = -0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta - 1);
    Sxi_zeta_compact(22) = 0.03125 * m_lenY * (eta + 1) * (eta - 1) * (eta - 1) * (xi + 1);
    Sxi_zeta_compact(23) = -0.03125 * m_lenZ * (zeta + 1) * (3 * zeta - 1) * (xi + 1) * (eta - 1);
    Sxi_zeta_compact(24) = -0.0625 * (xi + 1) * (eta + 1) * (eta * eta - eta + xi * xi - xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(25) = 0.03125 * m_lenX * (xi - 1) * (xi + 1) * (xi + 1) * (eta + 1);
    Sxi_zeta_compact(26) = 0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi + 1);
    Sxi_zeta_compact(27) = 0.03125 * m_lenZ * (zeta + 1) * (3 * zeta - 1) * (xi + 1) * (eta + 1);
    Sxi_zeta_compact(28) = 0.0625 * (xi - 1) * (eta + 1) * (eta * eta - eta + xi * xi + xi + 3 * zeta * zeta - 3);
    Sxi_zeta_compact(29) = 0.03125 * m_lenX * (xi + 1) * (xi - 1) * (xi - 1) * (eta + 1);
    Sxi_zeta_compact(30) = -0.03125 * m_lenY * (eta - 1) * (eta + 1) * (eta + 1) * (xi - 1);
    Sxi_zeta_compact(31) = -0.03125 * m_lenZ * (zeta + 1) * (3 * zeta - 1) * (xi - 1) * (eta + 1);
}

// Nx3 compact form of the partial derivatives of Normalized Shape Functions with respect to xi, eta, and zeta by
// columns

void ChElementHexaANCF_3843::Calc_Sxi_D(MatrixNx3c& Sxi_D, double xi, double eta, double zeta) {
    VectorN Sxi_D_col;
    Calc_Sxi_xi_compact(Sxi_D_col, xi, eta, zeta);
    Sxi_D.col(0) = Sxi_D_col;

    Calc_Sxi_eta_compact(Sxi_D_col, xi, eta, zeta);
    Sxi_D.col(1) = Sxi_D_col;

    Calc_Sxi_zeta_compact(Sxi_D_col, xi, eta, zeta);
    Sxi_D.col(2) = Sxi_D_col;
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

void ChElementHexaANCF_3843::CalcCoordVector(Vector3N& e) {
    e.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    e.segment(3, 3) = m_nodes[0]->GetD().eigen();
    e.segment(6, 3) = m_nodes[0]->GetDD().eigen();
    e.segment(9, 3) = m_nodes[0]->GetDDD().eigen();

    e.segment(12, 3) = m_nodes[1]->GetPos().eigen();
    e.segment(15, 3) = m_nodes[1]->GetD().eigen();
    e.segment(18, 3) = m_nodes[1]->GetDD().eigen();
    e.segment(21, 3) = m_nodes[1]->GetDDD().eigen();

    e.segment(24, 3) = m_nodes[2]->GetPos().eigen();
    e.segment(27, 3) = m_nodes[2]->GetD().eigen();
    e.segment(30, 3) = m_nodes[2]->GetDD().eigen();
    e.segment(33, 3) = m_nodes[2]->GetDDD().eigen();

    e.segment(36, 3) = m_nodes[3]->GetPos().eigen();
    e.segment(39, 3) = m_nodes[3]->GetD().eigen();
    e.segment(42, 3) = m_nodes[3]->GetDD().eigen();
    e.segment(45, 3) = m_nodes[3]->GetDDD().eigen();

    e.segment(48, 3) = m_nodes[4]->GetPos().eigen();
    e.segment(51, 3) = m_nodes[4]->GetD().eigen();
    e.segment(54, 3) = m_nodes[4]->GetDD().eigen();
    e.segment(57, 3) = m_nodes[4]->GetDDD().eigen();

    e.segment(60, 3) = m_nodes[5]->GetPos().eigen();
    e.segment(63, 3) = m_nodes[5]->GetD().eigen();
    e.segment(66, 3) = m_nodes[5]->GetDD().eigen();
    e.segment(69, 3) = m_nodes[5]->GetDDD().eigen();

    e.segment(72, 3) = m_nodes[6]->GetPos().eigen();
    e.segment(75, 3) = m_nodes[6]->GetD().eigen();
    e.segment(78, 3) = m_nodes[6]->GetDD().eigen();
    e.segment(81, 3) = m_nodes[6]->GetDDD().eigen();

    e.segment(84, 3) = m_nodes[7]->GetPos().eigen();
    e.segment(87, 3) = m_nodes[7]->GetD().eigen();
    e.segment(90, 3) = m_nodes[7]->GetDD().eigen();
    e.segment(93, 3) = m_nodes[7]->GetDDD().eigen();
}

void ChElementHexaANCF_3843::CalcCoordMatrix(Matrix3xN& ebar) {
    ebar.col(0) = m_nodes[0]->GetPos().eigen();
    ebar.col(1) = m_nodes[0]->GetD().eigen();
    ebar.col(2) = m_nodes[0]->GetDD().eigen();
    ebar.col(3) = m_nodes[0]->GetDDD().eigen();

    ebar.col(4) = m_nodes[1]->GetPos().eigen();
    ebar.col(5) = m_nodes[1]->GetD().eigen();
    ebar.col(6) = m_nodes[1]->GetDD().eigen();
    ebar.col(7) = m_nodes[1]->GetDDD().eigen();

    ebar.col(8) = m_nodes[2]->GetPos().eigen();
    ebar.col(9) = m_nodes[2]->GetD().eigen();
    ebar.col(10) = m_nodes[2]->GetDD().eigen();
    ebar.col(11) = m_nodes[2]->GetDDD().eigen();

    ebar.col(12) = m_nodes[3]->GetPos().eigen();
    ebar.col(13) = m_nodes[3]->GetD().eigen();
    ebar.col(14) = m_nodes[3]->GetDD().eigen();
    ebar.col(15) = m_nodes[3]->GetDDD().eigen();

    ebar.col(16) = m_nodes[4]->GetPos().eigen();
    ebar.col(17) = m_nodes[4]->GetD().eigen();
    ebar.col(18) = m_nodes[4]->GetDD().eigen();
    ebar.col(19) = m_nodes[4]->GetDDD().eigen();

    ebar.col(20) = m_nodes[5]->GetPos().eigen();
    ebar.col(21) = m_nodes[5]->GetD().eigen();
    ebar.col(22) = m_nodes[5]->GetDD().eigen();
    ebar.col(23) = m_nodes[5]->GetDDD().eigen();

    ebar.col(24) = m_nodes[6]->GetPos().eigen();
    ebar.col(25) = m_nodes[6]->GetD().eigen();
    ebar.col(26) = m_nodes[6]->GetDD().eigen();
    ebar.col(27) = m_nodes[6]->GetDDD().eigen();

    ebar.col(28) = m_nodes[7]->GetPos().eigen();
    ebar.col(29) = m_nodes[7]->GetD().eigen();
    ebar.col(30) = m_nodes[7]->GetDD().eigen();
    ebar.col(31) = m_nodes[7]->GetDDD().eigen();
}

void ChElementHexaANCF_3843::CalcCoordDerivVector(Vector3N& edot) {
    edot.segment(0, 3) = m_nodes[0]->GetPos_dt().eigen();
    edot.segment(3, 3) = m_nodes[0]->GetD_dt().eigen();
    edot.segment(6, 3) = m_nodes[0]->GetDD_dt().eigen();
    edot.segment(9, 3) = m_nodes[0]->GetDDD_dt().eigen();

    edot.segment(12, 3) = m_nodes[1]->GetPos_dt().eigen();
    edot.segment(15, 3) = m_nodes[1]->GetD_dt().eigen();
    edot.segment(18, 3) = m_nodes[1]->GetDD_dt().eigen();
    edot.segment(21, 3) = m_nodes[1]->GetDDD_dt().eigen();

    edot.segment(24, 3) = m_nodes[2]->GetPos_dt().eigen();
    edot.segment(27, 3) = m_nodes[2]->GetD_dt().eigen();
    edot.segment(30, 3) = m_nodes[2]->GetDD_dt().eigen();
    edot.segment(33, 3) = m_nodes[2]->GetDDD_dt().eigen();

    edot.segment(36, 3) = m_nodes[3]->GetPos_dt().eigen();
    edot.segment(39, 3) = m_nodes[3]->GetD_dt().eigen();
    edot.segment(42, 3) = m_nodes[3]->GetDD_dt().eigen();
    edot.segment(45, 3) = m_nodes[3]->GetDDD_dt().eigen();

    edot.segment(48, 3) = m_nodes[4]->GetPos_dt().eigen();
    edot.segment(51, 3) = m_nodes[4]->GetD_dt().eigen();
    edot.segment(54, 3) = m_nodes[4]->GetDD_dt().eigen();
    edot.segment(57, 3) = m_nodes[4]->GetDDD_dt().eigen();

    edot.segment(60, 3) = m_nodes[5]->GetPos_dt().eigen();
    edot.segment(63, 3) = m_nodes[5]->GetD_dt().eigen();
    edot.segment(66, 3) = m_nodes[5]->GetDD_dt().eigen();
    edot.segment(69, 3) = m_nodes[5]->GetDDD_dt().eigen();

    edot.segment(72, 3) = m_nodes[6]->GetPos_dt().eigen();
    edot.segment(75, 3) = m_nodes[6]->GetD_dt().eigen();
    edot.segment(78, 3) = m_nodes[6]->GetDD_dt().eigen();
    edot.segment(81, 3) = m_nodes[6]->GetDDD_dt().eigen();

    edot.segment(84, 3) = m_nodes[7]->GetPos_dt().eigen();
    edot.segment(87, 3) = m_nodes[7]->GetD_dt().eigen();
    edot.segment(90, 3) = m_nodes[7]->GetDD_dt().eigen();
    edot.segment(93, 3) = m_nodes[7]->GetDDD_dt().eigen();
}

void ChElementHexaANCF_3843::CalcCoordDerivMatrix(Matrix3xN& ebardot) {
    ebardot.col(0) = m_nodes[0]->GetPos_dt().eigen();
    ebardot.col(1) = m_nodes[0]->GetD_dt().eigen();
    ebardot.col(2) = m_nodes[0]->GetDD_dt().eigen();
    ebardot.col(3) = m_nodes[0]->GetDDD_dt().eigen();

    ebardot.col(4) = m_nodes[1]->GetPos_dt().eigen();
    ebardot.col(5) = m_nodes[1]->GetD_dt().eigen();
    ebardot.col(6) = m_nodes[1]->GetDD_dt().eigen();
    ebardot.col(7) = m_nodes[1]->GetDDD_dt().eigen();

    ebardot.col(8) = m_nodes[2]->GetPos_dt().eigen();
    ebardot.col(9) = m_nodes[2]->GetD_dt().eigen();
    ebardot.col(10) = m_nodes[2]->GetDD_dt().eigen();
    ebardot.col(11) = m_nodes[2]->GetDDD_dt().eigen();

    ebardot.col(12) = m_nodes[3]->GetPos_dt().eigen();
    ebardot.col(13) = m_nodes[3]->GetD_dt().eigen();
    ebardot.col(14) = m_nodes[3]->GetDD_dt().eigen();
    ebardot.col(15) = m_nodes[3]->GetDDD_dt().eigen();

    ebardot.col(16) = m_nodes[4]->GetPos_dt().eigen();
    ebardot.col(17) = m_nodes[4]->GetD_dt().eigen();
    ebardot.col(18) = m_nodes[4]->GetDD_dt().eigen();
    ebardot.col(19) = m_nodes[4]->GetDDD_dt().eigen();

    ebardot.col(20) = m_nodes[5]->GetPos_dt().eigen();
    ebardot.col(21) = m_nodes[5]->GetD_dt().eigen();
    ebardot.col(22) = m_nodes[5]->GetDD_dt().eigen();
    ebardot.col(23) = m_nodes[5]->GetDDD_dt().eigen();

    ebardot.col(24) = m_nodes[6]->GetPos_dt().eigen();
    ebardot.col(25) = m_nodes[6]->GetD_dt().eigen();
    ebardot.col(26) = m_nodes[6]->GetDD_dt().eigen();
    ebardot.col(27) = m_nodes[6]->GetDDD_dt().eigen();

    ebardot.col(28) = m_nodes[7]->GetPos_dt().eigen();
    ebardot.col(29) = m_nodes[7]->GetD_dt().eigen();
    ebardot.col(30) = m_nodes[7]->GetDD_dt().eigen();
    ebardot.col(31) = m_nodes[7]->GetDDD_dt().eigen();
}

void ChElementHexaANCF_3843::CalcCombinedCoordMatrix(MatrixNx6& ebar_ebardot) {
    ebar_ebardot.template block<1, 3>(0, 0) = m_nodes[0]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(0, 3) = m_nodes[0]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(1, 0) = m_nodes[0]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(1, 3) = m_nodes[0]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(2, 0) = m_nodes[0]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(2, 3) = m_nodes[0]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(3, 0) = m_nodes[0]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(3, 3) = m_nodes[0]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(4, 0) = m_nodes[1]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(4, 3) = m_nodes[1]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(5, 0) = m_nodes[1]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(5, 3) = m_nodes[1]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(6, 0) = m_nodes[1]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(6, 3) = m_nodes[1]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(7, 0) = m_nodes[1]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(7, 3) = m_nodes[1]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(8, 0) = m_nodes[2]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(8, 3) = m_nodes[2]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(9, 0) = m_nodes[2]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(9, 3) = m_nodes[2]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(10, 0) = m_nodes[2]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(10, 3) = m_nodes[2]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(11, 0) = m_nodes[2]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(11, 3) = m_nodes[2]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(12, 0) = m_nodes[3]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(12, 3) = m_nodes[3]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(13, 0) = m_nodes[3]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(13, 3) = m_nodes[3]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(14, 0) = m_nodes[3]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(14, 3) = m_nodes[3]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(15, 0) = m_nodes[3]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(15, 3) = m_nodes[3]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(16, 0) = m_nodes[4]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(16, 3) = m_nodes[4]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(17, 0) = m_nodes[4]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(17, 3) = m_nodes[4]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(18, 0) = m_nodes[4]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(18, 3) = m_nodes[4]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(19, 0) = m_nodes[4]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(19, 3) = m_nodes[4]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(20, 0) = m_nodes[5]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(20, 3) = m_nodes[5]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(21, 0) = m_nodes[5]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(21, 3) = m_nodes[5]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(22, 0) = m_nodes[5]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(22, 3) = m_nodes[5]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(23, 0) = m_nodes[5]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(23, 3) = m_nodes[5]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(24, 0) = m_nodes[6]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(24, 3) = m_nodes[6]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(25, 0) = m_nodes[6]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(25, 3) = m_nodes[6]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(26, 0) = m_nodes[6]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(26, 3) = m_nodes[6]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(27, 0) = m_nodes[6]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(27, 3) = m_nodes[6]->GetDDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(28, 0) = m_nodes[7]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(28, 3) = m_nodes[7]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(29, 0) = m_nodes[7]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(29, 3) = m_nodes[7]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(30, 0) = m_nodes[7]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(30, 3) = m_nodes[7]->GetDD_dt().eigen();
    ebar_ebardot.template block<1, 3>(31, 0) = m_nodes[7]->GetDDD().eigen();
    ebar_ebardot.template block<1, 3>(31, 3) = m_nodes[7]->GetDDD_dt().eigen();
}

// Calculate the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

void ChElementHexaANCF_3843::Calc_J_0xi(ChMatrix33<double>& J_0xi, double xi, double eta, double zeta) {
    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    J_0xi = m_ebar0 * Sxi_D;
}

// Calculate the determinant of the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

double ChElementHexaANCF_3843::Calc_det_J_0xi(double xi, double eta, double zeta) {
    ChMatrix33<double> J_0xi;
    Calc_J_0xi(J_0xi, xi, eta, zeta);

    return (J_0xi.determinant());
}

////////////////////////////////////////////////////////////////

//#ifndef CH_QUADRATURE_STATIC_TABLES
#define CH_QUADRATURE_STATIC_TABLES 10
ChQuadratureTables static_tables_3843(1, CH_QUADRATURE_STATIC_TABLES);
//#endif // !CH_QUADRATURE_STATIC_TABLES

ChQuadratureTables* ChElementHexaANCF_3843::GetStaticGQTables() {
    return &static_tables_3843;
}

}  // namespace fea
}  // namespace chrono
