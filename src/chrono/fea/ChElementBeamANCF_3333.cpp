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
// Fully Parameterized ANCF beam element with 3 nodes (27DOF). A Description of this element and the Enhanced Continuum
// Mechanics based method can be found in: K. Nachbagauer, P. Gruber, and J. Gerstmayr. Structural and Continuum
// Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element : Application to Static and Linearized
// Dynamic Examples.J.Comput.Nonlinear Dynam, 8 (2) : 021004, 2012.
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

#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementBeamANCF_3333::ChElementBeamANCF_3333()
    : m_method(IntFrcMethod::ContInt),
      m_lenX(0),
      m_thicknessY(0),
      m_thicknessZ(0),
      m_Alpha(0),
      m_damping_enabled(false) {
    m_nodes.resize(3);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementBeamANCF_3333::SetNodes(std::shared_ptr<ChNodeFEAxyzDD> nodeA,
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

// Specify the element dimensions (in the undeformed state - which is different than the reference configuration and it
// is a state the element potentially is never in).

void ChElementBeamANCF_3333::SetDimensions(double lenX, double thicknessY, double thicknessZ) {
    m_lenX = lenX;
    m_thicknessY = thicknessY;
    m_thicknessZ = thicknessZ;

    // Check to see if SetupInitial has already been called (i.e. at least one set of precomputed matrices has been
    // populated).  If so, the precomputed matrices will need to be re-generated.  If not, this will be handled once
    // SetupInitial is called.
    if (m_SD.size() + m_O1.size() > 0) {
        PrecomputeInternalForceMatricesWeights();
    }
}

// Specify the element material.

void ChElementBeamANCF_3333::SetMaterial(std::shared_ptr<ChMaterialBeamANCF> beam_mat) {
    m_material = beam_mat;

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

void ChElementBeamANCF_3333::SetAlphaDamp(double a) {
    m_Alpha = a;
    if (std::abs(m_Alpha) > 1e-10)
        m_damping_enabled = true;
    else
        m_damping_enabled = false;
}

// Change the method used to compute the generalized internal force vector and its Jacobian.

void ChElementBeamANCF_3333::SetIntFrcCalcMethod(IntFrcMethod method) {
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

ChMatrix33<> ChElementBeamANCF_3333::GetGreenLagrangeStrain(const double xi,
                                                    const double eta,
                                                    const double zeta) {
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

ChMatrix33<> ChElementBeamANCF_3333::GetPK2Stress(const double xi,
                                                  const double eta,
                                                  const double zeta) {
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

    ChMatrixNM<double, 6, 6> D;
    GetMaterial()->Get_D(D);

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

double ChElementBeamANCF_3333::GetVonMissesStress(const double xi, const double eta, const double zeta) {
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

    ChMatrixNM<double, 6, 6> D;
    GetMaterial()->Get_D(D);

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

void ChElementBeamANCF_3333::SetupInitial(ChSystem* system) {
    m_element_dof = 0;
    for (int i = 0; i < 3; i++) {
        m_element_dof += m_nodes[i]->GetNdofX();
    }

    m_full_dof = (m_element_dof == 3 * 9);

    if (!m_full_dof) {
        m_mapping_dof.resize(m_element_dof);
        int dof = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < m_nodes[i]->GetNdofX(); j++)
                m_mapping_dof(dof++) = i * 9 + j;
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

void ChElementBeamANCF_3333::GetStateBlock(ChVectorDynamic<>& mD) {
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

// State update.

void ChElementBeamANCF_3333::Update() {
    ChElementGeneric::Update();
}

// Return the mass matrix in full sparse form.

void ChElementBeamANCF_3333::ComputeMmatrixGlobal(ChMatrixRef M) {
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

void ChElementBeamANCF_3333::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0) + m_MassMatrix(3) + m_MassMatrix(6);
    m_nodes[1]->m_TotalMass += m_MassMatrix(3) + m_MassMatrix(24) + m_MassMatrix(27);
    m_nodes[2]->m_TotalMass += m_MassMatrix(6) + m_MassMatrix(27) + m_MassMatrix(39);
}

// Compute the generalized internal force vector for the current nodal coordinates and set the value in the Fi vector.

void ChElementBeamANCF_3333::ComputeInternalForces(ChVectorDynamic<>& Fi) {
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

void ChElementBeamANCF_3333::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
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
void ChElementBeamANCF_3333::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
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
// Interface to ChElementBeam base class (and similar methods)
// -----------------------------------------------------------------------------

void ChElementBeamANCF_3333::EvaluateSectionFrame(const double xi, ChVector<>& point, ChQuaternion<>& rot) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, 0, 0);
    VectorN Sxi_xi_compact;
    Calc_Sxi_xi_compact(Sxi_xi_compact, xi, 0, 0);
    VectorN Sxi_eta_compact;
    Calc_Sxi_eta_compact(Sxi_eta_compact, xi, 0, 0);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;

    // Since ANCF does not use rotations, calculate an approximate
    // rotation based off the position vector gradients
    ChVector<double> BeamAxisTangent = e_bar * Sxi_xi_compact * 2 / m_lenX;
    ChVector<double> CrossSectionY = e_bar * Sxi_eta_compact * 2 / m_thicknessY;

    // Since the position vector gradients are not in general orthogonal,
    // set the Dx direction tangent to the beam axis and
    // compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization, guided by the cross section Y direction
    ChMatrix33<> msect;
    msect.Set_A_Xdir(BeamAxisTangent, CrossSectionY);

    rot = msect.Get_A_quaternion();
}

void ChElementBeamANCF_3333::EvaluateSectionPoint(const double xi, ChVector<>& point) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, 0, 0);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // r = S*e written in compact form
    point = e_bar * Sxi_compact;
}

void ChElementBeamANCF_3333::EvaluateSectionVel(const double xi, ChVector<>& Result) {
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, 0, 0);

    Matrix3xN e_bardot;
    CalcCoordDerivMatrix(e_bardot);

    // rdot = S*edot written in compact form
    Result = e_bardot * Sxi_compact;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).

void ChElementBeamANCF_3333::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
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

void ChElementBeamANCF_3333::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
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

void ChElementBeamANCF_3333::LoadableStateIncrement(const unsigned int off_x,
                                                    ChState& x_new,
                                                    const ChState& x,
                                                    const unsigned int off_v,
                                                    const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 9, x_new, x, off_v + 9, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 18, x_new, x, off_v + 18, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.

void ChElementBeamANCF_3333::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
        mvars.push_back(&m_nodes[i]->Variables_DD());
    }
}

// Evaluate N'*F, which is the projection of the applied point force and moment at the beam axis coordinates (xi,0,0)
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.

void ChElementBeamANCF_3333::ComputeNF(
    const double xi,             // parametric coordinate along the beam axis
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    // Compute the generalized force vector for the applied force component using the compact form of the shape
    // functions.  This requires a reshaping of the calculated matrix to get it into the correct vector order (just a
    // reinterpretation of the data since the matrix is in row-major format)
    VectorN Sxi_compact;
    Calc_Sxi_compact(Sxi_compact, xi, 0, 0);
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
    Calc_Sxi_D(Sxi_D, xi, 0, 0);

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
    //  actual differential length and the normalized differential length.  The vector 2 norm is used to calculate this
    //  length ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.col(0).norm();
}

// Evaluate N'*F, which is the projection of the applied point force and moment at the coordinates (xi,eta,zeta)
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.

void ChElementBeamANCF_3333::ComputeNF(
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

double ChElementBeamANCF_3333::GetDensity() {
    return GetMaterial()->Get_rho();
}

// Calculate tangent to the centerline at coordinate xi [-1 to 1].

ChVector<> ChElementBeamANCF_3333::ComputeTangent(const double xi) {
    VectorN Sxi_xi_compact;
    Calc_Sxi_xi_compact(Sxi_xi_compact, xi, 0, 0);

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // partial derivative of the position vector with respect to xi (normalized coordinate along the beam axis).  In
    // general, this will not be a unit vector
    ChVector<double> BeamAxisTangent = e_bar * Sxi_xi_compact;

    return BeamAxisTangent.GetNormalized();
}

// -----------------------------------------------------------------------------
// Mass Matrix & Generalized Force Due to Gravity Calculation
// -----------------------------------------------------------------------------

void ChElementBeamANCF_3333::ComputeMassMatrixAndGravityForce() {
    // For this element, the mass matrix integrand is of order 9 in xi, 3 in eta, and 3 in zeta.
    // 4 GQ Points are needed in the xi direction and 2 GQ Points are needed in the eta and zeta directions for
    // exact integration of the element's mass matrix, even if the reference configuration is not straight. Since the
    // major pieces of the generalized force due to gravity can also be used to calculate the mass matrix, these
    // calculations are performed at the same time.  Only the matrix that scales the acceleration due to gravity is
    // calculated at this time so that any changes to the acceleration due to gravity in the system are correctly
    // accounted for in the generalized internal force calculation.

    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi = 4;        // 5 Point Gauss-Quadrature;
    unsigned int GQ_idx_eta_zeta = 1;  // 2 Point Gauss-Quadrature;

    // Mass Matrix in its compact matrix form.  Since the mass matrix is symmetric, just the upper diagonal entries will
    // be stored.
    ChMatrixNM<double, NSF, NSF> MassMatrixCompactSquare;

    // Set these to zeros since they will be incremented as the vector/matrix is calculated
    MassMatrixCompactSquare.setZero();
    m_GravForceScale.setZero();

    double rho = GetMaterial()->Get_rho();  // Density of the material for the element

    // Sum the contribution to the mass matrix and generalized force due to gravity at the current point
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi][it_xi] * GQTable->Weight[GQ_idx_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi][it_xi];
                double eta = GQTable->Lroots[GQ_idx_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_eta_zeta][it_zeta];
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

void ChElementBeamANCF_3333::PrecomputeInternalForceMatricesWeights() {
    if (m_method == IntFrcMethod::ContInt)
        PrecomputeInternalForceMatricesWeightsContInt();
    else
        PrecomputeInternalForceMatricesWeightsPreInt();
}

// Precalculate constant matrices for the internal force calculations when using the "Continuous Integration" style
// method

void ChElementBeamANCF_3333::PrecomputeInternalForceMatricesWeightsContInt() {
    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi = NP - 1;        // Gauss-Quadrature table index for xi
    unsigned int GQ_idx_eta_zeta = NT - 1;  // Gauss-Quadrature table index for eta and zeta

    m_SD.resize(NSF, 3 * NIP);
    m_kGQ_D0.resize(NIP_D0, 1);
    m_kGQ_Dv.resize(NIP_Dv, 1);

    ChMatrixNM<double, NSF, 3> SD_precompute_D;

    // Precalculate the matrices of normalized shape function derivatives corrected for a potentially non-straight
    // reference configuration & GQ Weights times the determinant of the element Jacobian for later use in the
    // generalized internal force and Jacobian calculations.

    // First calculate the matrices and constants for the portion of the Enhance Continuum Mechanics/Selective Reduced
    // Integration that does not include the Poisson effect.
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi][it_xi] * GQTable->Weight[GQ_idx_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi][it_xi];
                double eta = GQTable->Lroots[GQ_idx_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_eta_zeta][it_zeta];
                auto index = it_zeta + it_eta * GQTable->Lroots[GQ_idx_eta_zeta].size() +
                             it_xi * GQTable->Lroots[GQ_idx_eta_zeta].size() * GQTable->Lroots[GQ_idx_eta_zeta].size();
                ChMatrix33<double>
                    J_0xi;         // Element Jacobian between the reference configuration and normalized configuration
                MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives

                Calc_Sxi_D(Sxi_D, xi, eta, zeta);
                J_0xi.noalias() = m_ebar0 * Sxi_D;

                // Adjust the shape function derivative matrix to account for a potentially deformed reference state
                SD_precompute_D = Sxi_D * J_0xi.inverse();
                m_kGQ_D0(index) = -J_0xi.determinant() * GQ_weight;

                // Group all of the columns together in blocks with the shape function derivatives for the section that
                // does not include the Poisson effect at the beginning of m_SD
                m_SD.col(index) = SD_precompute_D.col(0);
                m_SD.col(index + NIP_D0) = SD_precompute_D.col(1);
                m_SD.col(index + 2 * NIP_D0) = SD_precompute_D.col(2);

                index++;
            }
        }
    }

    // Next calculate the matrices and constants for the portion of the Enhance Continuum Mechanics/Selective Reduced
    // Integration that includes the Poisson effect, but integrating across the volume of the element with one 1 point
    // GQ for the cross section directions.
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi].size(); it_xi++) {
        double GQ_weight = GQTable->Weight[GQ_idx_xi][it_xi] * 2 * 2;
        double xi = GQTable->Lroots[GQ_idx_xi][it_xi];
        double eta = 0;
        double zeta = 0;
        ChMatrix33<double> J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
        MatrixNx3c Sxi_D;          // Matrix of normalized shape function derivatives

        Calc_Sxi_D(Sxi_D, xi, eta, zeta);
        J_0xi.noalias() = m_ebar0 * Sxi_D;

        // Adjust the shape function derivative matrix to account for a potentially deformed reference state
        SD_precompute_D = Sxi_D * J_0xi.inverse();
        m_kGQ_Dv(it_xi) = -J_0xi.determinant() * GQ_weight;

        // Group all of the columns together in blocks with the shape function derivative for the Poisson effect at the
        // end of m_SD
        m_SD.col(3 * NIP_D0 + it_xi) = SD_precompute_D.col(0);
        m_SD.col(3 * NIP_D0 + NIP_Dv + it_xi) = SD_precompute_D.col(1);
        m_SD.col(3 * NIP_D0 + 2 * NIP_Dv + it_xi) = SD_precompute_D.col(2);
    }
}

// Precalculate constant matrices needed for the internal force calculations when using the "Pre-Integration" style
// method

void ChElementBeamANCF_3333::PrecomputeInternalForceMatricesWeightsPreInt() {
    ChQuadratureTables* GQTable = GetStaticGQTables();
    unsigned int GQ_idx_xi = NP - 1;        // Gauss-Quadrature table index for xi
    unsigned int GQ_idx_eta_zeta = NT - 1;  // Gauss-Quadrature table index for eta and zeta

    m_O1.resize(NSF * NSF, NSF * NSF);
    m_O2.resize(NSF * NSF, NSF * NSF);
    m_K3Compact.resize(NSF, NSF);
    m_K13Compact.resize(NSF, NSF);

    m_K13Compact.setZero();
    m_K3Compact.setZero();
    m_O1.setZero();

    // =============================================================================
    // =============================================================================
    // Calculate the contribution to constant matrices needed to account for the integration across the volume of the
    // beam element, excluding the Poisson effect for the Enhanced Continuum Mechanics method
    // =============================================================================
    // =============================================================================

    // Get the components of the stiffness tensor in 6x6 matrix form that exclude the Poisson effect
    const ChVectorN<double, 6>& D0 = GetMaterial()->Get_D0();
    ChMatrixNM<double, 6, 6> D;
    D.setZero();
    D.diagonal() = D0;

    // Setup the stiffness tensor in block matrix form to make it easier to iterate through all 4 subscripts
    ChMatrixNM<double, 9, 9> D_block;
    D_block << D(0, 0), D(0, 5), D(0, 4), D(0, 5), D(0, 1), D(0, 3), D(0, 4), D(0, 3), D(0, 2), D(5, 0), D(5, 5),
        D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3), D(5, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(5, 0), D(5, 5), D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3),
        D(5, 2), D(1, 0), D(1, 5), D(1, 4), D(1, 5), D(1, 1), D(1, 3), D(1, 4), D(1, 3), D(1, 2), D(3, 0), D(3, 5),
        D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3), D(3, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(3, 0), D(3, 5), D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3),
        D(3, 2), D(2, 0), D(2, 5), D(2, 4), D(2, 5), D(2, 1), D(2, 3), D(2, 4), D(2, 3), D(2, 2);

    // Pre-calculate the matrices of normalized shape function derivatives corrected for a potentially non-straight
    // reference configuration & GQ Weights times the determinate of the element Jacobian for later Calculating the
    // portion of the Selective Reduced Integration that does account for the Poisson effect
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi].size(); it_xi++) {
        for (unsigned int it_eta = 0; it_eta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_eta++) {
            for (unsigned int it_zeta = 0; it_zeta < GQTable->Lroots[GQ_idx_eta_zeta].size(); it_zeta++) {
                double GQ_weight = GQTable->Weight[GQ_idx_xi][it_xi] * GQTable->Weight[GQ_idx_eta_zeta][it_eta] *
                                   GQTable->Weight[GQ_idx_eta_zeta][it_zeta];
                double xi = GQTable->Lroots[GQ_idx_xi][it_xi];
                double eta = GQTable->Lroots[GQ_idx_eta_zeta][it_eta];
                double zeta = GQTable->Lroots[GQ_idx_eta_zeta][it_zeta];

                ChMatrix33<double>
                    J_0xi;         // Element Jacobian between the reference configuration and normalized configuration
                MatrixNx3c Sxi_D;  // Matrix of normalized shape function derivatives

                Calc_Sxi_D(Sxi_D, xi, eta, zeta);
                J_0xi.noalias() = m_ebar0 * Sxi_D;

                MatrixNx3c Sxi_D_0xi = Sxi_D * J_0xi.inverse();
                double GQWeight_det_J_0xi = -J_0xi.determinant() * GQ_weight;

                // Shortcut for:
                // m_K3Compact += GQWeight_det_J_0xi * 0.5 * (Sxi_D_0xi*D11*Sxi_D_0xi.transpose() + Sxi_D_0xi *
                // D22*Sxi_D_0xi.transpose() + Sxi_D_0xi * D33*Sxi_D_0xi.transpose());
                m_K3Compact += GQWeight_det_J_0xi * 0.5 *
                               (D0(0) * Sxi_D_0xi.template block<NSF, 1>(0, 0) *
                                    Sxi_D_0xi.template block<NSF, 1>(0, 0).transpose() +
                                D0(1) * Sxi_D_0xi.template block<NSF, 1>(0, 1) *
                                    Sxi_D_0xi.template block<NSF, 1>(0, 1).transpose() +
                                D0(2) * Sxi_D_0xi.template block<NSF, 1>(0, 2) *
                                    Sxi_D_0xi.template block<NSF, 1>(0, 2).transpose());

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

    // =============================================================================
    // =============================================================================
    // Calculate the contribution to constant matrices needed to account for the integration across the volume of the
    // beam element, including the Poisson effect only along the beam axis (selective reduced integration) with 1 point
    // Gauss quadrature for the cross section directions
    // =============================================================================
    // =============================================================================

    // Get the components of the stiffness tensor in 6x6 matrix form that include the Poisson effect

    D.setZero();
    D.block(0, 0, 3, 3) = GetMaterial()->Get_Dv();

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
    D_block << D(0, 0), D(0, 5), D(0, 4), D(0, 5), D(0, 1), D(0, 3), D(0, 4), D(0, 3), D(0, 2), D(5, 0), D(5, 5),
        D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3), D(5, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(5, 0), D(5, 5), D(5, 4), D(5, 5), D(5, 1), D(5, 3), D(5, 4), D(5, 3),
        D(5, 2), D(1, 0), D(1, 5), D(1, 4), D(1, 5), D(1, 1), D(1, 3), D(1, 4), D(1, 3), D(1, 2), D(3, 0), D(3, 5),
        D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3), D(3, 2), D(4, 0), D(4, 5), D(4, 4), D(4, 5), D(4, 1),
        D(4, 3), D(4, 4), D(4, 3), D(4, 2), D(3, 0), D(3, 5), D(3, 4), D(3, 5), D(3, 1), D(3, 3), D(3, 4), D(3, 3),
        D(3, 2), D(2, 0), D(2, 5), D(2, 4), D(2, 5), D(2, 1), D(2, 3), D(2, 4), D(2, 3), D(2, 2);

    // Loop over each Gauss quadrature point and sum the contribution to each constant matrix that will be used for
    // the internal force calculations
    for (unsigned int it_xi = 0; it_xi < GQTable->Lroots[GQ_idx_xi].size(); it_xi++) {
        double GQ_weight = GQTable->Weight[GQ_idx_xi][it_xi] * 2 * 2;
        double xi = GQTable->Lroots[GQ_idx_xi][it_xi];
        double eta = 0;
        double zeta = 0;

        ChMatrix33<double> J_0xi;  // Element Jacobian between the reference configuration and normalized configuration
        MatrixNx3c Sxi_D;          // Matrix of normalized shape function derivatives

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
                        m_O1.block<NSF, NSF>(NSF * t, NSF * f) += scale(t, f) * Sxi_D_0xi_n_Sxi_D_0xi_c_transpose;
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

void ChElementBeamANCF_3333::ComputeInternalForcesContIntDamping(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Continuous Integration" style of method assuming a
    // linear viscoelastic material model (single term damping model).  For this style of method, the generalized
    // internal force vector is integrated across the volume of the element every time this calculation is performed.
    // For this element, this is likely more efficient than the "Pre-Integration" style calculation method.  Note that
    // the integrand for the generalize internal force vector for a straight and normalized element is of order : 8 in
    // xi, 4 in eta, and 4 in zeta. This requires GQ 5 points along the xi direction and 3 GQ points along the eta and
    // zeta directions for "Full Integration". However, very similar results can be obtained with fewer GQ point in each
    // direction, resulting in significantly fewer calculations.  Based on testing, this could be as low as 3x2x2

    MatrixNx6 ebar_ebardot;
    CalcCombinedCoordMatrix(ebar_ebardot);

    // =============================================================================
    // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
    // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
    // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31  F11dot  F21dot  F31dot ] <-- D0 Block (No Poisson Effect)
    //      [F12  F22  F32  F12dot  F22dot  F32dot ] <-- D0 Block (No Poisson Effect)
    // FC = [F13  F23  F33  F13dot  F23dot  F33dot ] <-- D0 Block (No Poisson Effect)
    //      [F11  F21  F31  F11dot  F21dot  F31dot ] <-- Dv Block (With Poisson Effect)
    //      [F12  F22  F32  F12dot  F22dot  F32dot ] <-- Dv Block (With Poisson Effect)
    //      [F13  F23  F33  F13dot  F23dot  F33dot ] <-- Dv Block (With Poisson Effect)
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 6> FC = m_SD.transpose() * ebar_ebardot;

    // =============================================================================
    // =============================================================================
    // Since the Enhanced Continuum Mechanics method is utilized for this element, first calculate the contribution to
    // the generalized internal force vector from the terms that do not include the Poisson effect.  This is integrated
    // across the entire volume of the element using the full number of Gauss quadrature points.
    // =============================================================================
    // =============================================================================

    // =============================================================================
    // Get the diagonal terms of the 6x6 matrix (does not include the Poisson effect)
    // =============================================================================
    const ChVectorN<double, 6>& D0 = GetMaterial()->Get_D0();

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // Note that the scaled time derivatives of the Green-Lagrange strain are added to make the calculation of
    // the 2nd Piola-Kirchoff stresses more efficient.  The combined result is then scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point (m_kGQ) again for efficiency.
    // Since only the diagonal terms in the 6x6 stiffness matrix are used, the 2nd Piola-Kirchoff can be calculated by
    // simply scaling the vector of scaled Green-Lagrange strains in Voight notation by the corresponding diagonal entry
    // in the stiffness matrix.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    // Each entry in SPK2_1 = D11*kGQ*(E11+alpha*E11dot)
    //     = D11*kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP_D0 E_BlockDamping_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5));
    VectorNIP_D0 SPK2_1_Block_D0 = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 0)) +
                                   FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1)) +
                                   FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2));
    SPK2_1_Block_D0.array() -= 1;
    SPK2_1_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_1_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_1_Block_D0 *= (0.5 * D0(0));

    // Each entry in SPK2_2 = D22*kGQ*(E22+alpha*E22dot)
    //     = D22*kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5));
    VectorNIP_D0 SPK2_2_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_2_Block_D0.array() -= 1;
    SPK2_2_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_2_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_2_Block_D0 *= (0.5 * D0(1));

    // Each entry in SPK2_3 = D33*kGQ*(E33+alpha*E33dot)
    //     = D33*kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_3_Block_D0 =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_3_Block_D0.array() -= 1;
    SPK2_3_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_3_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_3_Block_D0 *= (0.5 * D0(2));

    // Each entry in SPK2_4 = D44*kGQ*(2*(E23+alpha*E23dot))
    //     = D44*kGQ*((F12*F13+F22*F23+F32*F33)
    //       +alpha*(F12dot*F13+F22dot*F23+F32dot*F33 + F12*F13dot+F22*F23dot+F32*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_4_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_4_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_4_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_4_Block_D0 *= D0(3);

    // Each entry in SPK2_5 = D55*kGQ*(2*(E13+alpha*E13dot))
    //     = D55*kGQ*((F11*F13+F21*F23+F31*F33)
    //       +alpha*(F11dot*F13+F21dot*F23+F31dot*F33 + F11*F13dot+F21*F23dot+F31*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5)) +
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_5_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_5_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_5_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_5_Block_D0 *= D0(4);

    // Each entry in SPK2_6 = D66*kGQ*(2*(E12+alpha*E12dot))
    //     = D66*kGQ*((F11*F12+F21*F22+F31*F32)
    //       +alpha*(F11dot*F12+F21dot*F22+F31dot*F32 + F11*F12dot+F21*F22dot+F31*F32dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5)) +
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5));
    VectorNIP_D0 SPK2_6_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_6_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_6_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_6_Block_D0 *= D0(5);

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- D0 Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- D0 Block
    //         = [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- D0 Block
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- Dv Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- Dv Block
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- Dv Block
    // Note that the Dv Block entries will be calculated separately in a later step.
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> P_Block;

    P_Block.template block<NIP_D0, 1>(0, 0) = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_5_Block_D0);
    P_Block.template block<NIP_D0, 1>(0, 1) = FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_5_Block_D0);
    P_Block.template block<NIP_D0, 1>(0, 2) = FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_5_Block_D0);

    P_Block.template block<NIP_D0, 1>(NIP_D0, 0) =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_4_Block_D0);
    P_Block.template block<NIP_D0, 1>(NIP_D0, 1) =
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_4_Block_D0);
    P_Block.template block<NIP_D0, 1>(NIP_D0, 2) =
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_4_Block_D0);

    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 0) =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_3_Block_D0);
    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 1) =
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_3_Block_D0);
    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 2) =
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_3_Block_D0);

    // =============================================================================
    // =============================================================================
    // Since the Enhanced Continuum Mechanics method is utilized for this element, second calculate the contribution to
    // the generalized internal force vector from the terms that include the Poisson effect.  This is integrated
    // across the entire volume of the element using Gauss quadrature points only along the beam axis.  It is assumed
    // that an isotropic material or orthotropic material aligned with its principle axes is used
    // =============================================================================
    // =============================================================================

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // Note that the scaled time derivatives of the Green-Lagrange strain are added to make the later calculation of
    // the 2nd Piola-Kirchoff stresses more efficient.  The combined result is then scaled by minus the Gauss
    // quadrature weight times the element Jacobian at the corresponding Gauss point (m_kGQ) again for efficiency.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // Note that with the material assumption being used, only [E11,E22,E33] need to be calculated
    // =============================================================================

    // Each entry in E1 = kGQ*(E11+alpha*E11dot)
    //                  = kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP_Dv E_BlockDamping_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 3)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 4)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 5));
    VectorNIP_Dv E1_Block_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2));
    E1_Block_Dv.array() -= 1;
    E1_Block_Dv *= 0.5;
    E1_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E1_Block_Dv.array() *= m_kGQ_Dv.array();

    // Each entry in E2 = kGQ*(E22+alpha*E22dot)
    //                  = kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping_Dv.noalias() = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 3)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 4)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 5));
    VectorNIP_Dv E2_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2));
    E2_Block_Dv.array() -= 1;
    E2_Block_Dv *= 0.5;
    E2_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E2_Block_Dv.array() *= m_kGQ_Dv.array();

    // Each entry in E3 = kGQ*(E33+alpha*E33dot)
    //                  = kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping_Dv.noalias() = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 3)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 4)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 5));
    VectorNIP_Dv E3_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2));
    E3_Block_Dv.array() -= 1;
    E3_Block_Dv *= 0.5;
    E3_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E3_Block_Dv.array() *= m_kGQ_Dv.array();

    // =============================================================================
    // Get the upper 3x3 block of the stiffness tensor in 6x6 matrix that accounts for the Poisson effect.  Note that
    // with the split of the stiffness matrix and the assumed materials, the entries in the 6x6 stiffness matrix outside
    // of the upper 3x3 block are all zeros.
    // =============================================================================

    const ChMatrix33<double>& Dv = GetMaterial()->Get_Dv();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have been scaled and already been
    // combined with their scaled time derivatives and minus the Gauss quadrature weight times the element Jacobian at
    // the corresponding Gauss point to make the calculation of the 2nd Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP_Dv SPK2_1_Block_Dv = Dv(0, 0) * E1_Block_Dv + Dv(0, 1) * E2_Block_Dv + Dv(0, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_2_Block_Dv = Dv(1, 0) * E1_Block_Dv + Dv(1, 1) * E2_Block_Dv + Dv(1, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_3_Block_Dv = Dv(2, 0) * E1_Block_Dv + Dv(2, 1) * E2_Block_Dv + Dv(2, 2) * E3_Block_Dv;

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- D0 Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- D0 Block
    //         = [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- D0 Block
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- Dv Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- Dv Block
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- Dv Block
    // Note that the D0 Block entries have already been calculated above
    // =============================================================================

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(SPK2_1_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(SPK2_1_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(SPK2_1_Block_Dv);

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0).cwiseProduct(SPK2_2_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).cwiseProduct(SPK2_2_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).cwiseProduct(SPK2_2_Block_Dv);

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).cwiseProduct(SPK2_3_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).cwiseProduct(SPK2_3_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).cwiseProduct(SPK2_3_Block_Dv);

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

void ChElementBeamANCF_3333::ComputeInternalForcesContIntNoDamping(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Continuous Integration" style of method assuming a
    // linear material model (no damping).  For this style of method, the generalized internal force vector is
    // integrated across the volume of the element every time this calculation is performed. For this element, this is
    // likely more efficient than the "Pre-Integration" style calculation method.  Note that the integrand for the
    // generalize internal force vector for a straight and normalized element is of order : 8 in xi, 4 in eta, and 4 in
    // zeta. This requires GQ 5 points along the xi direction and 3 GQ points along the eta and zeta directions for
    // "Full Integration". However, very similar results can be obtained with fewer GQ point in each direction,
    // resulting in significantly fewer calculations.  Based on testing, this could be as low as 3x2x2

    Matrix3xN e_bar;
    CalcCoordMatrix(e_bar);

    // =============================================================================
    // Calculate the deformation gradient for all Gauss quadrature points in a single matrix multiplication.  Note
    // that since the shape function derivative matrix is ordered by columns, the resulting deformation gradient
    // will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31 ] <-- D0 Block (No Poisson Effect)
    //      [F12  F22  F32 ] <-- D0 Block (No Poisson Effect)
    // FC = [F13  F23  F33 ] <-- D0 Block (No Poisson Effect)
    //      [F11  F21  F31 ] <-- Dv Block (With Poisson Effect)
    //      [F12  F22  F32 ] <-- Dv Block (With Poisson Effect)
    //      [F13  F23  F33 ] <-- Dv Block (With Poisson Effect)
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> FC = m_SD.transpose() * e_bar.transpose();

    // =============================================================================
    // =============================================================================
    // Since the Enhanced Continuum Mechanics method is utilized for this element, first calculate the contribution to
    // the generalized internal force vector from the terms that do not include the Poisson effect.  This is integrated
    // across the entire volume of the element using the full number of Gauss quadrature points.
    // =============================================================================
    // =============================================================================

    // =============================================================================
    // Get the diagonal terms of the 6x6 matrix (does not include the Poisson effect)
    // =============================================================================
    const ChVectorN<double, 6>& D0 = GetMaterial()->Get_D0();

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // The result is then scaled by minus the Gauss quadrature weight times the element Jacobian at the
    // corresponding Gauss point (m_kGQ) for efficiency. Since only the diagonal terms in the 6x6 stiffness matrix are
    // used, the 2nd Piola-Kirchoff stress can be calculated by simply scaling the vector of scaled Green-Lagrange
    // strains in Voight notation by the corresponding diagonal entry in the stiffness matrix.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    // Each entry in SPK2_1 = D11*kGQ*E11 = D11*kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP_D0 SPK2_1_Block_D0 = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 0)) +
                                   FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1)) +
                                   FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2));
    SPK2_1_Block_D0.array() -= 1;
    SPK2_1_Block_D0.array() *= (0.5 * D0(0)) * m_kGQ_D0.array();

    // Each entry in SPK2_2 = D22*kGQ*E22 = D22*kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP_D0 SPK2_2_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_2_Block_D0.array() -= 1;
    SPK2_2_Block_D0.array() *= (0.5 * D0(1)) * m_kGQ_D0.array();

    // Each entry in SPK2_3 = D33*kGQ_D0*E33 = D33*kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP_D0 SPK2_3_Block_D0 =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_3_Block_D0.array() -= 1;
    SPK2_3_Block_D0.array() *= (0.5 * D0(2)) * m_kGQ_D0.array();

    // Each entry in SPK2_4 = D44*kGQ*2*E23 = D44*kGQ*(F12*F13+F22*F23+F32*F33)
    VectorNIP_D0 SPK2_4_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_4_Block_D0.array() *= D0(3) * m_kGQ_D0.array();

    // Each entry in SPK2_5 = D55*kGQ*2*E13 = D55*kGQ*(F11*F13+F21*F23+F31*F33)
    VectorNIP_D0 SPK2_5_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_5_Block_D0.array() *= D0(4) * m_kGQ_D0.array();

    // Each entry in SPK2_6 = D66*kGQ*2*E12 = D66*(F11*F12+F21*F22+F31*F32)
    VectorNIP_D0 SPK2_6_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_6_Block_D0.array() *= D0(5) * m_kGQ_D0.array();

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- D0 Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- D0 Block
    //         = [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- D0 Block
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- Dv Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- Dv Block
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- Dv Block
    // Note that the Dv Blocks entries are calculated in a later step
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> P_Block;

    P_Block.template block<NIP_D0, 1>(0, 0) = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_5_Block_D0);
    P_Block.template block<NIP_D0, 1>(0, 1) = FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_5_Block_D0);
    P_Block.template block<NIP_D0, 1>(0, 2) = FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_1_Block_D0) +
                                              FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_6_Block_D0) +
                                              FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_5_Block_D0);

    P_Block.template block<NIP_D0, 1>(NIP_D0, 0) =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_4_Block_D0);
    P_Block.template block<NIP_D0, 1>(NIP_D0, 1) =
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_4_Block_D0);
    P_Block.template block<NIP_D0, 1>(NIP_D0, 2) =
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_6_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_2_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_4_Block_D0);

    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 0) =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(SPK2_3_Block_D0);
    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 1) =
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(SPK2_3_Block_D0);
    P_Block.template block<NIP_D0, 1>(2 * NIP_D0, 2) =
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(SPK2_5_Block_D0) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(SPK2_4_Block_D0) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(SPK2_3_Block_D0);

    // =============================================================================
    // =============================================================================
    // Since the Enhanced Continuum Mechanics method is utilized for this element, second calculate the contribution to
    // the generalized internal force vector from the terms that include the Poisson effect.  This is integrated
    // across the entire volume of the element using Gauss quadrature points only along the beam axis.  It is assumed
    // that an isotropic material or orthotropic material aligned with its principle axes is used
    // =============================================================================
    // =============================================================================

    // =============================================================================
    // Calculate each individual value of the Green-Lagrange strain component by component across all the
    // Gauss-Quadrature points at a time to better leverage vectorized CPU instructions.
    // The combined result is then scaled by minus the Gauss quadrature weight times the element Jacobian at the
    // corresponding Gauss point (m_kGQ) again for efficiency. Results are written in Voigt notation: epsilon =
    // [E11,E22,E33,2*E23,2*E13,2*E12] Note that with the material assumption being used, only [E11,E22,E33] need to be
    // calculated
    // =============================================================================

    // Each entry in E1 = kGQ*E11 = kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP_Dv E1_Block_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2));
    E1_Block_Dv.array() -= 1;
    E1_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // Each entry in E2 = kGQ*E22 = kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP_Dv E2_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2));
    E2_Block_Dv.array() -= 1;
    E2_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // Each entry in E3 = kGQ*E33 = kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP_Dv E3_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2));
    E3_Block_Dv.array() -= 1;
    E3_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // =============================================================================
    // Get the upper 3x3 block of the stiffness tensor in 6x6 matrix that accounts for the Poisson effect.  Note that
    // with the split of the stiffness matrix and the assumed materials, the entries in the 6x6 stiffness matrix outside
    // of the upper 3x3 block are all zeros.
    // =============================================================================

    const ChMatrix33<double>& Dv = GetMaterial()->Get_Dv();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Gauss quadrature points at a time
    // component by component. Note that the Green-Largrange strain components have already been scaled
    // by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point to make the
    // calculation of the 2nd Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    VectorNIP_Dv SPK2_1_Block_Dv = Dv(0, 0) * E1_Block_Dv + Dv(0, 1) * E2_Block_Dv + Dv(0, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_2_Block_Dv = Dv(1, 0) * E1_Block_Dv + Dv(1, 1) * E2_Block_Dv + Dv(1, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_3_Block_Dv = Dv(2, 0) * E1_Block_Dv + Dv(2, 1) * E2_Block_Dv + Dv(2, 2) * E3_Block_Dv;

    // =============================================================================
    // Calculate the transpose of the 1st Piola-Kirchoff stresses in block tensor form whose entries have been
    // scaled by minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss point.
    // The entries are grouped by component in block matrices (column vectors)
    // P_Block = kGQ*P_transpose = kGQ*SPK2*F_transpose
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- D0 Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- D0 Block
    //         = [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- D0 Block
    //           [kGQ*(P_transpose)_11  kGQ*(P_transpose)_12  kGQ*(P_transpose)_13 ] <-- Dv Block
    //           [kGQ*(P_transpose)_21  kGQ*(P_transpose)_22  kGQ*(P_transpose)_23 ] <-- Dv Block
    //           [kGQ*(P_transpose)_31  kGQ*(P_transpose)_32  kGQ*(P_transpose)_33 ] <-- Dv Block
    // Note that the D0 entries have already been calculated above
    // =============================================================================

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(SPK2_1_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(SPK2_1_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(SPK2_1_Block_Dv);

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0).cwiseProduct(SPK2_2_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).cwiseProduct(SPK2_2_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).cwiseProduct(SPK2_2_Block_Dv);

    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).cwiseProduct(SPK2_3_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).cwiseProduct(SPK2_3_Block_Dv);
    P_Block.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2) =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).cwiseProduct(SPK2_3_Block_Dv);

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

void ChElementBeamANCF_3333::ComputeInternalForcesContIntPreInt(ChVectorDynamic<>& Fi) {
    // Calculate the generalize internal force vector using the "Pre-Integration" style of method assuming a
    // linear viscoelastic material model (single term damping model).  For this style of method, the components of the
    // generalized internal force vector and its Jacobian that need to be integrated across the volume are calculated
    // once prior to the start of the simulation.

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

void ChElementBeamANCF_3333::ComputeInternalJacobianContIntDamping(ChMatrixRef& H,
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
    // calculation of the potentially non-symmetric and non-sparse component.  The second piece is the symmetric
    // and sparse component.

    // =============================================================================
    // Calculate the deformation gradient and time derivative of the deformation gradient for all Gauss quadrature
    // points in a single matrix multiplication.  Note that since the shape function derivative matrix is ordered by
    // columns, the resulting deformation gradient will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31  F11dot  F21dot  F31dot ] <-- D0 Block (No Poisson Effect)
    //      [F12  F22  F32  F12dot  F22dot  F32dot ] <-- D0 Block (No Poisson Effect)
    // FC = [F13  F23  F33  F13dot  F23dot  F33dot ] <-- D0 Block (No Poisson Effect)
    //      [F11  F21  F31  F11dot  F21dot  F31dot ] <-- Dv Block (With Poisson Effect)
    //      [F12  F22  F32  F12dot  F22dot  F32dot ] <-- Dv Block (With Poisson Effect)
    //      [F13  F23  F33  F13dot  F23dot  F33dot ] <-- Dv Block (With Poisson Effect)
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
    // PE = [(d epsilon1/d e)GQPnt1' (d epsilon1/d e)GQPnt2' ... (d epsilon1/d e)GQPntNIP_D0'...
    //       (d epsilon2/d e)GQPnt1' (d epsilon2/d e)GQPnt2' ... (d epsilon2/d e)GQPntNIP_D0'...
    //       (d epsilon3/d e)GQPnt1' (d epsilon3/d e)GQPnt2' ... (d epsilon3/d e)GQPntNIP_D0'...
    //       (d epsilon4/d e)GQPnt1' (d epsilon4/d e)GQPnt2' ... (d epsilon4/d e)GQPntNIP_D0'...
    //       (d epsilon5/d e)GQPnt1' (d epsilon5/d e)GQPnt2' ... (d epsilon5/d e)GQPntNIP_D0'...
    //       (d epsilon6/d e)GQPnt1' (d epsilon6/d e)GQPnt2' ... (d epsilon6/d e)GQPntNIP_D0'...
    //       (d epsilon1/d e)GQPntDv1' ... (d epsilon1/d e)GQPntNIP_Dv'...
    //       (d epsilon2/d e)GQPntDv1' ... (d epsilon2/d e)GQPntNIP_Dv'...
    //       (d epsilon3/d e)GQPntDv1' ... (d epsilon3/d e)GQPntNIP_Dv']
    // Note that each partial derivative block shown is placed to the left of the previous block.
    // The explanation of the calculation above is just too long to write it all on a single line.
    // Since there are no shear terms with the material assumption for Dv, there is no need to calculate
    // (d epsilon4/d e), (d epsilon5/d e), (d epsilon6/d e) for the Dv Gauss points since these partial
    // derivatives will be multiplied by zero and will not contribute anything to the Jacobian matrix.
    // The calculation is performed on 3 rows at a time to capture the partial with respect to the vector of nodal
    // coordinates e while using the compact matrix form ebar for the calculations.
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 6 * NIP_D0 + 3 * NIP_Dv> PE;

    for (auto i = 0; i < NSF; i++) {
        PE.template block<1, NIP_D0>(3 * i, 0 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 1 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose());

        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 0 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0 * NIP_Dv, 0).transpose());
        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 1 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 1 * NIP_Dv, 0).transpose());
        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).transpose());

        PE.template block<1, NIP_D0>((3 * i) + 1, 0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose());

        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).transpose());

        PE.template block<1, NIP_D0>((3 * i) + 2, 0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose());

        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).transpose());
    }

    // =============================================================================
    // Combine the deformation gradient, time derivative of the deformation gradient, damping coefficient, Gauss
    // quadrature weighting times the element Jacobian, and Jacobian component scale factors into a scaled block
    // deformation gradient matrix Calculate the deformation gradient and time derivative of the deformation
    // gradient for all Gauss quadrature points in a single matrix multiplication.  Note that the resulting combined
    // deformation gradient block matrix will be ordered by block matrix (column vectors) components Note that the
    // indices of the components are in transposed order
    //            [kGQ*(Kfactor+alpha*Rfactor)*F11+alpha*Rfactor*F11dot ... similar for F21 & F31 blocks] <- D0
    // FCscaled = [kGQ*(Kfactor+alpha*Rfactor)*F12+alpha*Rfactor*F12dot ... similar for F22 & F32 blocks] <- D0
    //            [kGQ*(Kfactor+alpha*Rfactor)*F13+alpha*Rfactor*F13dot ... similar for F23 & F33 blocks] <- D0
    //            [kGQ*(Kfactor+alpha*Rfactor)*F11+alpha*Rfactor*F11dot ... similar for F21 & F31 blocks] <- Dv
    //            [kGQ*(Kfactor+alpha*Rfactor)*F12+alpha*Rfactor*F12dot ... similar for F22 & F32 blocks] <- Dv
    //            [kGQ*(Kfactor+alpha*Rfactor)*F13+alpha*Rfactor*F13dot ... similar for F23 & F33 blocks] <- Dv
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> FCscaled = (Kfactor + m_Alpha * Rfactor) * FC.template block<3 * NIP, 3>(0, 0) +
                                               (m_Alpha * Kfactor) * FC.template block<3 * NIP, 3>(0, 3);

    for (auto i = 0; i < 3; i++) {
        FCscaled.template block<NIP_D0, 1>(0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_D0, 1>(NIP_D0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, i).array() *= m_kGQ_Dv.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, i).array() *= m_kGQ_Dv.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, i).array() *= m_kGQ_Dv.array();
    }

    // =============================================================================
    // Get the diagonal terms of the 6x6 matrix (do not include the Poisson effect
    // =============================================================================
    const ChVectorN<double, 6>& D0 = GetMaterial()->Get_D0();

    // =============================================================================
    // Calculate the combination of the scaled partial derivative of the Green-Largrange strains with respect to the
    // nodal coordinates, the scaled partial derivative of the time derivative of the Green-Largrange strains with
    // respect to the nodal coordinates, the scaled partial derivative of the Green-Largrange strains with respect
    // to the time derivative of the nodal coordinates, and the other parameters to correctly integrate across the
    // volume of the element.  This calculation is performed in blocks across all the Gauss quadrature points at the
    // same time.  This value should be store in row major memory layout to align with the access patterns for
    // calculating this matrix.
    // =============================================================================
    // For the D0 Gauss quadrature points, the stiffness matrix D0 is diagonal so the scaled partial derivatives can be
    // easily multiplied by D0.  The Dv Gauss quadrature points are handled separately after this
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 6 * NIP_D0 + 3 * NIP_Dv> DScaled_Combined_PE;

    for (auto i = 0; i < NSF; i++) {
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose()));

        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose()));

        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose()));
    }

    // =============================================================================
    // For the Dv Gauss quadrature points, the matrix of scaled partial derivatives needs to be calculated first since
    // they will be will be combined together when multiplying by the Dv stiffness matrix (upper 3x3 block is
    // potentially non-zero)
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 3 * NIP_Dv> Scaled_Combined_PE_Dv;

    for (auto i = 0; i < NSF; i++) {
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).transpose());

        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).transpose());

        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).transpose());
    }

    // =============================================================================
    // Get the upper 3x3 block of the stiffness tensor in 6x6 matrix that accounts for the Poisson effect.  Note that
    // with the split of the stiffness matrix and the assumed materials, the entries in the 6x6 stiffness matrix outside
    // of the upper 3x3 block are all zeros.
    // =============================================================================

    const ChMatrix33<double>& Dv = GetMaterial()->Get_Dv();

    // =============================================================================
    // For the Dv Gauss quadrature points, multiply the scaled and combined partial derivative block matrix by the
    // stiffness matrix for each individual Gauss quadrature point and place the results in th last 3*NIP_Dv columns of
    // the final scaled and combined partial derivative block matrix since the D0 Gauss quadrature point columns were
    // calculated above.
    // =============================================================================

    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + 0) =
        Dv(0, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(0, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(0, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);
    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + NIP_Dv) =
        Dv(1, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(1, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(1, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);
    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + 2 * NIP_Dv) =
        Dv(2, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(2, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(2, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);

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
    // =============================================================================
    // For the D0 Gauss quadrature points:
    // Since only the diagonal terms in the 6x6 stiffness matrix are used, the 2nd Piola-Kirchoff stress can be
    // calculated by simply scaling the vector of scaled Green-Lagrange strains in Voight notation by the corresponding
    // diagonal entry in the stiffness matrix.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    // Each entry in SPK2_1 = D11*kGQ*(E11+alpha*E11dot)
    //     = D11*kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP_D0 E_BlockDamping_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5));
    VectorNIP_D0 SPK2_1_Block_D0 = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 0)) +
                                   FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1)) +
                                   FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2));
    SPK2_1_Block_D0.array() -= 1;
    SPK2_1_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_1_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_1_Block_D0 *= (0.5 * D0(0));

    // Each entry in SPK2_2 = D22*kGQ*(E22+alpha*E22dot)
    //     = D22*kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5));
    VectorNIP_D0 SPK2_2_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_2_Block_D0.array() -= 1;
    SPK2_2_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_2_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_2_Block_D0 *= (0.5 * D0(1));

    // Each entry in SPK2_3 = D33*kGQ*(E33+alpha*E33dot)
    //     = D33*kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_3_Block_D0 =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_3_Block_D0.array() -= 1;
    SPK2_3_Block_D0 += (2 * m_Alpha) * E_BlockDamping_D0;
    SPK2_3_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_3_Block_D0 *= (0.5 * D0(2));

    // Each entry in SPK2_4 = D44*kGQ*(2*(E23+alpha*E23dot))
    //     = D44*kGQ*((F12*F13+F22*F23+F32*F33)
    //       +alpha*(F12dot*F13+F22dot*F23+F32dot*F33 + F12*F13dot+F22*F23dot+F32*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_4_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_4_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_4_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_4_Block_D0 *= D0(3);

    // Each entry in SPK2_5 = D55*kGQ*(2*(E13+alpha*E13dot))
    //     = D55*kGQ*((F11*F13+F21*F23+F31*F33)
    //       +alpha*(F11dot*F13+F21dot*F23+F31dot*F33 + F11*F13dot+F21*F23dot+F31*F33dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5)) +
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 5));
    VectorNIP_D0 SPK2_5_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_5_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_5_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_5_Block_D0 *= D0(4);

    // Each entry in SPK2_6 = D66*kGQ*(2*(E12+alpha*E12dot))
    //     = D66*kGQ*((F11*F12+F21*F22+F31*F32)
    //       +alpha*(F11dot*F12+F21dot*F22+F31dot*F32 + F11*F12dot+F21*F22dot+F31*F32dot))
    E_BlockDamping_D0.noalias() =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 3)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 4)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 5)) +
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 3)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 4)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 5));
    VectorNIP_D0 SPK2_6_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_6_Block_D0 += m_Alpha * E_BlockDamping_D0;
    SPK2_6_Block_D0.array() *= m_kGQ_D0.array();
    SPK2_6_Block_D0 *= D0(5);

    // =============================================================================
    // For the Dv Gauss quadrature points:
    // Since only the upper 3x3 terms in the 6x6 stiffness matrix are used, only the E11, E22, and E33 components of the
    // Green-Lagrange strain tensor (combined and scaled) are needed since multiplying by the Dv stiffness matrix will
    // result in 2nd Piola-Kirchoff stresses that are only possibly non-zero for SPK2_11, SPK2_22, and SPK2_33. Results
    // are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*(E11+alpha*E11dot)
    //                  = kGQ*(1/2*(F11*F11+F21*F21+F31*F31-1)+alpha*(F11*F11dot+F21*F21dot+F31*F31dot))
    VectorNIP_Dv E_BlockDamping_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 3)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 4)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 5));
    VectorNIP_Dv E1_Block_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2));
    E1_Block_Dv.array() -= 1;
    E1_Block_Dv *= 0.5;
    E1_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E1_Block_Dv.array() *= m_kGQ_Dv.array();

    // Each entry in E2 = kGQ*(E22+alpha*E22dot)
    //                  = kGQ*(1/2*(F12*F12+F22*F22+F32*F32-1)+alpha*(F12*F12dot+F22*F22dot+F32*F32dot))
    E_BlockDamping_Dv.noalias() = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 3)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 4)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 5));
    VectorNIP_Dv E2_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2));
    E2_Block_Dv.array() -= 1;
    E2_Block_Dv *= 0.5;
    E2_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E2_Block_Dv.array() *= m_kGQ_Dv.array();

    // Each entry in E3 = kGQ*(E33+alpha*E33dot)
    //                  = kGQ*(1/2*(F13*F13+F23*F23+F33*F33-1)+alpha*(F13*F13dot+F23*F23dot+F33*F33dot))
    E_BlockDamping_Dv.noalias() = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 3)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 4)) +
                                  FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                      .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 5));
    VectorNIP_Dv E3_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2));
    E3_Block_Dv.array() -= 1;
    E3_Block_Dv *= 0.5;
    E3_Block_Dv += m_Alpha * E_BlockDamping_Dv;
    E3_Block_Dv.array() *= m_kGQ_Dv.array();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Dv Gauss quadrature points.
    // Note that the Green-Largrange strain components have been scaled have already been combined with their scaled
    // time derivatives and minus the Gauss quadrature weight times the element Jacobian at the corresponding Gauss
    // point to make the calculation of the 2nd Piola-Kirchoff stresses more efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    //  where SPK2_23, SPK2_13, and SPK2_12 are zeros for the Dv Gauss quadrature points
    // =============================================================================

    VectorNIP_Dv SPK2_1_Block_Dv = Dv(0, 0) * E1_Block_Dv + Dv(0, 1) * E2_Block_Dv + Dv(0, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_2_Block_Dv = Dv(1, 0) * E1_Block_Dv + Dv(1, 1) * E2_Block_Dv + Dv(1, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_3_Block_Dv = Dv(2, 0) * E1_Block_Dv + Dv(2, 1) * E2_Block_Dv + Dv(2, 2) * E3_Block_Dv;

    // =============================================================================
    // Multiply the shape function derivative matrix by the 2nd Piola-Kirchoff stresses for each corresponding Gauss
    // quadrature point
    // =============================================================================

    ChMatrixNM<double, NSF, 3 * NIP> S_scaled_SD;

    for (auto i = 0; i < NSF; i++) {
        S_scaled_SD.template block<1, NIP_D0>(i, 0) =
            SPK2_1_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_6_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_5_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_D0>(i, NIP_D0) =
            SPK2_6_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_2_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_4_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_D0>(i, 2 * NIP_D0) =
            SPK2_5_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_4_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_3_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0) =
            SPK2_1_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + NIP_Dv) =
            SPK2_2_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv) =
            SPK2_3_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv));
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

void ChElementBeamANCF_3333::ComputeInternalJacobianContIntNoDamping(ChMatrixRef& H, double Kfactor, double Mfactor) {
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
    // calculation of the potentially non-symmetric and non-sparse component.  The second piece is the symmetric
    // and sparse component.

    // =============================================================================
    // Calculate the deformation gradient for all Gauss quadrature points in a single matrix multiplication.  Note
    // that since the shape function derivative matrix is ordered by columns, the resulting deformation gradient
    // will be ordered by block matrix (column vectors) components
    // Note that the indices of the components are in transposed order
    //      [F11  F21  F31 ] <-- D0 Block (No Poisson Effect)
    //      [F12  F22  F32 ] <-- D0 Block (No Poisson Effect)
    // FC = [F13  F23  F33 ] <-- D0 Block (No Poisson Effect)
    //      [F11  F21  F31 ] <-- Dv Block (With Poisson Effect)
    //      [F12  F22  F32 ] <-- Dv Block (With Poisson Effect)
    //      [F13  F23  F33 ] <-- Dv Block (With Poisson Effect)
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
    // PE = [(d epsilon1/d e)GQPnt1' (d epsilon1/d e)GQPnt2' ... (d epsilon1/d e)GQPntNIP_D0'...
    //       (d epsilon2/d e)GQPnt1' (d epsilon2/d e)GQPnt2' ... (d epsilon2/d e)GQPntNIP_D0'...
    //       (d epsilon3/d e)GQPnt1' (d epsilon3/d e)GQPnt2' ... (d epsilon3/d e)GQPntNIP_D0'...
    //       (d epsilon4/d e)GQPnt1' (d epsilon4/d e)GQPnt2' ... (d epsilon4/d e)GQPntNIP_D0'...
    //       (d epsilon5/d e)GQPnt1' (d epsilon5/d e)GQPnt2' ... (d epsilon5/d e)GQPntNIP_D0'...
    //       (d epsilon6/d e)GQPnt1' (d epsilon6/d e)GQPnt2' ... (d epsilon6/d e)GQPntNIP_D0'...
    //       (d epsilon1/d e)GQPntDv1' ... (d epsilon1/d e)GQPntNIP_Dv'...
    //       (d epsilon2/d e)GQPntDv1' ... (d epsilon2/d e)GQPntNIP_Dv'...
    //       (d epsilon3/d e)GQPntDv1' ... (d epsilon3/d e)GQPntNIP_Dv']
    // Note that each partial derivative block shown is placed to the left of the previous block.
    // The explanation of the calculation above is just too long to write it all on a single line.
    // Since there are no shear terms with the material assumption for Dv, there is no need to calculate
    // (d epsilon4/d e), (d epsilon5/d e), (d epsilon6/d e) for the Dv Gauss points since these partial
    // derivatives will be multiplied by zero and will not contribute anything to the Jacobian matrix.
    // The calculation is performed on 3 rows at a time to capture the partial with respect to the vector of nodal
    // coordinates e while using the compact matrix form ebar for the calculations.
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 6 * NIP_D0 + 3 * NIP_Dv> PE;

    for (auto i = 0; i < NSF; i++) {
        PE.template block<1, NIP_D0>(3 * i, 0 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 1 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        PE.template block<1, NIP_D0>(3 * i, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(0 * NIP_D0, 0).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(1 * NIP_D0, 0).transpose());

        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 0 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0 * NIP_Dv, 0).transpose());
        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 1 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 1 * NIP_Dv, 0).transpose());
        PE.template block<1, NIP_Dv>(3 * i, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).transpose());

        PE.template block<1, NIP_D0>((3 * i) + 1, 0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 1, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1).transpose());

        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 1, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).transpose());

        PE.template block<1, NIP_D0>((3 * i) + 2, 0) =
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 2 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 3 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 4 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        PE.template block<1, NIP_D0>((3 * i) + 2, 5 * NIP_D0) =
            m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2).transpose()) +
            m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                .cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2).transpose());

        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).transpose());
        PE.template block<1, NIP_Dv>((3 * i) + 2, 6 * NIP_D0 + 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).transpose());
    }

    // =============================================================================
    // Scale the block deformation gradient matrix by the Kfactor weighting factor for the Jacobian and multiply
    // each Gauss quadrature component by its Gauss quadrature weight times the element Jacobian (kGQ)
    // =============================================================================

    ChMatrixNM_col<double, 3 * NIP, 3> FCscaled = Kfactor * FC;

    for (auto i = 0; i < 3; i++) {
        FCscaled.template block<NIP_D0, 1>(0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_D0, 1>(NIP_D0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, i).array() *= m_kGQ_D0.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, i).array() *= m_kGQ_Dv.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, i).array() *= m_kGQ_Dv.array();
        FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, i).array() *= m_kGQ_Dv.array();
    }

    // =============================================================================
    // Get the diagonal terms of the 6x6 matrix (do not include the Poisson effect
    // =============================================================================
    const ChVectorN<double, 6>& D0 = GetMaterial()->Get_D0();

    // =============================================================================
    // Calculate the combination of the scaled partial derivative of the Green-Largrange strains with respect to the
    // nodal coordinates and the other parameters to correctly integrate across the
    // volume of the element multiplied by the stiffness matrix.  This calculation is performed in blocks across all the
    // Gauss quadrature points at the same time.  This value should be store in row major memory layout to align with
    // the access patterns for calculating this matrix.
    // =============================================================================
    // For the D0 Gauss quadrature points, the stiffness matrix D0 is diagonal so the scaled partial derivatives can be
    // easily multiplied by D0.  The Dv Gauss quadrature points are handled separately after this
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 6 * NIP_D0 + 3 * NIP_Dv> DScaled_Combined_PE;

    for (auto i = 0; i < NSF; i++) {
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 0).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>(3 * i, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 0).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 0).transpose()));

        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 1).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 1, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 1).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 1).transpose()));

        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 0) =
            D0(0) * m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, NIP_D0) =
            D0(1) * m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 2 * NIP_D0) =
            D0(2) * m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                        .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose());
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 3 * NIP_D0) =
            D0(3) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 4 * NIP_D0) =
            D0(4) * (m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(2 * NIP_D0, 2).transpose()));
        DScaled_Combined_PE.template block<1, NIP_D0>((3 * i) + 2, 5 * NIP_D0) =
            D0(5) * (m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(0, 2).transpose()) +
                     m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)
                         .cwiseProduct(FCscaled.template block<NIP_D0, 1>(NIP_D0, 2).transpose()));
    }

    // =============================================================================
    // For the Dv Gauss quadrature points, the matrix of scaled partial derivatives needs to be calculated first since
    // they will be will be combined together when multiplying by the Dv stiffness matrix (upper 3x3 block is
    // potentially non-zero)
    // =============================================================================

    ChMatrixNM<double, 3 * NSF, 3 * NIP_Dv> Scaled_Combined_PE_Dv;

    for (auto i = 0; i < NSF; i++) {
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>(3 * i, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0).transpose());

        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 1, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1).transpose());

        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, 0) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2).transpose());
        Scaled_Combined_PE_Dv.template block<1, NIP_Dv>((3 * i) + 2, 2 * NIP_Dv) =
            m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv)
                .cwiseProduct(FCscaled.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2).transpose());
    }

    // =============================================================================
    // Get the upper 3x3 block of the stiffness tensor in 6x6 matrix that accounts for the Poisson effect.  Note that
    // with the split of the stiffness matrix and the assumed materials, the entries in the 6x6 stiffness matrix outside
    // of the upper 3x3 block are all zeros.
    // =============================================================================

    const ChMatrix33<double>& Dv = GetMaterial()->Get_Dv();

    // =============================================================================
    // For the Dv Gauss quadrature points, multiply the scaled and combined partial derivative block matrix by the
    // stiffness matrix for each individual Gauss quadrature point and place the results in th last 3*NIP_Dv columns of
    // the final scaled and combined partial derivative block matrix since the D0 Gauss quadrature point columns were
    // calculated above.
    // =============================================================================

    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + 0) =
        Dv(0, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(0, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(0, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);
    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + NIP_Dv) =
        Dv(1, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(1, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(1, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);
    DScaled_Combined_PE.template block<3 * NSF, NIP_Dv>(0, 6 * NIP_D0 + 2 * NIP_Dv) =
        Dv(2, 0) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 0) +
        Dv(2, 1) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, NIP_Dv) +
        Dv(2, 2) * Scaled_Combined_PE_Dv.template block<3 * NSF, NIP_Dv>(0, 2 * NIP_Dv);

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
    // Note that the result is then scaled by minus the Gauss quadrature weight times the element Jacobian at the
    // corresponding Gauss point (m_kGQ) again for efficiency.
    // =============================================================================
    // For the D0 Gauss quadrature points:
    // Since only the diagonal terms in the 6x6 stiffness matrix are used, the 2nd Piola-Kirchoff stress can be
    // calculated by simply scaling the vector of scaled Green-Lagrange strains in Voight notation by the corresponding
    // diagonal entry in the stiffness matrix.
    // Results are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    // =============================================================================

    // Each entry in SPK2_1 = D11*kGQ*E11 = D11*kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP_D0 SPK2_1_Block_D0 = FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(0, 0)) +
                                   FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(0, 1)) +
                                   FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(0, 2));
    SPK2_1_Block_D0.array() -= 1;
    SPK2_1_Block_D0.array() *= (0.5 * D0(0)) * m_kGQ_D0.array();

    // Each entry in SPK2_2 = D22*kGQ*E22 = D22*kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP_D0 SPK2_2_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_2_Block_D0.array() -= 1;
    SPK2_2_Block_D0.array() *= (0.5 * D0(1)) * m_kGQ_D0.array();

    // Each entry in SPK2_3 = D33*kGQ_D0*E33 = D33*kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP_D0 SPK2_3_Block_D0 =
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(2 * NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_3_Block_D0.array() -= 1;
    SPK2_3_Block_D0.array() *= (0.5 * D0(2)) * m_kGQ_D0.array();

    // Each entry in SPK2_4 = D44*kGQ*2*E23 = D44*kGQ*(F12*F13+F22*F23+F32*F33)
    VectorNIP_D0 SPK2_4_Block_D0 =
        FC.template block<NIP_D0, 1>(NIP_D0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(NIP_D0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_4_Block_D0.array() *= D0(3) * m_kGQ_D0.array();

    // Each entry in SPK2_5 = D55*kGQ*2*E13 = D55*kGQ*(F11*F13+F21*F23+F31*F33)
    VectorNIP_D0 SPK2_5_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(2 * NIP_D0, 2));
    SPK2_5_Block_D0.array() *= D0(4) * m_kGQ_D0.array();

    // Each entry in SPK2_6 = D66*kGQ*2*E12 = D66*(F11*F12+F21*F22+F31*F32)
    VectorNIP_D0 SPK2_6_Block_D0 =
        FC.template block<NIP_D0, 1>(0, 0).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 0)) +
        FC.template block<NIP_D0, 1>(0, 1).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 1)) +
        FC.template block<NIP_D0, 1>(0, 2).cwiseProduct(FC.template block<NIP_D0, 1>(NIP_D0, 2));
    SPK2_6_Block_D0.array() *= D0(5) * m_kGQ_D0.array();

    // =============================================================================
    // For the Dv Gauss quadrature points:
    // Since only the upper 3x3 terms in the 6x6 stiffness matrix are used, only the E11, E22, and E33 components of the
    // Green-Lagrange strain tensor (scaled) are needed since multiplying by the Dv stiffness matrix will
    // result in 2nd Piola-Kirchoff stresses that are only possibly non-zero for SPK2_11, SPK2_22, and SPK2_33. Results
    // are written in Voigt notation: epsilon = [E11,E22,E33,2*E23,2*E13,2*E12]
    // =============================================================================

    // Each entry in E1 = kGQ*E11 = kGQ*1/2*(F11*F11+F21*F21+F31*F31-1)
    VectorNIP_Dv E1_Block_Dv =
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 0)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 1)) +
        FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2).cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 0, 2));
    E1_Block_Dv.array() -= 1;
    E1_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // Each entry in E2 = kGQ*E22 = kGQ*1/2*(F12*F12+F22*F22+F32*F32-1)
    VectorNIP_Dv E2_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + NIP_Dv, 2));
    E2_Block_Dv.array() -= 1;
    E2_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // Each entry in E3 = kGQ*E33 = kGQ*1/2*(F13*F13+F23*F23+F33*F33-1)
    VectorNIP_Dv E3_Block_Dv = FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 0)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 1)) +
                               FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2)
                                   .cwiseProduct(FC.template block<NIP_Dv, 1>(3 * NIP_D0 + 2 * NIP_Dv, 2));
    E3_Block_Dv.array() -= 1;
    E3_Block_Dv.array() *= 0.5 * m_kGQ_Dv.array();

    // =============================================================================
    // Calculate the 2nd Piola-Kirchoff stresses in Voigt notation across all the Dv Gauss quadrature points.
    // Note that the Green-Largrange strain components have been scaled by minus the Gauss quadrature weight times the
    // element Jacobian at the corresponding Gauss point to make the calculation of the 2nd Piola-Kirchoff stresses more
    // efficient.
    //  kGQ*SPK2 = kGQ*[SPK2_11,SPK2_22,SPK2_33,SPK2_23,SPK2_13,SPK2_12] = D * E_Combined
    //  where SPK2_23, SPK2_13, and SPK2_12 are zeros for the Dv Gauss quadrature points
    // =============================================================================

    VectorNIP_Dv SPK2_1_Block_Dv = Dv(0, 0) * E1_Block_Dv + Dv(0, 1) * E2_Block_Dv + Dv(0, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_2_Block_Dv = Dv(1, 0) * E1_Block_Dv + Dv(1, 1) * E2_Block_Dv + Dv(1, 2) * E3_Block_Dv;
    VectorNIP_Dv SPK2_3_Block_Dv = Dv(2, 0) * E1_Block_Dv + Dv(2, 1) * E2_Block_Dv + Dv(2, 2) * E3_Block_Dv;

    // =============================================================================
    // Multiply the shape function derivative matrix by the 2nd Piola-Kirchoff stresses for each corresponding Gauss
    // quadrature point
    // =============================================================================

    ChMatrixNM<double, NSF, 3 * NIP> S_scaled_SD;

    for (auto i = 0; i < NSF; i++) {
        S_scaled_SD.template block<1, NIP_D0>(i, 0) =
            SPK2_1_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_6_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_5_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_D0>(i, NIP_D0) =
            SPK2_6_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_2_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_4_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_D0>(i, 2 * NIP_D0) =
            SPK2_5_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 0 * NIP_D0)) +
            SPK2_4_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 1 * NIP_D0)) +
            SPK2_3_Block_D0.transpose().cwiseProduct(m_SD.template block<1, NIP_D0>(i, 2 * NIP_D0));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0) =
            SPK2_1_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 0 * NIP_Dv));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + NIP_Dv) =
            SPK2_2_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 1 * NIP_Dv));

        S_scaled_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv) =
            SPK2_3_Block_Dv.transpose().cwiseProduct(m_SD.template block<1, NIP_Dv>(i, 3 * NIP_D0 + 2 * NIP_Dv));
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

void ChElementBeamANCF_3333::ComputeInternalJacobianPreInt(ChMatrixRef& H,
                                                           double Kfactor,
                                                           double Rfactor,
                                                           double Mfactor) {
    // Calculate the Jacobian of the generalize internal force vector using the "Pre-Integration" style of method
    // assuming a linear viscoelastic material model (single term damping model).  For this style of method, the
    // components of the generalized internal force vector and its Jacobian that need to be integrated across the volume
    // are calculated once prior to the start of the simulation.  Since computationally expensive quantities are
    // required for both the generalized internal force vector and its Jacobian, these values were cached for reuse
    // during this Jacobian calculation.

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
    ChMatrixNM_col<double, 9, NSF* NSF> K2 = -PI2 * m_O2;

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

void ChElementBeamANCF_3333::Calc_Sxi_compact(VectorN& Sxi_compact, double xi, double eta, double zeta) {
    Sxi_compact(0) = 0.5 * (xi * xi - xi);
    Sxi_compact(1) = 0.25 * m_thicknessY * eta * (xi * xi - xi);
    Sxi_compact(2) = 0.25 * m_thicknessZ * zeta * (xi * xi - xi);
    Sxi_compact(3) = 0.5 * (xi * xi + xi);
    Sxi_compact(4) = 0.25 * m_thicknessY * eta * (xi * xi + xi);
    Sxi_compact(5) = 0.25 * m_thicknessZ * zeta * (xi * xi + xi);
    Sxi_compact(6) = 1.0 - xi * xi;
    Sxi_compact(7) = 0.5 * m_thicknessY * eta * (1.0 - xi * xi);
    Sxi_compact(8) = 0.5 * m_thicknessZ * zeta * (1.0 - xi * xi);
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to xi
// [s1; s2; s3; ...]

void ChElementBeamANCF_3333::Calc_Sxi_xi_compact(VectorN& Sxi_xi_compact, double xi, double eta, double zeta) {
    Sxi_xi_compact(0) = xi - 0.5;
    Sxi_xi_compact(1) = 0.25 * m_thicknessY * eta * (2.0 * xi - 1.0);
    Sxi_xi_compact(2) = 0.25 * m_thicknessZ * zeta * (2.0 * xi - 1.0);
    Sxi_xi_compact(3) = xi + 0.5;
    Sxi_xi_compact(4) = 0.25 * m_thicknessY * eta * (2.0 * xi + 1.0);
    Sxi_xi_compact(5) = 0.25 * m_thicknessZ * zeta * (2.0 * xi + 1.0);
    Sxi_xi_compact(6) = -2.0 * xi;
    Sxi_xi_compact(7) = -m_thicknessY * eta * xi;
    Sxi_xi_compact(8) = -m_thicknessZ * zeta * xi;
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to eta
// [s1; s2; s3; ...]

void ChElementBeamANCF_3333::Calc_Sxi_eta_compact(VectorN& Sxi_eta_compact, double xi, double eta, double zeta) {
    Sxi_eta_compact(0) = 0.0;
    Sxi_eta_compact(1) = 0.25 * m_thicknessY * (xi * xi - xi);
    Sxi_eta_compact(2) = 0.0;
    Sxi_eta_compact(3) = 0.0;
    Sxi_eta_compact(4) = 0.25 * m_thicknessY * (xi * xi + xi);
    Sxi_eta_compact(5) = 0.0;
    Sxi_eta_compact(6) = 0.0;
    Sxi_eta_compact(7) = 0.5 * m_thicknessY * (1 - xi * xi);
    Sxi_eta_compact(8) = 0.0;
}

// Nx1 Vector Form of the partial derivatives of Normalized Shape Functions with respect to zeta
// [s1; s2; s3; ...]

void ChElementBeamANCF_3333::Calc_Sxi_zeta_compact(VectorN& Sxi_zeta_compact, double xi, double eta, double zeta) {
    Sxi_zeta_compact(0) = 0.0;
    Sxi_zeta_compact(1) = 0.0;
    Sxi_zeta_compact(2) = 0.25 * m_thicknessZ * (xi * xi - xi);
    Sxi_zeta_compact(3) = 0.0;
    Sxi_zeta_compact(4) = 0.0;
    Sxi_zeta_compact(5) = 0.25 * m_thicknessZ * (xi * xi + xi);
    Sxi_zeta_compact(6) = 0.0;
    Sxi_zeta_compact(7) = 0.0;
    Sxi_zeta_compact(8) = 0.5 * m_thicknessZ * (1 - xi * xi);
}

// Nx3 compact form of the partial derivatives of Normalized Shape Functions with respect to xi, eta, and zeta by
// columns

void ChElementBeamANCF_3333::Calc_Sxi_D(MatrixNx3c& Sxi_D, double xi, double eta, double zeta) {
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

void ChElementBeamANCF_3333::CalcCoordVector(Vector3N& e) {
    e.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    e.segment(3, 3) = m_nodes[0]->GetD().eigen();
    e.segment(6, 3) = m_nodes[0]->GetDD().eigen();

    e.segment(9, 3) = m_nodes[1]->GetPos().eigen();
    e.segment(12, 3) = m_nodes[1]->GetD().eigen();
    e.segment(15, 3) = m_nodes[1]->GetDD().eigen();

    e.segment(18, 3) = m_nodes[2]->GetPos().eigen();
    e.segment(21, 3) = m_nodes[2]->GetD().eigen();
    e.segment(24, 3) = m_nodes[2]->GetDD().eigen();
}

void ChElementBeamANCF_3333::CalcCoordMatrix(Matrix3xN& ebar) {
    ebar.col(0) = m_nodes[0]->GetPos().eigen();
    ebar.col(1) = m_nodes[0]->GetD().eigen();
    ebar.col(2) = m_nodes[0]->GetDD().eigen();

    ebar.col(3) = m_nodes[1]->GetPos().eigen();
    ebar.col(4) = m_nodes[1]->GetD().eigen();
    ebar.col(5) = m_nodes[1]->GetDD().eigen();

    ebar.col(6) = m_nodes[2]->GetPos().eigen();
    ebar.col(7) = m_nodes[2]->GetD().eigen();
    ebar.col(8) = m_nodes[2]->GetDD().eigen();
}

void ChElementBeamANCF_3333::CalcCoordDerivVector(Vector3N& edot) {
    edot.segment(0, 3) = m_nodes[0]->GetPos_dt().eigen();
    edot.segment(3, 3) = m_nodes[0]->GetD_dt().eigen();
    edot.segment(6, 3) = m_nodes[0]->GetDD_dt().eigen();

    edot.segment(9, 3) = m_nodes[1]->GetPos_dt().eigen();
    edot.segment(12, 3) = m_nodes[1]->GetD_dt().eigen();
    edot.segment(15, 3) = m_nodes[1]->GetDD_dt().eigen();

    edot.segment(18, 3) = m_nodes[2]->GetPos_dt().eigen();
    edot.segment(21, 3) = m_nodes[2]->GetD_dt().eigen();
    edot.segment(24, 3) = m_nodes[2]->GetDD_dt().eigen();
}

void ChElementBeamANCF_3333::CalcCoordDerivMatrix(Matrix3xN& ebardot) {
    ebardot.col(0) = m_nodes[0]->GetPos_dt().eigen();
    ebardot.col(1) = m_nodes[0]->GetD_dt().eigen();
    ebardot.col(2) = m_nodes[0]->GetDD_dt().eigen();

    ebardot.col(3) = m_nodes[1]->GetPos_dt().eigen();
    ebardot.col(4) = m_nodes[1]->GetD_dt().eigen();
    ebardot.col(5) = m_nodes[1]->GetDD_dt().eigen();

    ebardot.col(6) = m_nodes[2]->GetPos_dt().eigen();
    ebardot.col(7) = m_nodes[2]->GetD_dt().eigen();
    ebardot.col(8) = m_nodes[2]->GetDD_dt().eigen();
}

void ChElementBeamANCF_3333::CalcCombinedCoordMatrix(MatrixNx6& ebar_ebardot) {
    ebar_ebardot.template block<1, 3>(0, 0) = m_nodes[0]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(0, 3) = m_nodes[0]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(1, 0) = m_nodes[0]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(1, 3) = m_nodes[0]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(2, 0) = m_nodes[0]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(2, 3) = m_nodes[0]->GetDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(3, 0) = m_nodes[1]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(3, 3) = m_nodes[1]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(4, 0) = m_nodes[1]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(4, 3) = m_nodes[1]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(5, 0) = m_nodes[1]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(5, 3) = m_nodes[1]->GetDD_dt().eigen();

    ebar_ebardot.template block<1, 3>(6, 0) = m_nodes[2]->GetPos().eigen();
    ebar_ebardot.template block<1, 3>(6, 3) = m_nodes[2]->GetPos_dt().eigen();
    ebar_ebardot.template block<1, 3>(7, 0) = m_nodes[2]->GetD().eigen();
    ebar_ebardot.template block<1, 3>(7, 3) = m_nodes[2]->GetD_dt().eigen();
    ebar_ebardot.template block<1, 3>(8, 0) = m_nodes[2]->GetDD().eigen();
    ebar_ebardot.template block<1, 3>(8, 3) = m_nodes[2]->GetDD_dt().eigen();
}

// Calculate the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

void ChElementBeamANCF_3333::Calc_J_0xi(ChMatrix33<double>& J_0xi, double xi, double eta, double zeta) {
    MatrixNx3c Sxi_D;
    Calc_Sxi_D(Sxi_D, xi, eta, zeta);

    J_0xi = m_ebar0 * Sxi_D;
}

// Calculate the determinant of the 3x3 Element Jacobian at the given point (xi,eta,zeta) in the element

double ChElementBeamANCF_3333::Calc_det_J_0xi(double xi, double eta, double zeta) {
    ChMatrix33<double> J_0xi;
    Calc_J_0xi(J_0xi, xi, eta, zeta);

    return (J_0xi.determinant());
}

////////////////////////////////////////////////////////////////

//#ifndef CH_QUADRATURE_STATIC_TABLES
#define CH_QUADRATURE_STATIC_TABLES 10
ChQuadratureTables static_tables_3333(1, CH_QUADRATURE_STATIC_TABLES);
//#endif // !CH_QUADRATURE_STATIC_TABLES

ChQuadratureTables* ChElementBeamANCF_3333::GetStaticGQTables() {
    return &static_tables_3333;
}

}  // namespace fea
}  // namespace chrono
