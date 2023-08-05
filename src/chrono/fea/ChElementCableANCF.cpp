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
// Authors: Alessandro Tasora, Radu Serban, Antonio Recuero
// =============================================================================
// ANCF gradient-deficient cable element.
// =============================================================================

#include <cmath>

#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChElementCableANCF.h"

namespace chrono {
namespace fea {

ChElementCableANCF::ChElementCableANCF() : m_alpha(0), m_use_damping(false) {
    m_nodes.resize(2);
}

void ChElementCableANCF::SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA, std::shared_ptr<ChNodeFEAxyzD> nodeB) {
    assert(nodeA);
    assert(nodeB);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
    Kmatr.SetVariables(mvars);
}

void ChElementCableANCF::ShapeFunctions(ShapeVector& N, double xi) {
    double l = GetRestLength();

    N(0) = 1 - 3 * pow(xi, 2) + 2 * pow(xi, 3);
    N(1) = l * (xi - 2 * pow(xi, 2) + pow(xi, 3));
    N(2) = 3 * pow(xi, 2) - 2 * pow(xi, 3);
    N(3) = l * (-pow(xi, 2) + pow(xi, 3));
};

void ChElementCableANCF::ShapeFunctionsDerivatives(ShapeVector& Nd, double xi) {
    double l = GetRestLength();

    Nd(0) = (6.0 * pow(xi, 2.0) - 6.0 * xi) / l;
    Nd(1) = 1.0 - 4.0 * xi + 3.0 * pow(xi, 2.0);
    Nd(2) = -(6.0 * pow(xi, 2.0) - 6.0 * xi) / l;
    Nd(3) = -2.0 * xi + 3.0 * pow(xi, 2.0);
};

void ChElementCableANCF::ShapeFunctionsDerivatives2(ShapeVector& Ndd, double xi) {
    double l = GetRestLength();
    Ndd(0) = (12 * xi - 6) / pow(l, 2);
    Ndd(1) = (-4 + 6 * xi) / l;
    Ndd(2) = (6 - 12 * xi) / pow(l, 2);
    Ndd(3) = (-2 + 6 * xi) / l;
};

void ChElementCableANCF::Update() {
    // parent class update:
    ChElementGeneric::Update();
};

void ChElementCableANCF::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.resize(12);

    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(6, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(9, 3) = m_nodes[1]->GetD().eigen();
}

// Computes the stiffness matrix of the element:
// K = integral( .... ),
// Note: in this 'basic' implementation, constant section and constant material are assumed.
void ChElementCableANCF::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    assert(m_section);
    bool use_numerical_differentiation = true;  // Only option tested for now

    // Option: compute the stiffness matrix by doing a numerical differentiation
    // of the internal forces. This fixes a problem with the code by D.Melanz, that
    // produces a rank deficient matrix for straight beams.
    if (use_numerical_differentiation) {
        double diff = 1e-8;
        ChVectorDynamic<> F0(12);
        ChVectorDynamic<> F1(12);

        ComputeInternalForces(F0);

        // Create local copies of the nodal coordinates and use the implementation version
        // of the function for calculating the internal forces.  With this, the calculation
        // of the Jacobian with finite differences is thread safe (otherwise, there would
        // be race conditions when adjacent elements attempt to perturb a common node).
        ChVector<> pos[2] = {m_nodes[0]->GetPos(), m_nodes[1]->GetPos()};
        ChVector<> D[2] = {m_nodes[0]->GetD(), m_nodes[1]->GetD()};
        ChVector<> pos_dt[2] = {m_nodes[0]->GetPos_dt(), m_nodes[1]->GetPos_dt()};
        ChVector<> D_dt[2] = {m_nodes[0]->GetD_dt(), m_nodes[1]->GetD_dt()};

        // Add part of the Jacobian stemming from elastic forces
        for (int inode = 0; inode < 2; ++inode) {
            pos[inode].x() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(0 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            pos[inode].x() -= diff;

            pos[inode].y() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(1 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            pos[inode].y() -= diff;

            pos[inode].z() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(2 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            pos[inode].z() -= diff;

            D[inode].x() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(3 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            D[inode].x() -= diff;

            D[inode].y() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(4 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            D[inode].y() -= diff;

            D[inode].z() += diff;
            ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            m_JacobianMatrix.col(5 + inode * 6) = (F0 - F1) * (1.0 / diff) * Kfactor;
            D[inode].z() -= diff;
        }

        // Add part of the Jacobian stemming from internal damping forces, if selected by user.
        if (m_use_damping) {
            for (int inode = 0; inode < 2; ++inode) {
                pos_dt[inode].x() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(0 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                pos_dt[inode].x() -= diff;

                pos_dt[inode].y() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(1 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                pos_dt[inode].y() -= diff;

                pos_dt[inode].z() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(2 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                pos_dt[inode].z() -= diff;

                D_dt[inode].x() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(3 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                D_dt[inode].x() -= diff;

                D_dt[inode].y() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(4 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                D_dt[inode].y() -= diff;

                D_dt[inode].z() += diff;
                ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
                m_JacobianMatrix.col(5 + inode * 6) += (F0 - F1) * (1.0 / diff) * Rfactor;
                D_dt[inode].z() -= diff;
            }
        }
    } else {
        // Option B: use the code in D.Melanz thesis. These formulas, however,
        // produce a rank deficient matrix for straight beams.
        //// TODO: Test it and include internal damping from strain rates.
        double Area = m_section->Area;
        double E = m_section->E;
        double I = m_section->I;

        ChVector<> pA = m_nodes[0]->GetPos();
        ChVector<> dA = m_nodes[0]->GetD();
        ChVector<> pB = m_nodes[1]->GetPos();
        ChVector<> dB = m_nodes[1]->GetD();

        // this matrix will be used in both CableANCF_StiffnessAxial and CableANCF_StiffnessCurv integrators
        ChMatrixNM<double, 4, 3> d;
        d(0, 0) = pA.x();
        d(0, 1) = pA.y();
        d(0, 2) = pA.z();
        d(1, 0) = dA.x();
        d(1, 1) = dA.y();
        d(1, 2) = dA.z();
        d(2, 0) = pB.x();
        d(2, 1) = pB.y();
        d(2, 2) = pB.z();
        d(3, 0) = dB.x();
        d(3, 1) = dB.y();
        d(3, 2) = dB.z();

        // 1)
        // Integrate   ((strainD'*strainD)+(strain*Sd'*Sd))

        class CableANCF_StiffnessAxial : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
          public:
            ChElementCableANCF* element;
            ChMatrixNM<double, 4, 3>* d;

            // Evaluate ((strainD'*strainD)+(strain*Sd'*Sd)) at point x
            virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) {
                ChElementCableANCF::ShapeVector Nd;
                element->ShapeFunctionsDerivatives(Nd, x);

                // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
                ChMatrixNM<double, 3, 12> Sd;
                Sd.setZero();
                for (int i = 0; i < 4; i++) {
                    Sd(0, 3 * i + 0) = Nd(i);
                    Sd(1, 3 * i + 1) = Nd(i);
                    Sd(2, 3 * i + 2) = Nd(i);
                }

                ChMatrixNM<double, 1, 3> Nd_d = Nd * (*d);
                ChMatrixNM<double, 1, 12> strainD = Nd_d * Sd;

                // strain = (Nd*(d*d')*Nd'-1)*0.5;
                double strain = 0.5 * (Nd_d.dot(Nd_d) - 1);

                // result:  ((strainD'*strainD)+(strain*Sd'*Sd))
                result = strainD.transpose() * strainD + strain * Sd.transpose() * Sd;
            }
        };

        CableANCF_StiffnessAxial myformulaAx;
        myformulaAx.d = &d;
        myformulaAx.element = this;

        ChMatrixNM<double, 12, 12> Kaxial;
        Kaxial.setZero();
        ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(Kaxial,       // result of integration will go there
                                                              myformulaAx,  // formula to integrate
                                                              0,            // start of x
                                                              1,            // end of x
                                                              5             // order of integration
        );

        m_JacobianMatrix = E * Area * length * Kaxial;

        // 2)
        // Integrate   (k_e'*k_e)

        class CableANCF_StiffnessCurv : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
          public:
            ChElementCableANCF* element;
            ChMatrixNM<double, 4, 3>* d;
            ChMatrixNM<double, 3, 12> Sd;
            ChMatrixNM<double, 3, 12> Sdd;
            ChElementCableANCF::ShapeVector Nd;
            ChElementCableANCF::ShapeVector Ndd;

            ChMatrixNM<double, 1, 12> g_e;
            ChMatrixNM<double, 1, 12> f_e;
            ChMatrixNM<double, 1, 12> k_e;
            ChMatrixNM<double, 3, 12> fe1;

            // Evaluate  at point x
            virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) {
                element->ShapeFunctionsDerivatives(Nd, x);
                element->ShapeFunctionsDerivatives2(Ndd, x);

                // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
                // Sdd=[Ndd1*eye(3) Ndd2*eye(3) Ndd3*eye(3) Ndd4*eye(3)]
                Sd.setZero();
                for (int i = 0; i < 4; i++) {
                    Sd(0, 3 * i + 0) = Nd(i);
                    Sd(1, 3 * i + 1) = Nd(i);
                    Sd(2, 3 * i + 2) = Nd(i);
                }

                Sdd.setZero();
                for (int i = 0; i < 4; i++) {
                    Sdd(0, 3 * i + 0) = Ndd(i);
                    Sdd(1, 3 * i + 1) = Ndd(i);
                    Sdd(2, 3 * i + 2) = Ndd(i);
                }

                ChVector<> vr_x((*d).transpose() * Nd.transpose());
                ChVector<> vr_xx((*d).transpose() * Ndd.transpose());
                ChVector<> vf1 = Vcross(vr_x, vr_xx);
                double f = vf1.Length();
                double g1 = vr_x.Length();
                double g = pow(g1, 3);

                g_e = (3 * g1) * Nd * (*d) * Sd;

                // do:  fe1=cross(Sd,r_xxrep)+cross(r_xrep,Sdd);
                for (int col = 0; col < 12; ++col) {
                    ChVector<> Sd_i = Sd.col(col);
                    fe1.col(col) = Vcross(Sd_i, vr_xx).eigen();
                    ChVector<> Sdd_i = Sdd.col(col);
                    fe1.col(col) = Vcross(vr_x, Sdd_i).eigen();
                }
                ChVectorN<double, 3> f1 = vf1.eigen();

                if (f == 0)
                    f_e = f1.transpose() * fe1;
                else {
                    f_e = (1 / f) * f1.transpose() * fe1;
                }

                k_e = (f_e * g - g_e * f) * (1 / (pow(g, 2)));

                result = k_e.transpose() * k_e;
            }
        };

        CableANCF_StiffnessCurv myformulaCurv;
        myformulaCurv.d = &d;
        myformulaCurv.element = this;

        ChMatrixNM<double, 12, 12> Kcurv;
        Kcurv.setZero();
        ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(Kcurv,          // result of integration will go there
                                                              myformulaCurv,  // formula to integrate
                                                              0,              // start of x
                                                              1,              // end of x
                                                              3               // order of integration
        );

        m_JacobianMatrix += (E * I * length) * Kcurv;  // Iyy should be the same value (circular section assumption)
    }

    //***DEBUG***
    /*
    GetLog() << "Stiffness matr file dump. L=" << length << " A=" << m_section->Area << " E=" <<
    m_section->E << " I=" << m_section->Izz << "\n";
    GetLog() << StiffnessMatrix;
    ChStreamOutAsciiFile mdump("dump_stiff.txt");
    StiffnessMatrix.StreamOutDenseMatlabFormat(mdump);
    */
}

// Computes the mass matrix of the element.
// Note: in this 'basic' implementation, constant section and constant material are assumed.
void ChElementCableANCF::ComputeMassMatrix() {
    assert(m_section);

    double L = length;
    double rhoAL = m_section->density * m_section->Area * L;
    double rhoAL2 = rhoAL * L;
    double rhoAL3 = rhoAL2 * L;

    m_MassMatrix.setZero();

    m_MassMatrix.block<3, 3>(0, 0).fillDiagonal(13.0 / 35.0 * rhoAL);
    m_MassMatrix.block<3, 3>(0, 3).fillDiagonal(11.0 / 210.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(0, 6).fillDiagonal(9.0 / 70.0 * rhoAL);
    m_MassMatrix.block<3, 3>(0, 9).fillDiagonal(-13.0 / 420.0 * rhoAL2);

    m_MassMatrix.block<3, 3>(3, 0).fillDiagonal(11.0 / 210.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(3, 3).fillDiagonal(1.0 / 105.0 * rhoAL3);
    m_MassMatrix.block<3, 3>(3, 6).fillDiagonal(13.0 / 420.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(3, 9).fillDiagonal(-1.0 / 140.0 * rhoAL3);

    m_MassMatrix.block<3, 3>(6, 0).fillDiagonal(9.0 / 70.0 * rhoAL);
    m_MassMatrix.block<3, 3>(6, 3).fillDiagonal(13.0 / 420.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(6, 6).fillDiagonal(13.0 / 35.0 * rhoAL);
    m_MassMatrix.block<3, 3>(6, 9).fillDiagonal(-11.0 / 210.0 * rhoAL2);

    m_MassMatrix.block<3, 3>(9, 0).fillDiagonal(-13.0 / 420.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(9, 3).fillDiagonal(-1.0 / 140.0 * rhoAL3);
    m_MassMatrix.block<3, 3>(9, 6).fillDiagonal(-11.0 / 210.0 * rhoAL2);
    m_MassMatrix.block<3, 3>(9, 9).fillDiagonal(1.0 / 105.0 * rhoAL3);
}

// Setup: precompute mass and matrices that do not change during the simulation.
void ChElementCableANCF::SetupInitial(ChSystem* system) {
    assert(m_section);

    m_element_dof = 0;
    for (int i = 0; i < 2; i++) {
        m_element_dof += m_nodes[i]->GetNdofX();
    }

    m_full_dof = (m_element_dof == 2 * 6);

    if (!m_full_dof) {
        m_mapping_dof.resize(m_element_dof);
        int dof = 0;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < m_nodes[i]->GetNdofX(); j++)
                m_mapping_dof(dof++) = i * 6 + j;
        }
    }

    // Compute rest length, mass:
    length = (m_nodes[1]->GetX0() - m_nodes[0]->GetX0()).Length();
    mass = length * m_section->Area * m_section->density;

    // Here we calculate the internal forces in the initial configuration
    // Contribution of initial configuration in elastic forces is automatically subtracted
    ChVectorDynamic<> FVector0(12);
    FVector0.setZero();
    m_GenForceVec0.setZero();  // Note: this is important here (m_GenForceVec0 used in ComputeInternalForces)
    ComputeInternalForces(FVector0);
    m_GenForceVec0 = FVector0;

    // Compute mass matrix
    ComputeMassMatrix();

    // Compute the matrix used to multiply the acceleration due to gravity to get the generalized gravitational force
    // vector for the element
    m_GravForceScale(0) = 0.5 * m_section->density * m_section->Area * length;
    m_GravForceScale(1) = 1.0 / 12.0 * m_section->density * m_section->Area * length * length;
    m_GravForceScale(2) = 0.5 * m_section->density * m_section->Area * length;
    m_GravForceScale(3) = -1.0 / 12.0 * m_section->density * m_section->Area * length * length;
}

// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also superimposes global
// damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
void ChElementCableANCF::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == GetNdofs()) && (H.cols() == GetNdofs()));
    assert(m_section);

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    H = m_JacobianMatrix + Mfactor * m_MassMatrix;
}

// Computes the internal forces and set values in the Fi vector.
// (e.g. the actual position of nodes is not in relaxed reference position).
void ChElementCableANCF::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    ComputeInternalForces_Impl(m_nodes[0]->GetPos(), m_nodes[0]->GetD(), m_nodes[1]->GetPos(), m_nodes[1]->GetD(),
                               m_nodes[0]->GetPos_dt(), m_nodes[0]->GetD_dt(), m_nodes[1]->GetPos_dt(),
                               m_nodes[1]->GetD_dt(), Fi);
}

// Worker function for computing the internal forces.
// This function takes the nodal coordinates as arguments and is therefore thread-safe.
// (Typically invoked by ComputeInternalForces. Used explicitly in the FD Jacobian approximation).
void ChElementCableANCF::ComputeInternalForces_Impl(const ChVector<>& pA,
                                                    const ChVector<>& dA,
                                                    const ChVector<>& pB,
                                                    const ChVector<>& dB,
                                                    const ChVector<>& pA_dt,
                                                    const ChVector<>& dA_dt,
                                                    const ChVector<>& pB_dt,
                                                    const ChVector<>& dB_dt,
                                                    ChVectorDynamic<>& Fi) {
    assert(Fi.size() == GetNdofs());
    assert(m_section);

    double Area = m_section->Area;
    double E = m_section->E;
    double I = m_section->I;

    // this matrix will be used in both CableANCF_ForceAxial and CableANCF_ForceCurv integrators
    ChMatrixNM<double, 4, 3> d;
    d(0, 0) = pA.x();
    d(0, 1) = pA.y();
    d(0, 2) = pA.z();
    d(1, 0) = dA.x();
    d(1, 1) = dA.y();
    d(1, 2) = dA.z();
    d(2, 0) = pB.x();
    d(2, 1) = pB.y();
    d(2, 2) = pB.z();
    d(3, 0) = dB.x();
    d(3, 1) = dB.y();
    d(3, 2) = dB.z();

    // this matrix will be used in both CableANCF_ForceAxial and CableANCF_ForceCurv integrators
    ChVectorN<double, 12> vel_vector;
    vel_vector(0) = pA_dt.x();
    vel_vector(1) = pA_dt.y();
    vel_vector(2) = pA_dt.z();
    vel_vector(3) = dA_dt.x();
    vel_vector(4) = dA_dt.y();
    vel_vector(5) = dA_dt.z();
    vel_vector(6) = pB_dt.x();
    vel_vector(7) = pB_dt.y();
    vel_vector(8) = pB_dt.z();
    vel_vector(9) = dB_dt.x();
    vel_vector(10) = dB_dt.y();
    vel_vector(11) = dB_dt.z();

    // 1)
    // Integrate   (strainD'*strain)

    class CableANCF_ForceAxial : public ChIntegrable1D<ChVectorN<double, 12>> {
      public:
        ChElementCableANCF* element;
        ChMatrixNM<double, 4, 3>* d;  // this is an external matrix, use pointer
        ChVectorN<double, 12>* d_dt;  // this is an external matrix, use pointer
        ChMatrixNM<double, 3, 12> Sd;
        ChElementCableANCF::ShapeVector Nd;
        ChMatrixNM<double, 1, 12> strainD;
        ChMatrixNM<double, 1, 3> Nd_d;

        // Evaluate (strainD'*strain)  at point x
        virtual void Evaluate(ChVectorN<double, 12>& result, const double x) override {
            element->ShapeFunctionsDerivatives(Nd, x);

            // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
            Sd.setZero();
            for (int i = 0; i < 4; i++) {
                Sd(0, 3 * i + 0) = Nd(i);
                Sd(1, 3 * i + 1) = Nd(i);
                Sd(2, 3 * i + 2) = Nd(i);
            }

            Nd_d = Nd * (*d);
            strainD = Nd_d * Sd;

            // strain = (Nd*(d*d')*Nd'-1)*0.5;
            double strain = 0.5 * (Nd_d.dot(Nd_d) - 1);

            // Add damping forces if selected
            if (element->m_use_damping)
                strain += (element->m_alpha) * (strainD * (*d_dt))(0, 0);

            result = strainD.transpose() * strain;
        }
    };

    CableANCF_ForceAxial myformulaAx;
    myformulaAx.d = &d;
    myformulaAx.d_dt = &vel_vector;
    myformulaAx.element = this;

    ChVectorN<double, 12> Faxial;
    Faxial.setZero();
    ChQuadrature::Integrate1D<ChVectorN<double, 12>>(Faxial,       // result of integration will go there
                                                     myformulaAx,  // formula to integrate
                                                     0,            // start of x
                                                     1,            // end of x
                                                     5             // order of integration
    );
    Faxial *= -E * Area * length;

    Fi = Faxial;

    // 2)
    // Integrate   (k_e'*k_e)

    class CableANCF_ForceCurv : public ChIntegrable1D<ChVectorN<double, 12>> {
      public:
        ChElementCableANCF* element;
        ChMatrixNM<double, 4, 3>* d;  // this is an external matrix, use pointer
        ChVectorN<double, 12>* d_dt;  // this is an external matrix, use pointer
        ChMatrixNM<double, 3, 12> Sd;
        ChMatrixNM<double, 3, 12> Sdd;
        ChElementCableANCF::ShapeVector Nd;
        ChElementCableANCF::ShapeVector Ndd;
        ChMatrixNM<double, 1, 12> g_e;
        ChMatrixNM<double, 1, 12> f_e;
        ChMatrixNM<double, 1, 12> k_e;
        ChMatrixNM<double, 3, 12> fe1;

        // Evaluate  at point x
        virtual void Evaluate(ChVectorN<double, 12>& result, const double x) override {
            element->ShapeFunctionsDerivatives(Nd, x);
            element->ShapeFunctionsDerivatives2(Ndd, x);

            // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
            // Sdd=[Ndd1*eye(3) Ndd2*eye(3) Ndd3*eye(3) Ndd4*eye(3)]
            Sd.setZero();
            for (int i = 0; i < 4; i++) {
                Sd(0, 3 * i + 0) = Nd(i);
                Sd(1, 3 * i + 1) = Nd(i);
                Sd(2, 3 * i + 2) = Nd(i);
            }

            Sdd.setZero();
            for (int i = 0; i < 4; i++) {
                Sdd(0, 3 * i + 0) = Ndd(i);
                Sdd(1, 3 * i + 1) = Ndd(i);
                Sdd(2, 3 * i + 2) = Ndd(i);
            }

            ChVector<> vr_x((*d).transpose() * Nd.transpose());
            ChVector<> vr_xx((*d).transpose() * Ndd.transpose());
            ChVector<> vf1 = Vcross(vr_x, vr_xx);
            double f = vf1.Length();
            double g1 = vr_x.Length();
            double g = pow(g1, 3);
            double k = f / g;

            g_e = (3 * g1) * Nd * (*d) * Sd;

            // do:  fe1=cross(Sd,r_xxrep)+cross(r_xrep,Sdd);
            for (int col = 0; col < 12; ++col) {
                ChVector<> Sd_i = Sd.col(col);
                fe1.col(col) = Vcross(Sd_i, vr_xx).eigen();
                ChVector<> Sdd_i = Sdd.col(col);
                fe1.col(col) += Vcross(vr_x, Sdd_i).eigen();
            }
            ChVectorN<double, 3> f1 = vf1.eigen();

            if (f == 0)
                f_e = f1.transpose() * fe1;
            else {
                f_e = (1 / f) * f1.transpose() * fe1;
            }

            k_e = (f_e * g - g_e * f) * (1 / (pow(g, 2)));

            // Add damping if selected by user: curvature rate
            if (element->m_use_damping)
                k += (element->m_alpha) * (k_e * (*d_dt))(0, 0);

            result = k * k_e.transpose();
        }
    };

    CableANCF_ForceCurv myformulaCurv;
    myformulaCurv.d = &d;
    myformulaCurv.d_dt = &vel_vector;
    myformulaCurv.element = this;

    ChVectorN<double, 12> Fcurv;
    Fcurv.setZero();
    ChQuadrature::Integrate1D<ChVectorN<double, 12>>(Fcurv,          // result of integration will go there
                                                     myformulaCurv,  // formula to integrate
                                                     0,              // start of x
                                                     1,              // end of x
                                                     3               // order of integration
    );

    // Also subtract contribution of initial configuration
    Fi -= (E * I * length) * Fcurv + m_GenForceVec0;
}

// Compute the generalized force vector due to gravity using the efficient ANCF specific method
void ChElementCableANCF::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    assert(Fg.size() == GetNdofs());

    // Calculate and add the generalized force due to gravity to the generalized internal force vector for the element.
    // The generalized force due to gravity could be computed once prior to the start of the simulation if gravity was
    // assumed constant throughout the entire simulation.  However, this implementation assumes that the acceleration
    // due to gravity, while a constant for the entire system, can change from step to step which could be useful for
    // gravity loaded units tests as an example.  The generalized force due to gravity is calculated in compact matrix
    // form and is pre-mapped to the desired vector format
    Eigen::Map<ChMatrixNM<double, 4, 3>> GravForceCompact(Fg.data(), 4, 3);
    GravForceCompact = m_GravForceScale * G_acc.eigen().transpose();
}

void ChElementCableANCF::EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) {
    ShapeVector N;
    double xi = (eta + 1.0) * 0.5;
    ShapeFunctions(N, xi);  // because ShapeFunctions() works in 0..1 range

    u_displ = VNULL;  //(not needed in ANCF? )
    u_rotaz = VNULL;  //(not needed in ANCF? )
}

void ChElementCableANCF::EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) {
    ChVector<> u_displ;
    ChVector<> u_rotaz;

    double xi = (eta + 1.0) * 0.5;  // because ShapeFunctions() works in 0..1 range
    ShapeVector N;
    ShapeFunctions(N, xi);

    ChVector<> pA = m_nodes[0]->GetPos();
    ChVector<> dA = m_nodes[0]->GetD();
    ChVector<> pB = m_nodes[1]->GetPos();
    ChVector<> dB = m_nodes[1]->GetD();

    point.x() = N(0) * pA.x() + N(1) * dA.x() + N(2) * pB.x() + N(3) * dB.x();
    point.y() = N(0) * pA.y() + N(1) * dA.y() + N(2) * pB.y() + N(3) * dB.y();
    point.z() = N(0) * pA.z() + N(1) * dA.z() + N(2) * pB.z() + N(3) * dB.z();

    ShapeFunctionsDerivatives(N, xi);

    ChVector<> Dx;

    Dx.x() = N(0) * pA.x() + N(1) * dA.x() + N(2) * pB.x() + N(3) * dB.x();
    Dx.y() = N(0) * pA.y() + N(1) * dA.y() + N(2) * pB.y() + N(3) * dB.y();
    Dx.z() = N(0) * pA.z() + N(1) * dA.z() + N(2) * pB.z() + N(3) * dB.z();

    // This element has no torsional dof, so once we have the Dx direction
    // of the line, we must compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization , where we propose a guess direction
    // VECT_Y for the vertical:
    ChMatrix33<> msect;
    Dx.Normalize();
    msect.Set_A_Xdir(Dx, VECT_Y);

    rot = msect.Get_A_quaternion();
}

void ChElementCableANCF::EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) {
    assert(m_section);

    ShapeVector N;
    ShapeVector Nd;
    ShapeVector Ndd;
    // double xi = (eta*2 - 1.0);
    // double xi = (eta + 1.0) / 2.0;

    /* To be completed*/
}

void ChElementCableANCF::EvaluateSectionStrain(const double eta, ChVector<>& StrainV) {
    assert(m_section);

    ShapeVector N;
    ShapeVector Nd;
    ShapeVector Ndd;
    // double xi = (eta*2 - 1.0);
    double xi = (eta + 1.0) / 2.0;

    ShapeFunctions(N, xi);  // Evaluate shape functions
    ShapeFunctionsDerivatives(Nd, xi);
    ShapeFunctionsDerivatives2(Ndd, xi);
    ChVectorDynamic<> mD(GetNdofs());

    GetStateBlock(mD);

    ChMatrixNM<double, 3, 12> Sd;
    Sd.setZero();
    for (int i = 0; i < 4; i++) {
        Sd(0, 3 * i + 0) = Nd(i);
        Sd(1, 3 * i + 1) = Nd(i);
        Sd(2, 3 * i + 2) = Nd(i);
    }

    ChMatrixNM<double, 3, 12> Sdd;
    Sdd.setZero();
    for (int i = 0; i < 4; i++) {
        Sdd(0, 3 * i + 0) = Ndd(i);
        Sdd(1, 3 * i + 1) = Ndd(i);
        Sdd(2, 3 * i + 2) = Ndd(i);
    }

    ChVector<> vr_x(Sd * mD);
    ChVector<> vr_xx(Sdd * mD);
    ChVector<> vf1 = Vcross(vr_x, vr_xx);
    double f = vf1.Length();
    double g1 = vr_x.Length();
    double g = pow(g1, 3);

    StrainV.x() = vr_x.Length2() - 1.0;
    StrainV.y() = f / g;  // Bending strain measure (Gertmayer and Shabana, 2006)
}

// Set structural damping.
void ChElementCableANCF::SetAlphaDamp(double a) {
    m_alpha = a;
    if (std::abs(m_alpha) > 1e-10)
        m_use_damping = true;
}

void ChElementCableANCF::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetD().eigen();
}

// Gets all the DOFs packed in a single vector (speed part)
void ChElementCableANCF::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetD_dt().eigen();
}

// Increment all DOFs using a delta.
void ChElementCableANCF::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 6, x_new, x, off_v + 6, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementCableANCF::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
};

// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface,
// each ranging in -1..+1
// F is a load, N'*F is the resulting generalized load.
// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
void ChElementCableANCF::ComputeNF(const double U,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    ShapeVector N;
    ShapeFunctions(N, (U + 1) * 0.5);  // evaluate shape functions (in compressed vector)

    detJ = GetRestLength() / 2.0;

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(0, 3);
    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
    Qi.segment(9, 3) = N(3) * F.segment(0, 3);
}

// Evaluate N'*F , where N is some type of shape function evaluated at U,V,W coordinates of the volume,
// each ranging in -1..+1
// F is a load, N'*F is the resulting generalized load.
// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
void ChElementCableANCF::ComputeNF(const double U,
                                   const double V,
                                   const double W,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}

}  // end namespace fea
}  // end namespace chrono
