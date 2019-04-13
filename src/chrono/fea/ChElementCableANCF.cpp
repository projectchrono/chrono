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

ChElementCableANCF::ChElementCableANCF() {
    nodes.resize(2);
    m_use_damping = false;  // flag to add internal damping and its Jacobian
    m_alpha = 0.0;          // scaling factor for internal damping

    // this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());
    // this->MassMatrix.Resize(this->GetNdofs(), this->GetNdofs());
}

void ChElementCableANCF::SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA, std::shared_ptr<ChNodeFEAxyzD> nodeB) {
    assert(nodeA);
    assert(nodeB);

    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[0]->Variables_D());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[1]->Variables_D());
    Kmatr.SetVariables(mvars);
}

void ChElementCableANCF::ShapeFunctions(ChMatrix<>& N, double xi) {
    double l = this->GetRestLength();

    N(0) = 1 - 3 * pow(xi, 2) + 2 * pow(xi, 3);
    N(1) = l * (xi - 2 * pow(xi, 2) + pow(xi, 3));
    N(2) = 3 * pow(xi, 2) - 2 * pow(xi, 3);
    N(3) = l * (-pow(xi, 2) + pow(xi, 3));
};

void ChElementCableANCF::ShapeFunctionsDerivatives(ChMatrix<>& Nd, double xi) {
    double l = this->GetRestLength();

    Nd(0) = (6.0 * pow(xi, 2.0) - 6.0 * xi) / l;
    Nd(1) = 1.0 - 4.0 * xi + 3.0 * pow(xi, 2.0);
    Nd(2) = -(6.0 * pow(xi, 2.0) - 6.0 * xi) / l;
    Nd(3) = -2.0 * xi + 3.0 * pow(xi, 2.0);
};

void ChElementCableANCF::ShapeFunctionsDerivatives2(ChMatrix<>& Ndd, double xi) {
    double l = this->GetRestLength();
    Ndd(0) = (12 * xi - 6) / pow(l, 2);
    Ndd(1) = (-4 + 6 * xi) / l;
    Ndd(2) = (6 - 12 * xi) / pow(l, 2);
    Ndd(3) = (-2 + 6 * xi) / l;
};

void ChElementCableANCF::Update() {
    // parent class update:
    ChElementGeneric::Update();
};

void ChElementCableANCF::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(12, 1);

    mD.PasteVector(this->nodes[0]->GetPos(), 0, 0);
    mD.PasteVector(this->nodes[0]->GetD(), 3, 0);
    mD.PasteVector(this->nodes[1]->GetPos(), 6, 0);
    mD.PasteVector(this->nodes[1]->GetD(), 9, 0);
}

// Computes the stiffness matrix of the element:
// K = integral( .... ),
// Note: in this 'basic' implementation, constant section and constant material are assumed.
void ChElementCableANCF::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    assert(section);
    bool use_numerical_differentiation = true;  // Only option tested for now

    // Option: compute the stiffness matrix by doing a numerical differentiation
    // of the internal forces. This fixes a problem with the code by D.Melanz, that
    // produces a rank deficient matrix for straight beams.
    if (use_numerical_differentiation) {
        double diff = 1e-8;
        ChMatrixDynamic<> Kcolumn(12, 1);
        ChMatrixDynamic<> F0(12, 1);
        ChMatrixDynamic<> F1(12, 1);

        this->ComputeInternalForces(F0);

        // Create local copies of the nodal coordinates and use the implementation version
        // of the function for calculating the internal forces.  With this, the calculation
        // of the Jacobian with finite differences is thread safe (otherwise, there would
        // be race conditions when adjacent elements attempt to perturb a common node).
        ChVector<> pos[2] = {this->nodes[0]->pos, this->nodes[1]->pos};
        ChVector<> D[2] = {this->nodes[0]->D, this->nodes[1]->D};
        ChVector<> pos_dt[2] = {this->nodes[0]->pos_dt, this->nodes[1]->pos_dt};
        ChVector<> D_dt[2] = {this->nodes[0]->D_dt, this->nodes[1]->D_dt};

        // Add part of the Jacobian stemming from elastic forces
        for (int inode = 0; inode < 2; ++inode) {
            pos[inode].x() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 0 + inode * 6);
            pos[inode].x() -= diff;

            pos[inode].y() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 1 + inode * 6);
            pos[inode].y() -= diff;

            pos[inode].z() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 2 + inode * 6);
            pos[inode].z() -= diff;

            D[inode].x() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 3 + inode * 6);
            D[inode].x() -= diff;

            D[inode].y() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 4 + inode * 6);
            D[inode].y() -= diff;

            D[inode].z() += diff;
            this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1], F1);
            Kcolumn = (F0 - F1) * (1.0 / diff) * Kfactor;
            this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 5 + inode * 6);
            D[inode].z() -= diff;
        }

        // Add part of the Jacobian stemming from internal damping forces, if selected by user.
        if (m_use_damping) {
            for (int inode = 0; inode < 2; ++inode) {
                pos_dt[inode].x() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 0 + inode * 6);
                pos_dt[inode].x() -= diff;

                pos_dt[inode].y() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 1 + inode * 6);
                pos_dt[inode].y() -= diff;

                pos_dt[inode].z() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 2 + inode * 6);
                pos_dt[inode].z() -= diff;

                D_dt[inode].x() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 3 + inode * 6);
                D_dt[inode].x() -= diff;

                D_dt[inode].y() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 4 + inode * 6);
                D_dt[inode].y() -= diff;

                D_dt[inode].z() += diff;
                this->ComputeInternalForces_Impl(pos[0], D[0], pos[1], D[1], pos_dt[0], D_dt[0], pos_dt[1], D_dt[1],
                                                 F1);
                Kcolumn = (F0 - F1) * (1.0 / diff) * Rfactor;
                this->m_JacobianMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 12, 1, 0, 5 + inode * 6);
                D_dt[inode].z() -= diff;
            }
        }
    }

    else {
        // Option B: use the code in D.Melanz thesis. These formulas, however,
        // produce a rank deficient matrix for straight beams.
        //// TODO: Test it and include internal damping from strain rates.
        double Area = section->Area;
        double E = section->E;
        double I = section->I;

        double l = this->length;

        ChVector<> pA = this->nodes[0]->GetPos();
        ChVector<> dA = this->nodes[0]->GetD();
        ChVector<> pB = this->nodes[1]->GetPos();
        ChVector<> dB = this->nodes[1]->GetD();

        // this matrix will be used in both MyStiffnessAxial and MyStiffnessCurv integrators
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

        class MyStiffnessAxial : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
          public:
            ChElementCableANCF* element;
            ChMatrixNM<double, 4, 3>* d;
            ChMatrixNM<double, 3, 12> Sd;
            ChMatrixNM<double, 1, 4> N;
            ChMatrixNM<double, 1, 4> Nd;
            ChMatrixNM<double, 1, 12> strainD;
            ChMatrixNM<double, 1, 1> strain;
            ChMatrixNM<double, 1, 3> Nd_d;
            ChMatrixNM<double, 12, 12> temp;

            // Evaluate ((strainD'*strainD)+(strain*Sd'*Sd)) at point x
            virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) {
                element->ShapeFunctionsDerivatives(Nd, x);

                // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
                ChMatrix33<> Sdi;
                Sdi.Reset();

                Sdi.FillDiag(Nd(0));
                Sd.PasteMatrix(Sdi, 0, 0);
                Sdi.FillDiag(Nd(1));
                Sd.PasteMatrix(Sdi, 0, 3);
                Sdi.FillDiag(Nd(2));
                Sd.PasteMatrix(Sdi, 0, 6);
                Sdi.FillDiag(Nd(3));
                Sd.PasteMatrix(Sdi, 0, 9);

                Nd_d = Nd * (*d);

                strainD = Nd_d * Sd;

                // strain = (Nd*(d*d')*Nd'-1)*0.5;

                strain.MatrMultiplyT(Nd_d, Nd_d);
                strain(0, 0) += -1;
                strain(0, 0) *= 0.5;  // strain

                // result:  ((strainD'*strainD)+(strain*Sd'*Sd))

                result.MatrTMultiply(strainD, strainD);  //(strainD'*strainD)

                temp.MatrTMultiply(Sd, Sd);  //(strain*Sd'*Sd)
                temp *= strain(0, 0);
                result += temp;
            }
        };

        MyStiffnessAxial myformulaAx;
        myformulaAx.d = &d;
        myformulaAx.element = this;

        ChMatrixNM<double, 12, 12> Kaxial;
        Kaxial.Reset();
        ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(Kaxial,       // result of integration will go there
                                                              myformulaAx,  // formula to integrate
                                                              0,            // start of x
                                                              1,            // end of x
                                                              5             // order of integration
        );
        Kaxial *= E * Area * length;

        this->m_JacobianMatrix = Kaxial;

        // 2)
        // Integrate   (k_e'*k_e)

        class MyStiffnessCurv : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
          public:
            ChElementCableANCF* element;
            ChMatrixNM<double, 4, 3>* d;
            ChMatrixNM<double, 3, 12> Sd;
            ChMatrixNM<double, 3, 12> Sdd;
            ChMatrixNM<double, 1, 4> Nd;
            ChMatrixNM<double, 1, 4> Ndd;

            ChMatrixNM<double, 1, 3> r_x;
            ChMatrixNM<double, 1, 3> r_xx;

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
                ChMatrix33<> Sdi;
                Sdi.Reset();

                Sdi.FillDiag(Nd(0));
                Sd.PasteMatrix(Sdi, 0, 0);
                Sdi.FillDiag(Nd(1));
                Sd.PasteMatrix(Sdi, 0, 3);
                Sdi.FillDiag(Nd(2));
                Sd.PasteMatrix(Sdi, 0, 6);
                Sdi.FillDiag(Nd(3));
                Sd.PasteMatrix(Sdi, 0, 9);

                Sdi.FillDiag(Ndd(0));
                Sdd.PasteMatrix(Sdi, 0, 0);
                Sdi.FillDiag(Ndd(1));
                Sdd.PasteMatrix(Sdi, 0, 3);
                Sdi.FillDiag(Ndd(2));
                Sdd.PasteMatrix(Sdi, 0, 6);
                Sdi.FillDiag(Ndd(3));
                Sdd.PasteMatrix(Sdi, 0, 9);

                r_x.MatrMultiply(Nd, *d);    // r_x=d'*Nd';  (transposed)
                r_xx.MatrMultiply(Ndd, *d);  // r_xx=d'*Ndd';  (transposed)

                // if (r_xx.Length()==0)
                //     {r_xx(0)=0; r_xx(1)=1; r_xx(2)=0;}
                ChVector<> vr_x(r_x(0), r_x(1), r_x(2));
                ChVector<> vr_xx(r_xx(0), r_xx(1), r_xx(2));
                ChVector<> vf1 = Vcross(vr_x, vr_xx);
                double f = vf1.Length();
                double g1 = vr_x.Length();
                double g = pow(g1, 3);
                double k = f / g;

                g_e = (Nd * (*d)) * Sd;
                g_e *= (3 * g1);

                // do:  fe1=cross(Sd,r_xxrep)+cross(r_xrep,Sdd);
                for (int col = 0; col < 12; ++col) {
                    ChVector<> Sd_i = Sd.ClipVector(0, col);
                    fe1.PasteVector(Vcross(Sd_i, vr_xx), 0, col);
                    ChVector<> Sdd_i = Sdd.ClipVector(0, col);
                    fe1.PasteSumVector(Vcross(vr_x, Sdd_i), 0, col);
                }
                ChMatrixNM<double, 3, 1> f1;
                f1.PasteVector(vf1, 0, 0);

                if (f == 0)
                    f_e.MatrTMultiply(f1, fe1);
                else {
                    f_e.MatrTMultiply(f1, fe1);
                    f_e *= (1 / f);
                }

                k_e = (f_e * g - g_e * f) * (1 / (pow(g, 2)));

                // result:  k_e'*k_e
                result.MatrTMultiply(k_e, k_e);
            }
        };

        MyStiffnessCurv myformulaCurv;
        myformulaCurv.d = &d;
        myformulaCurv.element = this;

        ChMatrixNM<double, 12, 12> Kcurv;
        Kcurv.Reset();
        ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(Kcurv,          // result of integration will go there
                                                              myformulaCurv,  // formula to integrate
                                                              0,              // start of x
                                                              1,              // end of x
                                                              3               // order of integration
        );
        Kcurv *= E * I * length;  // note Iyy should be the same value (circular section assumption)

        this->m_JacobianMatrix += Kcurv;
    }

    //***DEBUG***
    /*
    GetLog() << "Stiffness matr file dump. L=" << this->length << " A=" << this->section->Area << " E=" <<
    this->section->E << " I=" << this->section->Izz << "\n";
    GetLog() << this->StiffnessMatrix;
    ChStreamOutAsciiFile mdump("dump_stiff.txt");
    this->StiffnessMatrix.StreamOUTdenseMatlabFormat(mdump);
    */
}

// Computes the mass matrix of the element.
// Note: in this 'basic' implementation, constant section and constant material are assumed.
void ChElementCableANCF::ComputeMassMatrix() {
    assert(section);

    double Area = section->Area;
    double rho = section->density;

    // Integrate  Area*rho*(S'*S)
    // where S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]

    class MyMass : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
      public:
        ChElementCableANCF* element;
        ChMatrixNM<double, 3, 12> S;
        ChMatrixNM<double, 1, 4> N;

        // Evaluate the S'*S  at point x
        virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) {
            element->ShapeFunctions(N, x);
            // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]
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
            // perform  r = S'*S
            result.MatrTMultiply(S, S);
        }
    };

    MyMass myformula;
    myformula.element = this;
    m_MassMatrix.Reset();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(m_MassMatrix,  // result of integration will go there
                                                          myformula,     // formula to integrate
                                                          0,             // start of x
                                                          1,             // end of x
                                                          4              // order of integration
    );

    m_MassMatrix *= (rho * Area * this->length);
}

// Setup: precompute mass and matrices that do not change during the simulation.
void ChElementCableANCF::SetupInitial(ChSystem* system) {
    assert(section);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    this->mass = this->length * this->section->Area * this->section->density;

    // Here we calculate the internal forces in the initial configuration
    // Contribution of initial configuration in elastic forces is automatically subtracted
    ChMatrixDynamic<> FVector0(12, 1);
    FVector0.Reset();
    m_GenForceVec0.Reset();  // Note: this is important here (m_GenForceVec0 used in ComputeInternalForces)
    ComputeInternalForces(FVector0);
    m_GenForceVec0 = FVector0;

    // Compute mass matrix
    ComputeMassMatrix();
}

// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also superimposes global
// damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
void ChElementCableANCF::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 12) && (H.GetColumns() == 12));
    assert(section);

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 12; i++)
        for (int j = 0; j < 12; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Computes the internal forces and set values in the Fi vector.
// (e.g. the actual position of nodes is not in relaxed reference position).
void ChElementCableANCF::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    ComputeInternalForces_Impl(this->nodes[0]->GetPos(), this->nodes[0]->GetD(), this->nodes[1]->GetPos(),
                               this->nodes[1]->GetD(), this->nodes[0]->GetPos_dt(), this->nodes[0]->GetD_dt(),
                               this->nodes[1]->GetPos_dt(), this->nodes[1]->GetD_dt(), Fi);
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
                                                    ChMatrixDynamic<>& Fi) {
    assert((Fi.GetRows() == 12) && (Fi.GetColumns() == 1));
    assert(section);

    double Area = section->Area;
    double E = section->E;
    double I = section->I;

    double l = this->length;

    // this matrix will be used in both MyForcesAxial and MyForcesCurv integrators
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

    // this matrix will be used in both MyForcesAxial and MyForcesCurv integrators
    ChMatrixNM<double, 12, 1> vel_vector;
    vel_vector(0, 0) = pA_dt.x();
    vel_vector(1, 0) = pA_dt.y();
    vel_vector(2, 0) = pA_dt.z();
    vel_vector(3, 0) = dA_dt.x();
    vel_vector(4, 0) = dA_dt.y();
    vel_vector(5, 0) = dA_dt.z();
    vel_vector(6, 0) = pB_dt.x();
    vel_vector(7, 0) = pB_dt.y();
    vel_vector(8, 0) = pB_dt.z();
    vel_vector(9, 0) = dB_dt.x();
    vel_vector(10, 0) = dB_dt.y();
    vel_vector(11, 0) = dB_dt.z();

    // 1)
    // Integrate   (strainD'*strain)

    class MyForcesAxial : public ChIntegrable1D<ChMatrixNM<double, 12, 1>> {
      public:
        ChElementCableANCF* element;
        ChMatrixNM<double, 4, 3>* d;      // this is an external matrix, use pointer
        ChMatrixNM<double, 12, 1>* d_dt;  // this is an external matrix, use pointer
        ChMatrixNM<double, 3, 12> Sd;
        ChMatrixNM<double, 1, 4> N;
        ChMatrixNM<double, 1, 4> Nd;
        ChMatrixNM<double, 1, 12> strainD;
        ChMatrixNM<double, 1, 1> strain;
        ChMatrixNM<double, 1, 3> Nd_d;
        ChMatrixNM<double, 12, 12> temp;

        // Evaluate (strainD'*strain)  at point x
        virtual void Evaluate(ChMatrixNM<double, 12, 1>& result, const double x) {
            element->ShapeFunctionsDerivatives(Nd, x);

            // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
            ChMatrix33<> Sdi;
            Sdi.Reset();

            Sdi.FillDiag(Nd(0));
            Sd.PasteMatrix(Sdi, 0, 0);
            Sdi.FillDiag(Nd(1));
            Sd.PasteMatrix(Sdi, 0, 3);
            Sdi.FillDiag(Nd(2));
            Sd.PasteMatrix(Sdi, 0, 6);
            Sdi.FillDiag(Nd(3));
            Sd.PasteMatrix(Sdi, 0, 9);

            Nd_d = Nd * (*d);

            strainD = Nd_d * Sd;

            // strain = (Nd*(d*d')*Nd'-1)*0.5;

            strain.MatrMultiplyT(Nd_d, Nd_d);
            strain(0, 0) += -1;
            strain(0, 0) *= 0.5;

            // Add damping forces if selected
            if (element->m_use_damping)
                strain(0, 0) += (element->m_alpha) * (strainD * (*d_dt))(0, 0);

            result.MatrTMultiply(strainD, strain);
            // result:  strainD'*strain
        }
    };

    MyForcesAxial myformulaAx;
    myformulaAx.d = &d;
    myformulaAx.d_dt = &vel_vector;
    myformulaAx.element = this;

    ChMatrixNM<double, 12, 1> Faxial;
    Faxial.Reset();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 1>>(Faxial,       // result of integration will go there
                                                         myformulaAx,  // formula to integrate
                                                         0,            // start of x
                                                         1,            // end of x
                                                         5             // order of integration
    );
    Faxial *= -E * Area * length;

    Fi = Faxial;

    // 2)
    // Integrate   (k_e'*k_e)

    class MyForcesCurv : public ChIntegrable1D<ChMatrixNM<double, 12, 1>> {
      public:
        ChElementCableANCF* element;
        ChMatrixNM<double, 4, 3>* d;      // this is an external matrix, use pointer
        ChMatrixNM<double, 12, 1>* d_dt;  // this is an external matrix, use pointer
        ChMatrixNM<double, 3, 12> Sd;
        ChMatrixNM<double, 3, 12> Sdd;
        ChMatrixNM<double, 1, 4> Nd;
        ChMatrixNM<double, 1, 4> Ndd;
        ChMatrixNM<double, 1, 3> r_x;
        ChMatrixNM<double, 1, 3> r_xx;
        ChMatrixNM<double, 1, 12> g_e;
        ChMatrixNM<double, 1, 12> f_e;
        ChMatrixNM<double, 1, 12> k_e;
        ChMatrixNM<double, 3, 12> fe1;

        // Evaluate  at point x
        virtual void Evaluate(ChMatrixNM<double, 12, 1>& result, const double x) {
            element->ShapeFunctionsDerivatives(Nd, x);
            element->ShapeFunctionsDerivatives2(Ndd, x);

            // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
            // Sdd=[Ndd1*eye(3) Ndd2*eye(3) Ndd3*eye(3) Ndd4*eye(3)]
            ChMatrix33<> Sdi;
            Sdi.Reset();

            Sdi.FillDiag(Nd(0));
            Sd.PasteMatrix(Sdi, 0, 0);
            Sdi.FillDiag(Nd(1));
            Sd.PasteMatrix(Sdi, 0, 3);
            Sdi.FillDiag(Nd(2));
            Sd.PasteMatrix(Sdi, 0, 6);
            Sdi.FillDiag(Nd(3));
            Sd.PasteMatrix(Sdi, 0, 9);

            Sdi.FillDiag(Ndd(0));
            Sdd.PasteMatrix(Sdi, 0, 0);
            Sdi.FillDiag(Ndd(1));
            Sdd.PasteMatrix(Sdi, 0, 3);
            Sdi.FillDiag(Ndd(2));
            Sdd.PasteMatrix(Sdi, 0, 6);
            Sdi.FillDiag(Ndd(3));
            Sdd.PasteMatrix(Sdi, 0, 9);

            r_x.MatrMultiply(Nd, (*d));    // r_x=d'*Nd';  (transposed)
            r_xx.MatrMultiply(Ndd, (*d));  // r_xx=d'*Ndd';  (transposed)

            // if (r_xx.Length()==0)
            //     {r_xx(0)=0; r_xx(1)=1; r_xx(2)=0;}
            ChVector<> vr_x(r_x(0), r_x(1), r_x(2));
            ChVector<> vr_xx(r_xx(0), r_xx(1), r_xx(2));
            ChVector<> vf1 = Vcross(vr_x, vr_xx);
            double f = vf1.Length();
            double g1 = vr_x.Length();
            double g = pow(g1, 3);
            double k = f / g;
            g_e = (Nd * (*d)) * Sd;
            g_e *= (3 * g1);

            // do:  fe1=cross(Sd,r_xxrep)+cross(r_xrep,Sdd);
            for (int col = 0; col < 12; ++col) {
                ChVector<> Sd_i = Sd.ClipVector(0, col);
                fe1.PasteVector(Vcross(Sd_i, vr_xx), 0, col);
                ChVector<> Sdd_i = Sdd.ClipVector(0, col);
                fe1.PasteSumVector(Vcross(vr_x, Sdd_i), 0, col);
            }
            ChMatrixNM<double, 3, 1> f1;
            f1.PasteVector(vf1, 0, 0);

            if (f == 0)
                f_e.MatrTMultiply(f1, fe1);
            else {
                f_e.MatrTMultiply(f1, fe1);
                f_e *= (1 / f);
            }

            k_e = (f_e * g - g_e * f) * (1 / (pow(g, 2)));

            // result:  k_e'*k
            result.CopyFromMatrixT(k_e);

            // Add damping if selected by user: curvature rate
            if (element->m_use_damping)
                k += (element->m_alpha) * (k_e * (*d_dt))(0, 0);

            result *= k;
        }
    };

    MyForcesCurv myformulaCurv;
    myformulaCurv.d = &d;
    myformulaCurv.d_dt = &vel_vector;
    myformulaCurv.element = this;

    ChMatrixNM<double, 12, 1> Fcurv;
    Fcurv.Reset();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 1>>(Fcurv,          // result of integration will go there
                                                         myformulaCurv,  // formula to integrate
                                                         0,              // start of x
                                                         1,              // end of x
                                                         3               // order of integration
    );
    Fcurv *= -E * I * length;  // note Iyy should be the same value (circular section assumption)

    Fi += Fcurv;

    // Substract contribution of initial configuration
    Fi -= this->m_GenForceVec0;
}

void ChElementCableANCF::EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) {
    ChMatrixNM<double, 1, 4> N;

    double xi = (eta + 1.0) * 0.5;

    this->ShapeFunctions(N, xi);  // because ShapeFunctions() works in 0..1 range

    u_displ = VNULL;  //(not needed in ANCF? )
    u_rotaz = VNULL;  //(not needed in ANCF? )
}

void ChElementCableANCF::EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) {
    ChVector<> u_displ;
    ChVector<> u_rotaz;

    ChMatrixNM<double, 1, 4> N;

    double xi = (eta + 1.0) * 0.5;  // because ShapeFunctions() works in 0..1 range

    this->ShapeFunctions(N, xi);

    ChVector<> pA = this->nodes[0]->GetPos();
    ChVector<> dA = this->nodes[0]->GetD();
    ChVector<> pB = this->nodes[1]->GetPos();
    ChVector<> dB = this->nodes[1]->GetD();

    point.x() = N(0) * pA.x() + N(1) * dA.x() + N(2) * pB.x() + N(3) * dB.x();
    point.y() = N(0) * pA.y() + N(1) * dA.y() + N(2) * pB.y() + N(3) * dB.y();
    point.z() = N(0) * pA.z() + N(1) * dA.z() + N(2) * pB.z() + N(3) * dB.z();

    this->ShapeFunctionsDerivatives(N, xi);

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
    assert(section);

    ChMatrixNM<double, 1, 4> N;
    ChMatrixNM<double, 1, 4> Nd;
    ChMatrixNM<double, 1, 4> Ndd;
    // double xi = (eta*2 - 1.0);
    double xi = (eta + 1.0) / 2.0;

    /* To be completed*/
}

void ChElementCableANCF::EvaluateSectionStrain(const double eta, ChVector<>& StrainV) {
    assert(section);

    ChMatrixNM<double, 1, 4> N;
    ChMatrixNM<double, 1, 4> Nd;
    ChMatrixNM<double, 1, 4> Ndd;
    // double xi = (eta*2 - 1.0);
    double xi = (eta + 1.0) / 2.0;

    this->ShapeFunctions(N, xi);  // Evaluate shape functions
    this->ShapeFunctionsDerivatives(Nd, xi);
    this->ShapeFunctionsDerivatives2(Ndd, xi);
    ChMatrixDynamic<> mD(GetNdofs(), 1);
    ChMatrixNM<double, 3, 12> Sd;
    ChMatrixNM<double, 3, 12> Sdd;
    ChMatrixNM<double, 3, 1> r_x;
    ChMatrixNM<double, 3, 1> r_xx;

    this->GetStateBlock(mD);

    ChMatrix33<> Sdi;
    Sdi.Reset();

    Sdi.FillDiag(Nd(0));
    Sd.PasteMatrix(Sdi, 0, 0);
    Sdi.FillDiag(Nd(1));
    Sd.PasteMatrix(Sdi, 0, 3);
    Sdi.FillDiag(Nd(2));
    Sd.PasteMatrix(Sdi, 0, 6);
    Sdi.FillDiag(Nd(3));
    Sd.PasteMatrix(Sdi, 0, 9);
    Sdi.Reset();
    Sdi.FillDiag(Ndd(0));
    Sdd.PasteMatrix(Sdi, 0, 0);
    Sdi.FillDiag(Ndd(1));
    Sdd.PasteMatrix(Sdi, 0, 3);
    Sdi.FillDiag(Ndd(2));
    Sdd.PasteMatrix(Sdi, 0, 6);
    Sdi.FillDiag(Ndd(3));
    Sdd.PasteMatrix(Sdi, 0, 9);

    r_x.MatrMultiply(Sd, mD);  // r_x=d'*Nd';  (transposed)
    r_xx.MatrMultiply(Sdd, mD);

    ChVector<> vr_x(r_x(0), r_x(1), r_x(2));
    ChVector<> vr_xx(r_xx(0), r_xx(1), r_xx(2));
    ChVector<> vf1 = Vcross(vr_x, vr_xx);
    double f = vf1.Length();
    double g1 = vr_x.Length();
    double g = pow(g1, 3);

    StrainV.x() = (pow(r_x(0), 2) + pow(r_x(1), 2) + pow(r_x(2), 2) - 1.0);
    StrainV.y() = f / g;  // Bending strain measure (Gertmayer and Shabana, 2006)
}

// Set structural damping.
void ChElementCableANCF::SetAlphaDamp(double a) {
    m_alpha = a;
    if (std::abs(m_alpha) > 1e-10)
        m_use_damping = true;
}

void ChElementCableANCF::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.PasteVector(this->nodes[0]->GetPos(), block_offset, 0);
    mD.PasteVector(this->nodes[0]->GetD(), block_offset + 3, 0);
    mD.PasteVector(this->nodes[1]->GetPos(), block_offset + 6, 0);
    mD.PasteVector(this->nodes[1]->GetD(), block_offset + 9, 0);
}

// Gets all the DOFs packed in a single vector (speed part)
void ChElementCableANCF::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.PasteVector(this->nodes[0]->GetPos_dt(), block_offset, 0);
    mD.PasteVector(this->nodes[0]->GetD_dt(), block_offset + 3, 0);
    mD.PasteVector(this->nodes[1]->GetPos_dt(), block_offset + 6, 0);
    mD.PasteVector(this->nodes[1]->GetD_dt(), block_offset + 9, 0);
}

// Increment all DOFs using a delta.
void ChElementCableANCF::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    nodes[1]->NodeIntStateIncrement(off_x + 6, x_new, x, off_v + 6, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementCableANCF::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&this->nodes[0]->Variables());
    mvars.push_back(&this->nodes[0]->Variables_D());
    mvars.push_back(&this->nodes[1]->Variables());
    mvars.push_back(&this->nodes[1]->Variables_D());
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
    ChMatrixNM<double, 1, 4> N;
    this->ShapeFunctions(
        N, (U + 1) * 0.5);  // evaluate shape functions (in compressed vector), btw. not dependant on state

    detJ = this->GetRestLength() / 2.0;

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 9, 0);
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
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}

}  // end namespace fea
}  // end namespace chrono
