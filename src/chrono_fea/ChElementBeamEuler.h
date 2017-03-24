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

#ifndef CHELEMENTBEAMEULER_H
#define CHELEMENTBEAMEULER_H

//#define BEAM_VERBOSE

#include "chrono_fea/ChElementBeam.h"
#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono_fea/ChElementCorotational.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Simple beam element with two nodes and Euler-Bernoulli formulation.
/// For this 'basic' implementation, constant section and constant
/// material are assumed.
///
/// Further information in the
/// [white paper PDF](http://www.projectchrono.org/assets/white_papers/euler_beams.pdf)
///
/// Note that there are also ChElementCableANCF if no torsional effects
/// are needed, as in cables.

class ChApiFea ChElementBeamEuler : public ChElementBeam,
                                    public ChLoadableU,
                                    public ChLoadableUVW,
                                    public ChElementCorotational {
  protected:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > nodes;

    std::shared_ptr<ChBeamSectionAdvanced> section;

    ChMatrixDynamic<> StiffnessMatrix;  // undeformed local stiffness matrix

    ChQuaternion<> q_refrotA;
    ChQuaternion<> q_refrotB;

    ChQuaternion<> q_element_abs_rot;
    ChQuaternion<> q_element_ref_rot;

    bool disable_corotate;
    bool disable_projector;
    bool force_symmetric_stiffness;

  public:
    ChElementBeamEuler() {
        nodes.resize(2);

        this->StiffnessMatrix.Resize(this->GetNdofs(), this->GetNdofs());

        q_refrotA = QUNIT;
        q_refrotB = QUNIT;
        q_element_abs_rot = QUNIT;
        q_element_ref_rot = QUNIT;
        disable_corotate = false;
        disable_projector = true;  //***TO DO*** see why projectors work worse... ex. see lateral buckling test
        force_symmetric_stiffness = false;
    }

    virtual ~ChElementBeamEuler() {}

    virtual int GetNnodes() override { return 2; }
    virtual int GetNdofs() override { return 2 * 6; }
    virtual int GetNodeNdofs(int n) override { return 6; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNodeN(int n) override { return nodes[n]; }

    virtual void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB) {
        assert(nodeA);
        assert(nodeB);

        nodes[0] = nodeA;
        nodes[1] = nodeB;
        std::vector<ChVariables*> mvars;
        mvars.push_back(&nodes[0]->Variables());
        mvars.push_back(&nodes[1]->Variables());
        Kmatr.SetVariables(mvars);
    }

    //
    // FEM functions
    //

    /// Set the section & material of beam element .
    /// It is a shared property, so it can be shared between other beams.
    void SetSection(std::shared_ptr<ChBeamSectionAdvanced> my_material) { section = my_material; }
    /// Get the section & material of the element
    std::shared_ptr<ChBeamSectionAdvanced> GetSection() { return section; }

    /// Get the first node (beginning)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() { return nodes[0]; }

    /// Get the second node (ending)
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() { return nodes[1]; }

    /// Set the reference rotation of nodeA respect to the element rotation.
    void SetNodeAreferenceRot(ChQuaternion<> mrot) { q_refrotA = mrot; }
    ChQuaternion<> GetNodeAreferenceRot() { return q_refrotA; }

    /// Set the reference rotation of nodeB respect to the element rotation.
    void SetNodeBreferenceRot(ChQuaternion<> mrot) { q_refrotB = mrot; }
    ChQuaternion<> GetNodeBreferenceRot() { return q_refrotB; }

    /// Get the absolute rotation of element in space
    /// This is not the same of Rotation() , that expresses
    /// the accumulated rotation from starting point.
    ChQuaternion<> GetAbsoluteRotation() { return q_element_abs_rot; }

    /// Get the original reference rotation of element in space
    ChQuaternion<> GetRefRotation() { return q_element_ref_rot; }

    /// Set this as true to have the beam behave like a non-corotated beam
    /// (i.e. only for small deformations).
    void SetDisableCorotate(bool md) { disable_corotate = md; }

    /// Set this as true to disable the projectors for computing the
    /// tangent matrix in the corotational formulation
    /// (see C.A.Felippa, B.Haugen, N.Omid, C.Rankin et al. for details).
    /// Disabling the projectors leads to a faster code, but convergence might be more difficult.
    void SetDisableProjector(bool md) { disable_projector = md; }

    /// Set this as true to force the tangent stiffness matrix to be
    /// inexact, but symmetric. This allows the use of faster solvers. For systems close to
    /// the equilibrium, the tangent stiffness would be symmetric anyway.
    void SetForceSymmetricStiffness(bool md) { force_symmetric_stiffness = md; }

    /// Fills the N matrix (compressed! single row, 12 columns) with the
    /// values of shape functions at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Given  u = 12-d state block {u1,r1,u2,r2}' , d = 6-d field {u,r},
    /// one has   f(eta) = [S(eta)]*u   where one fills the sparse [S] matrix
    /// with the N shape functions in this pattern:
    ///      | 0    .   .   .   .   .   3   .   .   .   .   .   |
    ///      | .    1   .   .   .   2   .   4   .   .   .   5   |
    /// [S] =| .    .   1   .   -2  .   .   .   4   .   -5  .   |
    ///      | .    .   .   0   .   .   .   .   .   3   .   .   |
    ///      | .    .   -6  .   8   .   .   .   -7  .   9   .   |
    ///      | .    6   .   .   .   8   .   7   .   .   .   9   |
    virtual void ShapeFunctions(ChMatrix<>& N, double eta) {
        double Nx1 = (1. / 2.) * (1 - eta);
        double Nx2 = (1. / 2.) * (1 + eta);
        double Ny1 = (1. / 4.) * pow((1 - eta), 2) * (2 + eta);
        double Ny2 = (1. / 4.) * pow((1 + eta), 2) * (2 - eta);
        double Nr1 = (this->length / 8.) * pow((1 - eta), 2) * (1 + eta);
        double Nr2 = (this->length / 8.) * pow((1 + eta), 2) * (eta - 1);
        /*
        N(0) = Nx1;
        N(1) = Ny1;
        N(2) = Ny1;
        N(3) = Nx1;
        N(4) = -Nr1;
        N(5) = Nr1;
        N(6) = Nx2;
        N(7) = Ny2;
        N(8) = Ny2;
        N(9) = Nx2;
        N(10) = -Nr2;
        N(11) = Nr2;
        */
        double dN_ua = (1. / (2. * this->length)) * (-3. + 3 * eta * eta);
        double dN_ub = (1. / (2. * this->length)) * (3. - 3 * eta * eta);
        double dN_ra = (1. / 4.) * (-1. - 2 * eta + 3 * eta * eta);
        double dN_rb = -(1. / 4.) * (1. - 2 * eta - 3 * eta * eta);
        N(0) = Nx1;
        N(1) = Ny1;
        N(2) = Nr1;
        N(3) = Nx2;
        N(4) = Ny2;
        N(5) = Nr2;
        N(6) = dN_ua;
        N(7) = dN_ub;
        N(8) = dN_ra;
        N(9) = dN_rb;
    };

    virtual void Update() override {
        // parent class update:
        ChElementGeneric::Update();

        // always keep updated the rotation matrix A:
        this->UpdateRotation();
    };

    /// Compute large rotation of element for corotational approach
    /// The reference frame of this Euler-Bernoulli beam has X aligned to two nodes and Y parallel to Y of 1st node
    virtual void UpdateRotation() override {
        ChMatrix33<> A0(this->q_element_ref_rot);

        ChMatrix33<> Aabs;
        if (this->disable_corotate) {
            Aabs = A0;
            q_element_abs_rot = q_element_ref_rot;
        } else {
            ChVector<> mXele_w = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
            // propose Y_w as absolute dir of the Y axis of A node, removing the effect of Aref-to-A rotation if any:
            //    Y_w = [R Aref->w ]*[R Aref->A ]'*{0,1,0}
            ChVector<> myele_wA = nodes[0]->Frame().GetRot().Rotate(q_refrotA.RotateBack(ChVector<>(0, 1, 0)));
            // propose Y_w as absolute dir of the Y axis of B node, removing the effect of Bref-to-B rotation if any:
            //    Y_w = [R Bref->w ]*[R Bref->B ]'*{0,1,0}
            ChVector<> myele_wB = nodes[1]->Frame().GetRot().Rotate(q_refrotB.RotateBack(ChVector<>(0, 1, 0)));
            // Average the two Y directions to have midpoint torsion (ex -30° torsion A and +30° torsion B= 0°)
            ChVector<> myele_w = (myele_wA + myele_wB).GetNormalized();
            Aabs.Set_A_Xdir(mXele_w, myele_w);
            q_element_abs_rot = Aabs.Get_A_quaternion();
        }

        this->A.MatrTMultiply(A0, Aabs);
    }

    /// Fills the D vector (column matrix) with the current
    /// field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Give that this element includes rotations at nodes, this gives:
    ///  {x_a y_a z_a Rx_a Ry_a Rz_a x_b y_b z_b Rx_b Ry_b Rz_b}
    virtual void GetStateBlock(ChMatrixDynamic<>& mD) override {
        mD.Reset(12, 1);

        ChVector<> delta_rot_dir;
        double delta_rot_angle;

        // Node 0, displacement (in local element frame, corotated back)
        //     d = [Atw]' Xt - [A0w]'X0
        ChVector<> displ = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos()) -
                           this->q_element_ref_rot.RotateBack(nodes[0]->GetX0().GetPos());
        mD.PasteVector(displ, 0, 0);

        // Node 0, x,y,z small rotations (in local element frame)
        ChQuaternion<> q_delta0 =
            q_element_abs_rot.GetConjugate() % nodes[0]->Frame().GetRot() % q_refrotA.GetConjugate();
        // note, for small incremental rotations this is opposite of ChNodeFEAxyzrot::VariablesQbIncrementPosition
        q_delta0.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);

        if (delta_rot_angle > CH_C_PI)
            delta_rot_angle -= CH_C_2PI;  // no 0..360 range, use -180..+180

        mD.PasteVector(delta_rot_dir * delta_rot_angle, 3, 0);

        // Node 1, displacement (in local element frame, corotated back)
        //     d = [Atw]' Xt - [A0w]'X0
        displ = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos()) -
                this->q_element_ref_rot.RotateBack(nodes[1]->GetX0().GetPos());
        mD.PasteVector(displ, 6, 0);

        // Node 1, x,y,z small rotations (in local element frame)
        ChQuaternion<> q_delta1 =
            q_element_abs_rot.GetConjugate() % nodes[1]->Frame().GetRot() % q_refrotB.GetConjugate();
        // note, for small incremental rotations this is opposite of ChNodeFEAxyzrot::VariablesQbIncrementPosition
        q_delta1.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);

        if (delta_rot_angle > CH_C_PI)
            delta_rot_angle -= CH_C_2PI;  // no 0..360 range, use -180..+180

        mD.PasteVector(delta_rot_dir * delta_rot_angle, 9, 0);
    }

    /// Fills the Ddt vector (column matrix) with the current
    /// tme derivatives of field values at the nodes of the element, with proper ordering.
    /// If the D vector has not the size of this->GetNdofs(), it will be resized.
    /// For corotational elements, field is assumed in local reference!
    /// Give that this element includes rotations at nodes, this gives:
    ///  {v_a v_a v_a wx_a wy_a wz_a v_b v_b v_b wx_b wy_b wz_b}
    virtual void GetField_dt(ChMatrixDynamic<>& mD_dt) {
        mD_dt.Reset(12, 1);

        // Node 0, velocity (in local element frame, corotated back by A' )
        mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos_dt()), 0, 0);

        // Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
        mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetWvel_par()), 3, 0);

        // Node 1, velocity (in local element frame, corotated back by A' )
        mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos_dt()), 6, 0);

        // Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
        mD_dt.PasteVector(this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetWvel_par()), 9, 0);
    }

    /// Computes the local STIFFNESS MATRIX of the element:
    /// K = integral( [B]' * [D] * [B] ),
    /// Note: in this 'basic' implementation, constant section and
    /// constant material are assumed, so the explicit result of quadrature is used.
    virtual void ComputeStiffnessMatrix() {
        assert(section);

        double Area = section->Area;
        double E = section->E;
        double Izz = section->Izz;
        double Iyy = section->Iyy;
        double G = section->G;

        double h = this->length;
        double Jpolar = section->J;

        double om_xz = 0;  // For Euler-Bernoulli
        double om_xy = 0;  // For Euler-Bernoulli
        if (false) {
            //***TEST REDDY BEAMS***
            double Ks_z = section->Ks_z;
            double Ks_y = section->Ks_y;
            double om_xz = E * Iyy / (G * Area * Ks_z * h);  // For Reddy's RBT
            double om_xy = E * Izz / (G * Area * Ks_y * h);  // For Reddy's RBT
        }
        double u_xz = 1 + 12 * om_xz;
        double l_xz = 1 + 3 * om_xz;
        double e_xz = 1 - 6 * om_xz;
        double u_xy = 1 + 12 * om_xy;
        double l_xy = 1 + 3 * om_xy;
        double e_xy = 1 - 6 * om_xy;

        double k_u = E * Area / h;
        double k_f = G * Jpolar / h;

        double k_w = 12 * E * Iyy / (u_xz * h * h * h);
        double k_v = 12 * E * Izz / (u_xy * h * h * h);

        double k_t = 4 * E * Iyy * l_xz / (u_xz * h);
        double k_p = 4 * E * Izz * l_xy / (u_xy * h);

        double k_wt = 6 * E * Iyy / (u_xz * h * h);
        double k_vp = 6 * E * Izz / (u_xy * h * h);

        double k_tt = 2 * E * Iyy * e_xz / (u_xz * h);
        double k_pp = 2 * E * Izz * e_xy / (u_xy * h);

        StiffnessMatrix(0, 0) = k_u;
        StiffnessMatrix(1, 1) = k_v;
        StiffnessMatrix(2, 2) = k_w;
        StiffnessMatrix(3, 3) = k_f;
        StiffnessMatrix(4, 4) = k_t;
        StiffnessMatrix(5, 5) = k_p;
        StiffnessMatrix(6, 6) = k_u;
        StiffnessMatrix(7, 7) = k_v;
        StiffnessMatrix(8, 8) = k_w;
        StiffnessMatrix(9, 9) = k_f;
        StiffnessMatrix(10, 10) = k_t;
        StiffnessMatrix(11, 11) = k_p;

        StiffnessMatrix(0, 6) = -k_u;
        StiffnessMatrix(1, 7) = -k_v;
        StiffnessMatrix(2, 8) = -k_w;
        StiffnessMatrix(3, 9) = -k_f;
        StiffnessMatrix(4, 10) = k_tt;
        StiffnessMatrix(5, 11) = k_pp;

        StiffnessMatrix(4, 8) = k_wt;
        StiffnessMatrix(5, 7) = -k_vp;
        StiffnessMatrix(1, 11) = k_vp;
        StiffnessMatrix(2, 10) = -k_wt;

        StiffnessMatrix(1, 5) = k_vp;
        StiffnessMatrix(2, 4) = -k_wt;
        StiffnessMatrix(7, 11) = -k_vp;
        StiffnessMatrix(8, 10) = k_wt;

        // symmetric part;
        for (int r = 0; r < 12; r++)
            for (int c = r + 1; c < 12; c++)
                StiffnessMatrix(c, r) = StiffnessMatrix(r, c);

        // In case the section is rotated:
        if (this->section->alpha) {
            // Do [K]^ = [R][K][R]'
            ChMatrix33<> Rotsect;
            Rotsect.Set_A_Rxyz(ChVector<>(-section->alpha, 0, 0));
            ChMatrixDynamic<> CKtemp(12, 12);
            ChMatrixCorotation<>::ComputeCK(this->StiffnessMatrix, Rotsect, 4, CKtemp);
            ChMatrixCorotation<>::ComputeKCt(CKtemp, Rotsect, 4, this->StiffnessMatrix);
        }
        // In case the section has a centroid displacement:
        if (this->section->Cy || this->section->Cz) {
            // Do [K]" = [T_c][K]^[T_c]'

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(4, i) += this->section->Cz * this->StiffnessMatrix(0, i);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(5, i) += -this->section->Cy * this->StiffnessMatrix(0, i);

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(10, i) += this->section->Cz * this->StiffnessMatrix(6, i);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(11, i) += -this->section->Cy * this->StiffnessMatrix(6, i);

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 4) += this->section->Cz * this->StiffnessMatrix(i, 0);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 5) += -this->section->Cy * this->StiffnessMatrix(i, 0);

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 10) += this->section->Cz * this->StiffnessMatrix(i, 6);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 11) += -this->section->Cy * this->StiffnessMatrix(i, 6);
        }

        // In case the section has a shear center displacement:
        if (this->section->Sy || this->section->Sz) {
            // Do [K]° = [T_s][K]"[T_s]'

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(3, i) +=
                    this->section->Sz * this->StiffnessMatrix(1, i) - this->section->Sy * this->StiffnessMatrix(2, i);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(9, i) +=
                    this->section->Sz * this->StiffnessMatrix(7, i) - this->section->Sy * this->StiffnessMatrix(8, i);

            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 3) +=
                    this->section->Sz * this->StiffnessMatrix(i, 1) - this->section->Sy * this->StiffnessMatrix(i, 2);
            for (int i = 0; i < 12; ++i)
                this->StiffnessMatrix(i, 9) +=
                    this->section->Sz * this->StiffnessMatrix(i, 7) - this->section->Sy * this->StiffnessMatrix(i, 8);
        }
    }

    /// Setup. Precompute mass and matrices that do not change during the
    /// simulation, such as the local tangent stiffness Kl of each element, if needed, etc.

    virtual void SetupInitial(ChSystem* system) override {
        assert(section);

        // Compute rest length, mass:
        this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
        this->mass = this->length * this->section->Area * this->section->density;

        // Compute initial rotation
        ChMatrix33<> A0;
        ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
        ChVector<> myele = nodes[0]->GetX0().GetA().Get_A_Yaxis();
        A0.Set_A_Xdir(mXele, myele);
        q_element_ref_rot = A0.Get_A_quaternion();

        // Compute local stiffness matrix:
        ComputeStiffnessMatrix();
    }

    /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
    /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
    virtual void ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor = 0, double Mfactor = 0) override {
        assert((H.GetRows() == 12) && (H.GetColumns() == 12));
        assert(section);

        // Corotational K stiffness:
        ChMatrixDynamic<> CK(12, 12);
        ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix

        if (!disable_projector) {
            //
            // Corotational approach as in G.Felippa
            //

            // compute [H(theta)]'[K_loc] [H(theta]

            ChMatrixDynamic<> displ(12, 1);
            this->GetStateBlock(displ);

            ChMatrix33<> mI;
            mI.Set33Identity();

            ChMatrix33<> LambdaA;
            ChMatrix33<> LambdaB;
            ChMatrix33<> MthetaA;
            ChMatrix33<> MthetaB;
            ChVector<> VthetaA(displ(3), displ(4), displ(5));
            ChVector<> VthetaB(displ(9), displ(10), displ(11));
            MthetaA.Set_X_matrix(VthetaA);
            MthetaB.Set_X_matrix(VthetaB);
            double thetaA = VthetaA.Length();
            double thetaB = VthetaB.Length();
            double zetaA;
            double zetaB;
            if (fabs(thetaA) > 0.05)
                zetaA = (1.0 - 0.5 * thetaA * (1.0 / tan(0.5 * thetaA))) / (pow(thetaA, 2));
            else
                zetaA = 1.0 / 12.0 + pow(thetaA, 2) / 720.0 + pow(thetaA, 4) / 30240.0 + pow(thetaA, 6) / 1209600.0;
            if (fabs(thetaB) > 0.05)
                zetaB = (1.0 - 0.5 * thetaB * (1.0 / tan(0.5 * thetaB))) / (pow(thetaB, 2));
            else
                zetaB = 1.0 / 12.0 + pow(thetaB, 2) / 720.0 + pow(thetaB, 4) / 30240.0 + pow(thetaB, 6) / 1209600.0;
            LambdaA = mI - MthetaA * 0.5 + (MthetaA * MthetaA) * zetaA;
            LambdaB = mI - MthetaB * 0.5 + (MthetaB * MthetaB) * zetaB;

            LambdaA.MatrTranspose();
            LambdaB.MatrTranspose();
            ChMatrixDynamic<> HtKH(12, 12);
            std::vector<ChMatrix33<>*> Ht;
            Ht.push_back(&mI);
            Ht.push_back(&LambdaA);
            Ht.push_back(&mI);
            Ht.push_back(&LambdaB);

            ChMatrixCorotation<>::ComputeCK(this->StiffnessMatrix, Ht, 4, CK);  // CK = [H(theta)]'[K_loc]
            ChMatrixCorotation<>::ComputeKCt(CK, Ht, 4, HtKH);                  // HtKH = [H(theta)]'[K_loc] [H(theta)]

            // compute K_m, K_gr, K_gm, K_gp

            ChVector<> vC = 0.5 * (nodes[0]->Frame().GetPos() + nodes[1]->Frame().GetPos());  // centroid
            ChVector<> vX_a_loc = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos() - vC);
            ChVector<> vX_b_loc = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos() - vC);
            double Lel = (nodes[0]->Frame().GetPos() - nodes[1]->Frame().GetPos()).Length();

            ChMatrix33<> mX_a;
            ChMatrix33<> mX_b;
            mX_a.Set_X_matrix(-vX_a_loc);
            mX_b.Set_X_matrix(-vX_b_loc);

            ChMatrixDynamic<> mS(12, 3);  // [S] = [ -skew[X_a_loc];  [I];  -skew[X_b_loc];  [I] ]
            mS.PasteMatrix(mX_a, 0, 0);
            mS.PasteMatrix(mI, 3, 0);
            mS.PasteMatrix(mX_b, 6, 0);
            mS.PasteMatrix(mI, 9, 0);

            ChMatrixDynamic<> mG(3, 12);  // [G] = [dw_frame/du_a; dw_frame/dw_a; dw_frame/du_b; dw_frame/dw_b]
            mG(2, 1) = -1. / Lel;
            mG(1, 2) = 1. / Lel;
            mG(2, 7) = 1. / Lel;
            mG(1, 8) = -1. / Lel;
            mG(0, 4) = 0.5;
            mG(0, 10) = 0.5;

            ChMatrixDynamic<> mP(12, 12);  // [P] = [I]-[S][G]
            mP.MatrMultiply(mS, mG);
            mP.MatrNeg();
            for (int k = 0; k < 12; ++k)
                mP(k, k) += 1.0;

            ChMatrixDynamic<> f_local(12, 1);  // f_loc = [K_loc]*u_loc
            f_local.MatrMultiply(StiffnessMatrix, displ);

            ChMatrixDynamic<> f_h(12, 1);  // f_h = [H(theta)]' [K_loc]*u_loc
            f_h.PasteVector(f_local.ClipVector(0, 0), 0, 0);
            f_h.PasteVector(LambdaA * f_local.ClipVector(3, 0), 3, 0);  // LambdaA is already transposed
            f_h.PasteVector(f_local.ClipVector(6, 0), 6, 0);
            f_h.PasteVector(LambdaB * f_local.ClipVector(9, 0), 9, 0);  // LambdaB is already transposed

            ChMatrixDynamic<> f_p(12, 1);  // f_p = [P]' [H(theta)]' [K_loc]*u_loc
            f_p.MatrTMultiply(mP, f_h);

            ChMatrixDynamic<> mFnm(12, 3);
            ChMatrixDynamic<> mFn(12, 3);
            ChMatrix33<> skew_f;

            if (!force_symmetric_stiffness) {
                skew_f.Set_X_matrix(f_p.ClipVector(0, 0));
                mFnm.PasteMatrix(skew_f, 0, 0);
                mFn.PasteMatrix(skew_f, 0, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(3, 0));
                mFnm.PasteMatrix(skew_f, 3, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(6, 0));
                mFnm.PasteMatrix(skew_f, 6, 0);
                mFn.PasteMatrix(skew_f, 6, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(9, 0));
                mFnm.PasteMatrix(skew_f, 9, 0);
            } else {
                skew_f.Set_X_matrix(f_p.ClipVector(0, 0));
                mFnm.PasteMatrix(skew_f, 0, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(3, 0) * 0.5);
                mFnm.PasteMatrix(skew_f, 3, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(6, 0));
                mFnm.PasteMatrix(skew_f, 6, 0);
                skew_f.Set_X_matrix(f_p.ClipVector(9, 0) * 0.5);
                mFnm.PasteMatrix(skew_f, 9, 0);
                mFn = mFnm;
            }

            ChMatrixDynamic<> mtemp(12, 12);

            ChMatrixDynamic<> K_m(12, 12);  // [K_m]  = [P]' [H(theta)]'[K_loc] [H(theta] [P]
            mtemp.MatrMultiply(HtKH, mP);
            K_m.MatrTMultiply(mP, mtemp);

            ChMatrixDynamic<> K_gr(12, 12);  // [K_gr] = [F_nm][G]
            K_gr.MatrMultiply(mFnm, mG);

            ChMatrixDynamic<> K_gp(12, 12);  // [K_gp] = [G]'[F_n]'[P] = ([F_n][G])'[P]
            mtemp.MatrMultiply(mFn, mG);
            K_gp.MatrTMultiply(mtemp, mP);

            // ...							// [K_gm] = [P]'[L][P]  (simplify: avoid computing this)

            ChMatrixDynamic<> K_tang(12, 12);  // [K_tang] = [K_m] - [K_gr] - [K_gp] + [K_gm]
            K_tang.MatrInc(K_m);
            K_tang.MatrDec(K_gr);
            K_tang.MatrDec(K_gp);

            // finally, do :   [K_tang_global] = [R][K_tang][R]'
            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            ChMatrixCorotation<>::ComputeCK(K_tang, R, 4, CK);
            ChMatrixCorotation<>::ComputeKCt(CK, R, 4, CKCt);
        } else {
            //
            // Simplified (inexact, faster) corotational approach
            //

            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            ChMatrixCorotation<>::ComputeCK(this->StiffnessMatrix, R, 4, CK);
            ChMatrixCorotation<>::ComputeKCt(CK, R, 4, CKCt);
        }

        // For strict symmetry, copy L=U because the computations above might
        // lead to small errors because of numerical roundoff even with force_symmetric_stiffness
        if (force_symmetric_stiffness) {
            for (int row = 0; row < CKCt.GetRows() - 1; ++row)
                for (int col = row + 1; col < CKCt.GetColumns(); ++col)
                    CKCt(row, col) = CKCt(col, row);
        }

        // For K stiffness matrix and R matrix: scale by factors

        double mkrfactor = Kfactor + Rfactor * this->section->rdamping;

        CKCt.MatrScale(mkrfactor);

        H.PasteMatrix(CKCt, 0, 0);  // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]

        // For M mass matrix, do mass lumping:

        ChMatrixDynamic<> Mloc(12, 12);

        double lmass = mass * 0.5;
        double lineryz = (1. / 50.) * mass * pow(length, 2);  // note: 1/50 can be even less (this is 0 in many texts)
        double linerx =
            (1. / 2.) * length * section->GetDensity() * (section->GetIyy() + section->GetIzz());  //***TO CHECK***

        Mloc(0, 0) += Mfactor * lmass;  // node A x,y,z
        Mloc(1, 1) += Mfactor * lmass;
        Mloc(2, 2) += Mfactor * lmass;

        Mloc(6, 6) += Mfactor * lmass;  // node B x,y,z
        Mloc(7, 7) += Mfactor * lmass;
        Mloc(8, 8) += Mfactor * lmass;

        Mloc(3, 3) += Mfactor * linerx;  // node A Ixx,Iyy,Izz  // NEED COROTATION
        Mloc(4, 4) += Mfactor * lineryz;
        Mloc(5, 5) += Mfactor * lineryz;

        Mloc(9, 9) += Mfactor * linerx;  // node B Ixx,Iyy,Izz // NEED COROTATION
        Mloc(10, 10) += Mfactor * lineryz;
        Mloc(11, 11) += Mfactor * lineryz;

        /* The following would be needed if consistent mass matrix is used, but...
        ChMatrix33<> Atoabs(this->q_element_abs_rot);
        ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        std::vector< ChMatrix33<>* > R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);

        ChMatrixCorotation<>::ComputeCK(Mloc, R, 4, CK);
        ChMatrixCorotation<>::ComputeKCt(CK, R, 4, CKCt);

        H.PasteSumMatrix(CKCt,0,0);
        */
        // ..rather do this because lumped mass matrix does not need rotation transf.
        H.PasteSumMatrix(Mloc, 0, 0);

        //***TO DO*** better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform
        // materials.
    }

    /// Computes the internal forces (ex. the actual position of
    /// nodes is not in relaxed reference position) and set values
    /// in the Fi vector.
    virtual void ComputeInternalForces(ChMatrixDynamic<>& Fi) override {
        assert((Fi.GetRows() == 12) && (Fi.GetColumns() == 1));
        assert(section);

        // set up vector of nodal displacements and small rotations (in local element system)
        ChMatrixDynamic<> displ(12, 1);
        this->GetStateBlock(displ);

        // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
        ChMatrixDynamic<> FiK_local(12, 1);
        FiK_local.MatrMultiply(StiffnessMatrix, displ);

        // set up vector of nodal velocities (in local element system)
        ChMatrixDynamic<> displ_dt(12, 1);
        this->GetField_dt(displ_dt);

        ChMatrixDynamic<> FiR_local(12, 1);
        FiR_local.MatrMultiply(StiffnessMatrix, displ_dt);
        FiR_local.MatrScale(this->section->GetBeamRaleyghDamping());

        FiK_local.MatrInc(FiR_local);

        FiK_local.MatrScale(-1.0);

        if (!disable_projector) {
            //
            // Corotational approach as in G.Felippa
            //

            ChMatrixDynamic<> displ(12, 1);
            this->GetStateBlock(displ);
            ChMatrix33<> mI;
            mI.Set33Identity();
            ChMatrix33<> LambdaA;
            ChMatrix33<> LambdaB;
            ChMatrix33<> MthetaA;
            ChMatrix33<> MthetaB;
            ChVector<> VthetaA(displ(3), displ(4), displ(5));
            ChVector<> VthetaB(displ(9), displ(10), displ(11));
            MthetaA.Set_X_matrix(VthetaA);
            MthetaB.Set_X_matrix(VthetaB);
            double thetaA = VthetaA.Length();
            double thetaB = VthetaB.Length();
            double zetaA;
            double zetaB;
            if (fabs(thetaA) > 0.05)
                zetaA = (1.0 - 0.5 * thetaA * (1.0 / tan(0.5 * thetaA))) / (pow(thetaA, 2));
            else
                zetaA = 1.0 / 12.0 + pow(thetaA, 2) / 720.0 + pow(thetaA, 4) / 30240.0 + pow(thetaA, 6) / 1209600.0;
            if (fabs(thetaB) > 0.05)
                zetaB = (1.0 - 0.5 * thetaB * (1.0 / tan(0.5 * thetaB))) / (pow(thetaB, 2));
            else
                zetaB = 1.0 / 12.0 + pow(thetaB, 2) / 720.0 + pow(thetaB, 4) / 30240.0 + pow(thetaB, 6) / 1209600.0;
            LambdaA = mI - MthetaA * 0.5 + (MthetaA * MthetaA) * zetaA;
            LambdaB = mI - MthetaB * 0.5 + (MthetaB * MthetaB) * zetaB;

            LambdaA.MatrTranspose();
            LambdaB.MatrTranspose();

            ChVector<> vC = 0.5 * (nodes[0]->Frame().GetPos() + nodes[1]->Frame().GetPos());  // centroid
            ChVector<> vX_a_loc = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos() - vC);
            ChVector<> vX_b_loc = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos() - vC);
            double Lel = (nodes[0]->Frame().GetPos() - nodes[1]->Frame().GetPos()).Length();

            ChMatrix33<> mX_a;
            ChMatrix33<> mX_b;
            mX_a.Set_X_matrix(-vX_a_loc);
            mX_b.Set_X_matrix(-vX_b_loc);

            ChMatrixDynamic<> mS(12, 3);  // [S] = [ -skew[X_a_loc];  [I];  -skew[X_b_loc];  [I] ]
            mS.PasteMatrix(mX_a, 0, 0);
            mS.PasteMatrix(mI, 3, 0);
            mS.PasteMatrix(mX_b, 6, 0);
            mS.PasteMatrix(mI, 9, 0);

            ChMatrixDynamic<> mG(3, 12);  // [G] = [dw_frame/du_a; dw_frame/dw_a; dw_frame/du_b; dw_frame/dw_b]
            mG(2, 1) = -1. / Lel;
            mG(1, 2) = 1. / Lel;
            mG(2, 7) = 1. / Lel;
            mG(1, 8) = -1. / Lel;
            mG(0, 4) = 0.5;
            mG(0, 10) = 0.5;

            ChMatrixDynamic<> mP(12, 12);  // [P] = [I]-[S][G]
            mP.MatrMultiply(mS, mG);
            mP.MatrNeg();
            for (int k = 0; k < 12; ++k)
                mP(k, k) += 1.0;

            ChMatrixDynamic<> HF(12, 1);  //  HF =  [H(theta)]' F_local
            HF.PasteVector(FiK_local.ClipVector(0, 0), 0, 0);
            HF.PasteVector(LambdaA * FiK_local.ClipVector(3, 0), 3, 0);
            HF.PasteVector(FiK_local.ClipVector(6, 0), 6, 0);
            HF.PasteVector(LambdaB * FiK_local.ClipVector(9, 0), 9, 0);

            ChMatrixDynamic<> PHF(12, 1);
            PHF.MatrTMultiply(mP, HF);

            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);
            ChMatrixCorotation<>::ComputeCK(PHF, R, 4, Fi);
        } else {
            //
            // Simplified (inexact, faster) corotational approach
            //

            // Fi = C * Fi_local  with C block-diagonal rotations A  , for nodal forces in abs. frame
            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);
            ChMatrixCorotation<>::ComputeCK(FiK_local, R, 4, Fi);
        }

#ifdef BEAM_VERBOSE
        GetLog() << "\nInternal forces (local): \n";
        for (int c = 0; c < 6; c++)
            GetLog() << FiK_local(c) << "  ";
        GetLog() << "\n";
        for (int c = 6; c < 12; c++)
            GetLog() << FiK_local(c) << "  ";
        GetLog() << "\n\nInternal forces (ABS) : \n";
        for (int c = 0; c < 6; c++)
            GetLog() << Fi(c) << "  ";
        GetLog() << "\n";
        for (int c = 6; c < 12; c++)
            GetLog() << Fi(c) << "  ";
        GetLog() << "\n";
#endif
    }

    //
    // Beam-specific functions
    //

    /// Gets the xyz displacement of a point on the beam line,
    /// and the rotation RxRyRz of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are not corotated.
    virtual void EvaluateSectionDisplacement(const double eta,
                                             const ChMatrix<>& displ,
                                             ChVector<>& u_displ,
                                             ChVector<>& u_rotaz) override {
        ChMatrixNM<double, 1, 12> N;

        this->ShapeFunctions(N, eta);  // Evaluate compressed shape functions
        /*
        u_displ.x() = N(0) * displ(0) + N(6) * displ(6);      // x_a   x_b
        u_displ.y() = N(1) * displ(1) + N(7) * displ(7)       // y_a   y_b
                    + N(5) * displ(5) + N(11) * displ(11);  // Rz_a  Rz_b
        u_displ.z() = N(2) * displ(2) + N(8) * displ(8)       // z_a   z_b
                    + N(4) * displ(4) + N(10) * displ(10);  // Ry_a  Ry_b

        u_rotaz.x() = N(3) * displ(3) + N(9) * displ(9);  // Rx_a  Rx_b

        double dN_ua = (1. / (2. * this->GetRestLength())) *
                       (-3. + 3 * eta * eta);  // slope shape functions are computed here on-the-fly
        double dN_ub = (1. / (2. * this->GetRestLength())) * (3. - 3 * eta * eta);
        double dN_ra = (1. / 4.) * (-1. - 2 * eta + 3 * eta * eta);
        double dN_rb = -(1. / 4.) * (1. - 2 * eta - 3 * eta * eta);
        u_rotaz.y() = -dN_ua * displ(2) - dN_ub * displ(8) +  // z_a   z_b   note - sign
                    dN_ra * displ(4) + dN_rb * displ(10);   // Ry_a  Ry_b
        u_rotaz.z() = dN_ua * displ(1) + dN_ub * displ(7) +   // y_a   y_b
                    dN_ra * displ(5) + dN_rb * displ(11);   // Rz_a  Rz_b
        */
        u_displ.x() = N(0) * displ(0) + N(3) * displ(6);     // x_a   x_b
        u_displ.y() = N(1) * displ(1) + N(4) * displ(7)      // y_a   y_b
                    + N(2) * displ(5) + N(5) * displ(11);  // Rz_a  Rz_b
        u_displ.z() = N(1) * displ(2) + N(4) * displ(8)      // z_a   z_b
                    - N(2) * displ(4) - N(5) * displ(10);  // Ry_a  Ry_b

        u_rotaz.x() = N(0) * displ(3) + N(3) * displ(9);    // Rx_a  Rx_b
        u_rotaz.y() = -N(6) * displ(2) - N(7) * displ(8) +  // z_a   z_b   note - sign
                    N(8) * displ(4) + N(9) * displ(10);   // Ry_a  Ry_b
        u_rotaz.z() = N(6) * displ(1) + N(7) * displ(7) +   // y_a   y_b
                    N(8) * displ(5) + N(9) * displ(11);   // Rz_a  Rz_b
    }

    /// Gets the absolute xyz position of a point on the beam line,
    /// and the absolute rotation of section plane, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock()
    /// Results are corotated (expressed in world reference)
    virtual void EvaluateSectionFrame(const double eta,
                                      const ChMatrix<>& displ,
                                      ChVector<>& point,
                                      ChQuaternion<>& rot) override {
        ChVector<> u_displ;
        ChVector<> u_rotaz;
        double Nx1 = (1. / 2.) * (1 - eta);
        double Nx2 = (1. / 2.) * (1 + eta);

        this->EvaluateSectionDisplacement(eta, displ, u_displ, u_rotaz);

        // Since   d = [Atw]' Xt - [A0w]'X0   , so we have
        //        Xt = [Atw] (d +  [A0w]'X0)

        point =
            this->q_element_abs_rot.Rotate(u_displ +
                                           this->q_element_ref_rot.RotateBack(Nx1 * this->nodes[0]->GetX0().GetPos() +
                                                                              Nx2 * this->nodes[1]->GetX0().GetPos()));

        ChQuaternion<> msectionrot;
        msectionrot.Q_from_AngAxis(u_rotaz.Length(), u_rotaz.GetNormalized());
        rot = this->q_element_abs_rot % msectionrot;
    }

    /// Gets the force (traction x, shear y, shear z) and the
    /// torque (torsion on x, bending on y, on bending on z) at a section along
    /// the beam line, at abscyssa 'eta'.
    /// Note, eta=-1 at node1, eta=+1 at node2.
    /// Note, 'displ' is the displ.state of 2 nodes, ex. get it as GetStateBlock().
    /// Results are not corotated, and are expressed in the reference system of beam.
    virtual void EvaluateSectionForceTorque(const double eta,
                                            const ChMatrix<>& displ,
                                            ChVector<>& Fforce,
                                            ChVector<>& Mtorque) override {
        assert(section);

        double Jpolar = section->J;

        // ChMatrixNM<double, 1, 12> N;

        // this->ShapeFunctions(N, eta);  // Evaluate compressed shape functions

        // shape function derivatives are computed here on-the-fly
        double dN_xa = -(1. / length);
        double dN_xb = (1. / length);
        double ddN_ua = (6. / (length * length)) * (eta);
        double ddN_ub = -(6. / (length * length)) * (eta);
        double ddN_ra = -(1. / length) + ((3.0 / length) * eta);
        double ddN_rb = (1. / length) + ((3.0 / length) * eta);
        double dddN_ua = (12. / (length * length * length));
        double dddN_ub = -(12. / (length * length * length));
        double dddN_ra = (6.0 / (length * length));
        double dddN_rb = (6.0 / (length * length));

        // generalized strains/curvatures;
        ChMatrixNM<double, 6, 1> sect_ek;

        // e_x
        sect_ek(0) = (dN_xa * displ(0) + dN_xb * displ(6));  // x_a   x_b
        // e_y
        sect_ek(1) = (dddN_ua * displ(1) + dddN_ub * displ(7) +   // y_a   y_b
                      dddN_ra * displ(5) + dddN_rb * displ(11));  // Rz_a  Rz_b
        // e_z
        sect_ek(2) = (-dddN_ua * displ(2) - dddN_ub * displ(8) +  // z_a   z_b   note - sign
                      dddN_ra * displ(4) + dddN_rb * displ(10));  // Ry_a  Ry_b

        // k_x
        sect_ek(3) = (dN_xa * displ(3) + dN_xb * displ(9));  // Rx_a  Rx_b
        // k_y
        sect_ek(4) = (-ddN_ua * displ(2) - ddN_ub * displ(8) +  // z_a   z_b   note - sign
                      ddN_ra * displ(4) + ddN_rb * displ(10));  // Ry_a  Ry_b
        // k_z
        sect_ek(5) = (ddN_ua * displ(1) + ddN_ub * displ(7) +   // y_a   y_b
                      ddN_ra * displ(5) + ddN_rb * displ(11));  // Rz_a  Rz_b

        if (false)  // section->alpha ==0 && section->Cy ==0 && section->Cz==0 && section->Sy==0 && section->Sz==0)
        {
            // Fast computation:
            Fforce.x() = this->section->E * this->section->Area * sect_ek(0);
            Fforce.y() = this->section->E * this->section->Izz * sect_ek(1);
            Fforce.z() = this->section->E * this->section->Iyy * sect_ek(2);

            Mtorque.x() = this->section->G * Jpolar * sect_ek(3);
            Mtorque.y() = this->section->E * this->section->Iyy * sect_ek(4);
            Mtorque.z() = this->section->E * this->section->Izz * sect_ek(5);
        } else {
            // Generic computation, by rotating and translating the constitutive
            // matrix of the beam:
            ChMatrixNM<double, 6, 6> Klaw_d;
            double ca = cos(section->alpha);
            double sa = sin(section->alpha);
            double cb = cos(section->alpha);  // could be beta if shear custom axes
            double sb = sin(section->alpha);
            double Cy = section->Cy;
            double Cz = section->Cz;
            double Sy = section->Sy;
            double Sz = section->Sz;
            double Klaw_d0 = this->section->E * this->section->Area;
            double Klaw_d1 = this->section->E * this->section->Izz;
            double Klaw_d2 = this->section->E * this->section->Iyy;
            double Klaw_d3 = this->section->G * Jpolar;
            double Klaw_d4 = this->section->E * this->section->Iyy;
            double Klaw_d5 = this->section->E * this->section->Izz;
            // ..unrolled rotated constitutive matrix..
            ChMatrixNM<double, 6, 6> Klaw_r;

            Klaw_r(0, 0) = Klaw_d0;
            Klaw_r(1, 1) = Klaw_d1 * cb * cb + Klaw_d2 * sb * sb;
            Klaw_r(2, 2) = Klaw_d1 * sb * sb + Klaw_d2 * cb * cb;
            Klaw_r(1, 2) = Klaw_d1 * cb * sb - Klaw_d2 * cb * sb;
            Klaw_r(2, 1) = Klaw_r(1, 2);
            Klaw_r(3, 3) = Klaw_d3;
            Klaw_r(4, 4) = Klaw_d4 * ca * ca + Klaw_d5 * sa * sa;
            Klaw_r(5, 5) = Klaw_d4 * sa * sa + Klaw_d5 * ca * ca;
            Klaw_r(4, 5) = Klaw_d4 * ca * sa - Klaw_d5 * ca * sa;
            Klaw_r(5, 4) = Klaw_r(4, 5);

            // ..also translate for Cy Cz
            for (int i = 0; i < 6; ++i)
                Klaw_r(i, 4) += Cz * Klaw_r(i, 0);
            for (int i = 0; i < 6; ++i)
                Klaw_r(i, 5) += -Cy * Klaw_r(i, 0);

            // ..also translate for Sy Sz
            for (int i = 0; i < 6; ++i)
                Klaw_r(i, 3) += Sz * Klaw_r(i, 1) - Sy * Klaw_r(i, 2);

            // .. compute wrench = Klaw_l * sect_ek
            ChMatrixNM<double, 6, 1> wrench;
            wrench.MatrMultiply(Klaw_r, sect_ek);
            Fforce = wrench.ClipVector(0, 0);
            Mtorque = wrench.ClipVector(3, 0);

            // Note: to be checked.
            // Note: can be improved with more unrolling.
        }
    }
    virtual void EvaluateSectionStrain(
        const double eta,
        const ChMatrix<>& displ,
        ChVector<>& StrainV) override { /* To be completed: Created to be consistent with base class implementation*/
    }
    //
    // Functions for interfacing to the solver
    //            (***not needed, thank to bookkeeping in parent class ChElementGeneric)

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 2 * 7; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 2 * 6; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos(), block_offset, 0);
        mD.PasteQuaternion(this->nodes[0]->GetRot(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[1]->GetPos(), block_offset + 7, 0);
        mD.PasteQuaternion(this->nodes[1]->GetRot(), block_offset + 10, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(this->nodes[0]->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->nodes[0]->GetWvel_loc(), block_offset + 3, 0);
        mD.PasteVector(this->nodes[1]->GetPos_dt(), block_offset + 6, 0);
        mD.PasteVector(this->nodes[1]->GetWvel_loc(), block_offset + 9, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        nodes[0]->NodeIntStateIncrement(off_x   , x_new, x, off_v   , Dv);
        nodes[1]->NodeIntStateIncrement(off_x+7 , x_new, x, off_v+6 , Dv);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
    virtual int Get_field_ncoords() override { return 6; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 2; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return nodes[nblock]->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 6; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        mvars.push_back(&this->nodes[0]->Variables());
        mvars.push_back(&this->nodes[1]->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U coordinates of the line, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in line
                           ChVectorDynamic<>& Qi,       ///< Return result of Q = N'*F  here
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is =n. field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        ChMatrixNM<double, 1, 12> N;
        this->ShapeFunctions(N, U);  // evaluate shape functions (in compressed vector), btw. not dependant on state

        detJ = this->GetRestLength() / 2.0;

        Qi(0) = N(0) * F(0);
        Qi(1) = N(1) * F(1) + N(6) * F(5);
        Qi(2) = N(1) * F(2) - N(6) * F(4);
        Qi(3) = N(0) * F(3);
        Qi(4) = -N(2) * F(2) + N(8) * F(4);
        Qi(5) = N(2) * F(1) + N(8) * F(5);

        Qi(6) = N(3) * F(0);
        Qi(7) = N(4) * F(1) + N(7) * F(5);
        Qi(8) = N(4) * F(2) - N(7) * F(4);
        Qi(9) = N(3) * F(3);
        Qi(10) = -N(5) * F(2) + N(9) * F(4);
        Qi(11) = N(5) * F(1) + N(9) * F(5);
    }

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in volume
                           const double V,              ///< parametric coordinate in volume
                           const double W,              ///< parametric coordinate in volume
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
        detJ /= 4.0;  // because volume
    }

    /// This is needed so that it can be accessed by ChLoaderVolumeGravity
    virtual double GetDensity() override { return this->section->Area * this->section->density; }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
