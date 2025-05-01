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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMate)

void ChLinkMate::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMate>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMate::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMate>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateGeneric)

ChLinkMateGeneric::ChLinkMateGeneric(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz)
    : c_x(mc_x), c_y(mc_y), c_z(mc_z), c_rx(mc_rx), c_ry(mc_ry), c_rz(mc_rz) {
    SetupLinkMask();
}

ChLinkMateGeneric::ChLinkMateGeneric(const ChLinkMateGeneric& other) : ChLinkMate(other) {
    c_x = other.c_x;
    c_y = other.c_y;
    c_z = other.c_z;
    c_rx = other.c_rx;
    c_ry = other.c_ry;
    c_rz = other.c_rz;

    SetupLinkMask();
}

void ChLinkMateGeneric::SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
    c_rx = mc_rx;
    c_ry = mc_ry;
    c_rz = mc_rz;

    SetupLinkMask();
}

void ChLinkMateGeneric::SetupLinkMask() {
    int nc = 0;
    if (c_x)
        nc++;
    if (c_y)
        nc++;
    if (c_z)
        nc++;
    if (c_rx)
        nc++;
    if (c_ry)
        nc++;
    if (c_rz)
        nc++;

    mask.SetNumConstraints(nc);

    C.setZero(nc);

    P = ChMatrix33<>(0.5);

    ChangedLinkMask();
}

void ChLinkMateGeneric::ChangedLinkMask() {
    m_num_constr = mask.GetNumConstraintsActive();
    m_num_constr_bil = mask.GetNumConstraintsBilateralActive();
    m_num_constr_uni = mask.GetNumConstraintsUnilateralActive();
}

void ChLinkMateGeneric::SetDisabled(bool mdis) {
    ChLinkMate::SetDisabled(mdis);

    if (mask.SetAllDisabled(mdis) > 0)
        ChangedLinkMask();
}

void ChLinkMateGeneric::SetBroken(bool mbro) {
    ChLinkMate::SetBroken(mbro);

    if (mask.SetAllBroken(mbro) > 0)
        ChangedLinkMask();
}

void ChLinkMateGeneric::Update(double time, bool update_assets) {
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::Update(time, update_assets);

    if (this->m_body1 && this->m_body2) {
        this->mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());

        ChFrame<> F1_W = m_frame1 >> (*this->m_body1);
        ChFrame<> F2_W = m_frame2 >> (*this->m_body2);
        ChFrame<> F1_wrt_F2 = F2_W.TransformParentToLocal(F1_W);
        // Now 'F1_wrt_F2' contains the position/rotation of frame 1 respect to frame 2, in frame 2 coords.

        ChMatrix33<> Jx1 = F2_W.GetRotMat().transpose();
        ChMatrix33<> Jx2 = -F2_W.GetRotMat().transpose();

        ChMatrix33<> Jr1 = -F2_W.GetRotMat().transpose() * m_body1->GetRotMat() * ChStarMatrix33<>(m_frame1.GetPos());
        ChVector3d r12_B2 = m_body2->GetRotMat().transpose() * (F1_W.GetPos() - F2_W.GetPos());
        ChMatrix33<> Jr2 = m_frame2.GetRotMat().transpose() * ChStarMatrix33<>(m_frame2.GetPos() + r12_B2);

        // Premultiply by Jw1 and Jw2 by P = 0.5 * [Fp(q_resid^*)]'.bottomRow(3) to get residual as imaginary part of a
        // quaternion. For small misalignment this effect is almost insignificant because P ~= [I33], but otherwise it
        // is needed (if you want to use the stabilization term - if not, you can live without).
        this->P = 0.5 * (ChMatrix33<>(F1_wrt_F2.GetRot().e0()) + ChStarMatrix33<>(F1_wrt_F2.GetRot().GetVector()));

        ChMatrix33<> Jw1 = this->P.transpose() * F2_W.GetRotMat().transpose() * m_body1->GetRotMat();
        ChMatrix33<> Jw2 = -this->P.transpose() * F2_W.GetRotMat().transpose() * m_body2->GetRotMat();

        // Another equivalent expression:
        // ChMatrix33<> Jw1 = this->P * F1_W.GetRotMat().transpose() * m_body1->GetRotMat();
        // ChMatrix33<> Jw2 = -this->P * F1_W.GetRotMat().transpose() * m_body2->GetRotMat();

        // The Jacobian matrix of constraint is:
        // Cq = [ Jx1,  Jr1,  Jx2,  Jr2 ]
        //      [   0,  Jw1,    0,  Jw2 ]

        int nc = 0;

        if (c_x) {
            C(nc) = F1_wrt_F2.GetPos().x();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(0);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(0);
            nc++;
        }
        if (c_y) {
            C(nc) = F1_wrt_F2.GetPos().y();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(1);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(1);
            nc++;
        }
        if (c_z) {
            C(nc) = F1_wrt_F2.GetPos().z();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(2);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(2);
            nc++;
        }
        if (c_rx) {
            C(nc) = F1_wrt_F2.GetRot().e1();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = F1_wrt_F2.GetRot().e2();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = F1_wrt_F2.GetRot().e3();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
    }
}

void ChLinkMateGeneric::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                   std::shared_ptr<ChBodyFrame> body2,
                                   ChFrame<> absframe) {
    this->Initialize(body1, body2, false, absframe, absframe);
}

void ChLinkMateGeneric::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                   std::shared_ptr<ChBodyFrame> body2,
                                   bool pos_are_relative,
                                   ChFrame<> frame1,
                                   ChFrame<> frame2) {
    assert(body1.get() != body2.get());

    this->m_body1 = body1.get();
    this->m_body2 = body2.get();
    // this->SetSystem(body1->GetSystem());

    this->mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());

    if (pos_are_relative) {
        m_frame1 = frame1;
        m_frame2 = frame2;
    } else {
        // from abs to body-rel
        m_frame1 = static_cast<ChFrame<>*>(this->m_body1)->TransformParentToLocal(frame1);
        m_frame2 = static_cast<ChFrame<>*>(this->m_body2)->TransformParentToLocal(frame2);
    }
}

void ChLinkMateGeneric::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                   std::shared_ptr<ChBodyFrame> body2,
                                   bool pos_are_relative,
                                   const ChVector3d& point1,
                                   const ChVector3d& point2,
                                   const ChVector3d& dir1,
                                   const ChVector3d& dir2) {
    if (body1.get() == body2.get())
        throw std::invalid_argument("Cannot constrain a body to itself.");

    this->m_body1 = body1.get();
    this->m_body2 = body2.get();
    // this->SetSystem(body1->GetSystem());

    this->mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());

    ChMatrix33<> mrot;

    ChFrame<> mfr1;
    ChFrame<> mfr2;

    if (pos_are_relative) {
        mrot.SetFromAxisZ(dir1);
        mfr1.SetRot(mrot);
        mfr1.SetPos(point1);

        mrot.SetFromAxisZ(dir2);
        mfr2.SetRot(mrot);
        mfr2.SetPos(point2);
    } else {
        // from abs to body-relative coordinates
        ChVector3d link_dir_local;

        link_dir_local = this->m_body1->TransformDirectionParentToLocal(dir1);
        mrot.SetFromAxisZ(link_dir_local);
        mfr1.SetRot(mrot);
        mfr1.SetPos(this->m_body1->TransformPointParentToLocal(point1));

        link_dir_local = this->m_body2->TransformDirectionParentToLocal(dir2);
        mrot.SetFromAxisZ(link_dir_local);
        mfr2.SetRot(mrot);
        mfr2.SetPos(this->m_body2->TransformPointParentToLocal(point2));
    }

    m_frame1 = mfr1;
    m_frame2 = mfr2;
}

void ChLinkMateGeneric::SetUseTangentStiffness(bool useKc) {
    if (useKc && this->Kmatr == nullptr) {
        this->Kmatr = chrono_types::make_unique<ChKRMBlock>(&m_body1->Variables(), &m_body2->Variables());
        this->Kmatr->GetMatrix().resize(12, 12);
        this->Kmatr->GetMatrix().setZero();

    } else if (!useKc && this->Kmatr != nullptr) {
        this->Kmatr.reset();
    }
}

void ChLinkMateGeneric::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (!this->IsActive())
        return;

    if (this->Kmatr)
        descriptor.InsertKRMBlock(Kmatr.get());
}

void ChLinkMateGeneric::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (!this->IsActive())
        return;

    if (this->Kmatr) {
        ChMatrix33<> R_B1_W = m_body1->GetRotMat();
        ChMatrix33<> R_B2_W = m_body2->GetRotMat();
        // ChMatrix33<> R_F1_B1 = frame1.GetRotMat();
        // ChMatrix33<> R_F2_B2 = frame2.GetRotMat();
        ChFrame<> F1_W = m_frame1 >> (*this->m_body1);
        ChFrame<> F2_W = m_frame2 >> (*this->m_body2);
        ChMatrix33<> R_F1_W = F1_W.GetRotMat();
        ChMatrix33<> R_F2_W = F2_W.GetRotMat();
        ChVector3d r12_B2 = R_B2_W.transpose() * (F1_W.GetPos() - F2_W.GetPos());
        ChFrame<> F1_wrt_F2 = F2_W.TransformParentToLocal(F1_W);

        ChVector3d r_F1_B1 = m_frame1.GetPos();
        ChVector3d r_F2_B2 = m_frame2.GetPos();
        ChStarMatrix33<> rtilde_F1_B1(r_F1_B1);
        ChStarMatrix33<> rtilde_F2_B2(r_F2_B2);

        // Precomupte utilities
        ChMatrix33<> R_F2_W_cross_gamma_f = R_F2_W * ChStarMatrix33<>(gamma_f);
        ChMatrix33<> R_W_F2 = R_F2_W.transpose();
        ChVector3d v_F1_F2 = F1_wrt_F2.GetRot().GetVector();
        ChMatrix33<> s_F1_F2_mat(F1_wrt_F2.GetRot().e0());
        ChMatrix33<> R_F2_W_times_G_times_R_W_F1 =
            R_F2_W *
            (-0.25 * TensorProduct(gamma_m, v_F1_F2) -
             0.25 * ChStarMatrix33<>(gamma_m) * (s_F1_F2_mat + ChStarMatrix33<>(v_F1_F2))) *
            R_F1_W.transpose();
        ChMatrix33<> R_F2_W_cross_gamma_f_times_R_B2_F2 = R_F2_W_cross_gamma_f * R_W_F2 * R_B2_W;

        // Populate Kc
        this->Kmatr->GetMatrix().block<3, 3>(0, 9) = -R_F2_W_cross_gamma_f * R_W_F2 * R_B2_W;

        this->Kmatr->GetMatrix().block<3, 3>(3, 3) =
            rtilde_F1_B1 * R_B1_W.transpose() * R_F2_W_cross_gamma_f * R_W_F2 * R_B1_W +
            R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(this->P * gamma_m) * R_W_F2 * R_B1_W
            // stabilization part
            + R_B1_W.transpose() * R_F2_W_times_G_times_R_W_F1 * R_B1_W;

        this->Kmatr->GetMatrix().block<3, 3>(3, 9) =
            -rtilde_F1_B1 * R_B1_W.transpose() * R_F2_W_cross_gamma_f * R_W_F2 * R_B2_W -
            R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(this->P * gamma_m) * R_W_F2 * R_B2_W
            // stabilization part
            - R_B1_W.transpose() * R_F2_W_times_G_times_R_W_F1 * R_B2_W;

        this->Kmatr->GetMatrix().block<3, 3>(6, 9) = R_F2_W_cross_gamma_f * R_W_F2 * R_B2_W;

        this->Kmatr->GetMatrix().block<3, 3>(9, 0) = R_F2_W_cross_gamma_f_times_R_B2_F2;

        this->Kmatr->GetMatrix().block<3, 3>(9, 3) = -R_F2_W_cross_gamma_f_times_R_B2_F2 * R_B1_W * rtilde_F1_B1
                                                 // stabilization part
                                                 - R_B2_W.transpose() * R_F2_W_times_G_times_R_W_F1 * R_B1_W;

        this->Kmatr->GetMatrix().block<3, 3>(9, 6) = -R_F2_W_cross_gamma_f_times_R_B2_F2;

        this->Kmatr->GetMatrix().block<3, 3>(9, 9) =
            R_F2_W_cross_gamma_f_times_R_B2_F2 * R_B2_W * ChStarMatrix33<>(r12_B2 + r_F2_B2)
            // stabilization part
            + R_B2_W.transpose() * R_F2_W_times_G_times_R_W_F1 * R_B2_W;

        // The complete tangent stiffness matrix
        this->Kmatr->GetMatrix() *= Kfactor;
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMateGeneric::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_f.x();
        nc++;
    }
    if (c_y) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_f.y();
        nc++;
    }
    if (c_z) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_f.z();
        nc++;
    }
    if (c_rx) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_m.x();
        nc++;
    }
    if (c_ry) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_m.y();
        nc++;
    }
    if (c_rz) {
        if (mask.GetConstraint(nc).IsActive())
            L(off_L + nc) = -gamma_m.z();
        nc++;
    }
}

void ChLinkMateGeneric::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force = VNULL;
    react_torque = VNULL;
    gamma_f = VNULL;
    gamma_m = VNULL;

    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.x() = -L(off_L + nc);
        nc++;
    }
    if (c_y) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.y() = -L(off_L + nc);
        nc++;
    }
    if (c_z) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.z() = -L(off_L + nc);
        nc++;
    }
    if (c_rx) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.x() = -L(off_L + nc);
        nc++;
    }
    if (c_ry) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.y() = -L(off_L + nc);
        nc++;
    }
    if (c_rz) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.z() = -L(off_L + nc);
        nc++;
    }

    // The transpose of the Jacobian matrix of constraint is:
    // Cq.T = [ Jx1.T;  O      ]
    //        [ Jr1.T;  Jw1.T  ]
    //        [ Jx2.T;  O      ]
    //        [ Jr2.T;  Jw2.T  ]
    // The Lagrange multipliers are:
    // gamma = [ gamma_f ]
    //         [ gamma_m ]
    //
    // The forces and torques acting on m_body1 and m_body2 are simply calculated as: Cq.T*gamma,
    // where the forces are expressed in the absolute frame,
    // the torques are expressed in the local frame of B1,B2, respectively.
    // This is consistent with the mixed basis of rigid bodies and nodes.
    react_force = gamma_f;
    ChFrame<> F1_W = m_frame1 >> (*this->m_body1);
    ChFrame<> F2_W = m_frame2 >> (*this->m_body2);
    ChVector3d r12_F2 = F2_W.GetRotMat().transpose() * (F1_W.GetPos() - F2_W.GetPos());
    react_torque = ChStarMatrix33<>(r12_F2) * gamma_f + this->P * gamma_m;
}

void ChLinkMateGeneric::IntLoadResidual_CqL(const unsigned int off_L,
                                            ChVectorDynamic<>& R,
                                            const ChVectorDynamic<>& L,
                                            const double c) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).AddJacobianTransposedTimesScalarInto(R, L(off_L + cnt) * c);
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntLoadConstraint_C(const unsigned int off_L,
                                            ChVectorDynamic<>& Qc,
                                            const double c,
                                            bool do_clamp,
                                            double recovery_clamp) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            if (do_clamp) {
                if (mask.GetConstraint(i).IsUnilateral())
                    Qc(off_L + cnt) += std::max(c * C(cnt), -recovery_clamp);
                else
                    Qc(off_L + cnt) += std::min(std::max(c * C(cnt), -recovery_clamp), recovery_clamp);
            } else
                Qc(off_L + cnt) += c * C(cnt);
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    // NOT NEEDED BECAUSE NO RHEONOMIC TERM
}

void ChLinkMateGeneric::IntToDescriptor(const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,
                                        const ChVectorDynamic<>& L,
                                        const ChVectorDynamic<>& Qc) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).SetLagrangeMultiplier(L(off_L + cnt));
            mask.GetConstraint(i).SetRightHandSide(Qc(off_L + cnt));
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntFromDescriptor(const unsigned int off_v,
                                          ChStateDelta& v,
                                          const unsigned int off_L,
                                          ChVectorDynamic<>& L) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            L(off_L + cnt) = mask.GetConstraint(i).GetLagrangeMultiplier();
            cnt++;
        }
    }
}

// SOLVER INTERFACES

void ChLinkMateGeneric::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!this->IsActive())
        return;

    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive())
            descriptor.InsertConstraint(&mask.GetConstraint(i));
    }
}

void ChLinkMateGeneric::ConstraintsBiReset() {
    if (!this->IsActive())
        return;

    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        mask.GetConstraint(i).SetRightHandSide(0.);
    }
}

void ChLinkMateGeneric::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!this->IsActive())
        return;

    //// TEST
    /*
        std::cout << "cload: " ;
        if (this->c_x) std::cout << " x";
        if (this->c_y) std::cout << " y";
        if (this->c_z) std::cout << " z";
        if (this->c_rx) std::cout << " Rx";
        if (this->c_ry) std::cout << " Ry";
        if (this->c_rz) std::cout << " Rz";
        std::cout << *this->C << std::endl;
    */
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            if (do_clamp) {
                if (mask.GetConstraint(i).IsUnilateral())
                    mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() +
                                                  std::max(factor * C(cnt), -recovery_clamp));
                else
                    mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() +
                                                  std::min(std::max(factor * C(cnt), -recovery_clamp), recovery_clamp));
            } else
                mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() + factor * C(cnt));

            cnt++;
        }
    }
}

void ChLinkMateGeneric::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    // NOT NEEDED BECAUSE NO RHEONOMIC TERM
}

void ChLinkMateGeneric::LoadConstraintJacobians() {
    // already loaded when doing Update (which used the matrices of the scalar constraint objects)
}

void ChLinkMateGeneric::ConstraintsFetch_react(double factor) {
    react_force = VNULL;
    react_torque = VNULL;
    gamma_f = VNULL;
    gamma_m = VNULL;

    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.x() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }
    if (c_y) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.y() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }
    if (c_z) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_f.z() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }
    if (c_rx) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.x() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }
    if (c_ry) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.y() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }
    if (c_rz) {
        if (mask.GetConstraint(nc).IsActive())
            gamma_m.z() = -mask.GetConstraint(nc).GetLagrangeMultiplier() * factor;
        nc++;
    }

    react_force = gamma_f;

    ChFrame<> F1_W = m_frame1 >> (*this->m_body1);
    ChFrame<> F2_W = m_frame2 >> (*this->m_body2);
    ChVector3d r12_F2 = F2_W.GetRotMat().transpose() * (F1_W.GetPos() - F2_W.GetPos());
    react_torque = ChStarMatrix33<>(r12_F2) * gamma_f + this->P * gamma_m;
}

void ChLinkMateGeneric::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateGeneric>();

    // serialize parent class
    ChLinkMate::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_frame1);
    archive_out << CHNVP(m_frame2);
    archive_out << CHNVP(c_x);
    archive_out << CHNVP(c_y);
    archive_out << CHNVP(c_z);
    archive_out << CHNVP(c_rx);
    archive_out << CHNVP(c_ry);
    archive_out << CHNVP(c_rz);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateGeneric::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateGeneric>();

    // deserialize parent class
    ChLinkMate::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_frame1);
    archive_in >> CHNVP(m_frame2);

    bool c_x_success = archive_in.in(CHNVP(c_x));
    bool c_y_success = archive_in.in(CHNVP(c_y));
    bool c_z_success = archive_in.in(CHNVP(c_z));
    bool c_rx_success = archive_in.in(CHNVP(c_rx));
    bool c_ry_success = archive_in.in(CHNVP(c_ry));
    bool c_rz_success = archive_in.in(CHNVP(c_rz));
    if (c_x_success && c_y_success && c_z_success && c_rx_success && c_ry_success && c_rz_success)
        this->SetConstrainedCoords(c_x, c_y, c_z, c_rx, c_ry, c_rz);  // takes care of mask
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMatePlanar)

ChLinkMatePlanar::ChLinkMatePlanar(const ChLinkMatePlanar& other) : ChLinkMateGeneric(other) {
    m_flipped = other.m_flipped;
    m_distance = other.m_distance;
}

void ChLinkMatePlanar::SetFlipped(bool doflip) {
    if (doflip != this->m_flipped) {
        // swaps direction of Z axis by flipping 180 deg the frame 1 (slave)

        ChFrame<> frameRotator(VNULL, QuatFromAngleY(CH_PI));
        m_frame1.ConcatenatePostTransformation(frameRotator);

        this->m_flipped = doflip;
    }
}

void ChLinkMatePlanar::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                  std::shared_ptr<ChBodyFrame> body2,
                                  bool pos_are_relative,
                                  const ChVector3d& point1,
                                  const ChVector3d& point2,
                                  const ChVector3d& norm1,
                                  const ChVector3d& norm2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, m_flipped ? -norm1 : norm1, norm2);
}

void ChLinkMatePlanar::Update(double time, bool update_assets) {
    // Parent class inherit
    ChLinkMateGeneric::Update(time, update_assets);

    // Compensate for the imposed offset between the two planes
    C(0) -= m_distance;
}

void ChLinkMatePlanar::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMatePlanar>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_flipped);
    archive_out << CHNVP(m_distance);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMatePlanar::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMatePlanar>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_flipped);
    archive_in >> CHNVP(m_distance);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateCylindrical)

ChLinkMateCylindrical::ChLinkMateCylindrical(const ChLinkMateCylindrical& other) : ChLinkMateGeneric(other) {
    m_flipped = other.m_flipped;
}

void ChLinkMateCylindrical::SetFlipped(bool doflip) {
    if (doflip != m_flipped) {
        // swaps direction of Z axis by flipping 180 deg the frame A (slave)

        ChFrame<> frameRotator(VNULL, QuatFromAngleY(CH_PI));
        m_frame1.ConcatenatePostTransformation(frameRotator);

        m_flipped = doflip;
    }
}

void ChLinkMateCylindrical::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                       std::shared_ptr<ChBodyFrame> body2,
                                       bool pos_are_relative,
                                       const ChVector3d& point1,
                                       const ChVector3d& point2,
                                       const ChVector3d& dir1,
                                       const ChVector3d& dir2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, m_flipped ? -dir1 : dir1, dir2);
}

void ChLinkMateCylindrical::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateCylindrical>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateCylindrical::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateCylindrical>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    archive_in >> CHNVP(m_flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateRevolute)

ChLinkMateRevolute::ChLinkMateRevolute(const ChLinkMateRevolute& other) : ChLinkMateGeneric(other) {
    m_flipped = other.m_flipped;
}

void ChLinkMateRevolute::SetFlipped(bool doflip) {
    if (doflip != m_flipped) {
        // swaps direction of Z axis by flipping 180 deg the frame A (slave)

        ChFrame<> frameRotator(VNULL, QuatFromAngleY(CH_PI));
        m_frame1.ConcatenatePostTransformation(frameRotator);

        m_flipped = doflip;
    }
}

void ChLinkMateRevolute::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                    std::shared_ptr<ChBodyFrame> body2,
                                    bool pos_are_relative,
                                    const ChVector3d& point1,
                                    const ChVector3d& point2,
                                    const ChVector3d& dir1,
                                    const ChVector3d& dir2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, m_flipped ? -dir1 : dir1, dir2);
}

double ChLinkMateRevolute::GetRelativeAngle() {
    ChFrame<> F1_W = m_frame1 >> *m_body1;
    ChFrame<> F2_W = m_frame2 >> *m_body2;
    ChFrame<> F1_F2 = F2_W.TransformParentToLocal(F1_W);
    double angle = std::remainder(F1_F2.GetRot().GetRotVec().z(), CH_2PI);  // NB: assumes rotation along z
    return angle;
}

double ChLinkMateRevolute::GetRelativeAngleDt() {
    ChFrameMoving<> F1_W = ChFrameMoving<>(m_frame1) >> *m_body1;
    ChFrameMoving<> F2_W = ChFrameMoving<>(m_frame2) >> *m_body2;
    ChFrameMoving<> F1_F2 = F2_W.TransformParentToLocal(F1_W);
    double vel = F1_F2.GetAngVelLocal().z();  // NB: assumes rotation along z
    return vel;
}

double ChLinkMateRevolute::GetRelativeAngleDt2() {
    ChFrameMoving<> F1_W = ChFrameMoving<>(m_frame1) >> *m_body1;
    ChFrameMoving<> F2_W = ChFrameMoving<>(m_frame2) >> *m_body2;
    ChFrameMoving<> F1_F2 = F2_W.TransformParentToLocal(F1_W);
    double acc = F1_F2.GetAngAccLocal().z();  // NB: assumes rotation along z
    return acc;
}

void ChLinkMateRevolute::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateRevolute>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateRevolute::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateRevolute>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    archive_in >> CHNVP(m_flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMatePrismatic)

ChLinkMatePrismatic::ChLinkMatePrismatic(const ChLinkMatePrismatic& other) : ChLinkMateGeneric(other) {
    m_flipped = other.m_flipped;
}

void ChLinkMatePrismatic::SetFlipped(bool doflip) {
    if (doflip != m_flipped) {
        // swaps direction of Z axis by flipping 180 deg the frame A (slave)

        ChFrame<> frameRotator(VNULL, QuatFromAngleY(CH_PI));
        m_frame1.ConcatenatePostTransformation(frameRotator);

        m_flipped = doflip;
    }
}

void ChLinkMatePrismatic::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                     std::shared_ptr<ChBodyFrame> body2,
                                     bool pos_are_relative,
                                     const ChVector3d& point1,
                                     const ChVector3d& point2,
                                     const ChVector3d& dir1,
                                     const ChVector3d& dir2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, m_flipped ? -dir1 : dir1, dir2);
}

double ChLinkMatePrismatic::GetRelativePos() {
    ChFrame<> F1_W = m_frame1 >> *m_body1;
    ChFrame<> F2_W = m_frame2 >> *m_body2;
    ChVector3d pos12_F2 = F2_W.TransformDirectionParentToLocal(F1_W.GetPos() - F2_W.GetPos());
    return pos12_F2.z();  // NB: assumes translation along Z
}

double ChLinkMatePrismatic::GetRelativePosDt() {
    ChFrameMoving<> F1_W = ChFrameMoving<>(m_frame1) >> *m_body1;
    ChFrameMoving<> F2_W = ChFrameMoving<>(m_frame2) >> *m_body2;
    ChVector3d vel12_F2 = F2_W.TransformDirectionParentToLocal(F1_W.GetPosDt() - F2_W.GetPosDt());
    return vel12_F2.z();  // NB: assumes translation along Z
}

double ChLinkMatePrismatic::GetRelativePosDt2() {
    ChFrameMoving<> F1_W = ChFrameMoving<>(m_frame1) >> *m_body1;
    ChFrameMoving<> F2_W = ChFrameMoving<>(m_frame2) >> *m_body2;
    ChVector3d acc12_F2 = F2_W.TransformDirectionParentToLocal(F1_W.GetPosDt2() - F2_W.GetPosDt2());
    return acc12_F2.z();  // NB: assumes translation along Z
}

void ChLinkMatePrismatic::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMatePrismatic>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMatePrismatic::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMatePrismatic>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    archive_in >> CHNVP(m_flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateSpherical)

ChLinkMateSpherical::ChLinkMateSpherical(const ChLinkMateSpherical& other) : ChLinkMateGeneric(other) {}

void ChLinkMateSpherical::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                     std::shared_ptr<ChBodyFrame> body2,
                                     bool pos_are_relative,
                                     ChVector3d point1,
                                     ChVector3d point2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, VECT_Z, VECT_Z);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateDistanceZ)

ChLinkMateDistanceZ::ChLinkMateDistanceZ(const ChLinkMateDistanceZ& other) : ChLinkMateGeneric(other) {
    m_distance = other.m_distance;
}

void ChLinkMateDistanceZ::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                     std::shared_ptr<ChBodyFrame> body2,
                                     bool pos_are_relative,
                                     ChVector3d point1,
                                     ChVector3d point2,
                                     ChVector3d mdir2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, mdir2, mdir2);
}

void ChLinkMateDistanceZ::Update(double mtime, bool update_assets) {
    // Parent class inherit
    ChLinkMateGeneric::Update(mtime, update_assets);

    // .. then add the effect of imposed distance on C residual vector
    C(0) -= m_distance;  // for this mate, C = {Cz}
}

void ChLinkMateDistanceZ::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateDistanceZ>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_distance);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateDistanceZ::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateDistanceZ>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_distance);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateParallel)

ChLinkMateParallel::ChLinkMateParallel(const ChLinkMateParallel& other) : ChLinkMateGeneric(other) {
    m_flipped = other.m_flipped;
}

void ChLinkMateParallel::SetFlipped(bool doflip) {
    if (doflip != m_flipped) {
        // swaps direction of Z axis by flipping 180 deg the frame 1

        ChFrame<> frameRotator(VNULL, QuatFromAngleY(CH_PI));
        m_frame1.ConcatenatePostTransformation(frameRotator);

        m_flipped = doflip;
    }
}

void ChLinkMateParallel::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                    std::shared_ptr<ChBodyFrame> body2,
                                    bool pos_are_relative,
                                    const ChVector3d& point1,
                                    const ChVector3d& point2,
                                    const ChVector3d& dir1,
                                    const ChVector3d& dir2) {
    ChLinkMateGeneric::Initialize(body1, body2, pos_are_relative, point1, point2, m_flipped ? -dir1 : dir1, dir2);
}

void ChLinkMateParallel::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateParallel>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateParallel::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateParallel>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateOrthogonal)

ChLinkMateOrthogonal::ChLinkMateOrthogonal(const ChLinkMateOrthogonal& other) : ChLinkMateGeneric(other) {
    m_reldir1 = other.m_reldir1;
    m_reldir2 = other.m_reldir2;
}

void ChLinkMateOrthogonal::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                      std::shared_ptr<ChBodyFrame> body2,
                                      bool pos_are_relative,
                                      const ChVector3d& point1,
                                      const ChVector3d& point2,
                                      const ChVector3d& dir1,
                                      const ChVector3d& dir2) {
    // set the two frames so that they have the Z axis aligned
    if (pos_are_relative) {
        m_reldir1 = dir1;
        m_reldir2 = dir2;
    } else {
        m_reldir1 = body1->TransformDirectionParentToLocal(dir1);
        m_reldir2 = body2->TransformDirectionParentToLocal(dir2);
    }

    // do this asap otherwise the following Update() won't work..
    m_body1 = body1.get();
    m_body2 = body2.get();

    // Force the alignment of frames so that the Z axis is cross product of two dirs, etc.
    // by calling the custom update function of ChLinkMateOrthogonal.
    Update(ChTime, true);

    // Perform initialization (set pointers to variables, etc.)
    ChLinkMateGeneric::Initialize(body1, body2,
                                  true,  // recycle already-updated frames
                                  m_frame1, m_frame2);
}

/// Override _all_ time, jacobian etc. updating.
void ChLinkMateOrthogonal::Update(double mtime, bool update_assets) {
    // Prepare the alignment of the two frames so that the Z axis is orthogonal
    // to the two directions

    ChVector3d absdir1, absdir2;

    if (this->m_body1 && this->m_body2) {
        absdir1 = this->m_body1->TransformDirectionLocalToParent(m_reldir1);
        absdir2 = this->m_body2->TransformDirectionLocalToParent(m_reldir2);

        ChVector3d Zcomm = Vcross(absdir2, absdir1);
        bool succ = Zcomm.Normalize();

        // absdir1 and absdir2 resulted to be parallel;
        // this is possible if bodies are in specific violating configuration
        if (!succ) {
            ChVector3d ortho_gen(0);

            for (auto i = 0; i < 3; ++i) {
                ortho_gen[i] = 1;
                Zcomm = Vcross(absdir1, ortho_gen);
                succ = Zcomm.Normalize();
                if (succ)
                    break;
            }
            assert(succ &&
                   "Developer error: the algorithm above should have found a way to create a Zcomm vector orthogonal "
                   "to absdir1, but didn't.");
        }

        // set R1 to have Z as common axis, Y as absdir1
        ChVector3d X1, Y1;
        ChMatrix33<> R1;
        Y1 = absdir1;
        X1 = Vcross(Y1, Zcomm);
        R1.SetFromDirectionAxes(X1, Y1, Zcomm);

        // set R2 to have Z as common axis, X as absdir2
        ChVector3d X2, Y2;
        ChMatrix33<> R2;
        X2 = absdir2;
        Y2 = Vcross(Zcomm, X2);
        R2.SetFromDirectionAxes(X2, Y2, Zcomm);

        ChFrame<> absframe1(VNULL, R1);  // position not needed for orth. constr. computation
        ChFrame<> absframe2(VNULL, R2);  // position not needed for orth. constr. computation

        // from abs to body-rel
        m_frame1 = static_cast<ChFrame<>*>(this->m_body1)->TransformParentToLocal(absframe1);
        m_frame2 = static_cast<ChFrame<>*>(this->m_body2)->TransformParentToLocal(absframe2);
    }

    // Parent class inherit
    ChLinkMateGeneric::Update(mtime, update_assets);
}

void ChLinkMateOrthogonal::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateOrthogonal>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_reldir1);
    archive_out << CHNVP(m_reldir2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateOrthogonal::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateOrthogonal>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_reldir1);
    archive_in >> CHNVP(m_reldir2);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateFix)

ChLinkMateFix::ChLinkMateFix(const ChLinkMateFix& other) : ChLinkMateGeneric(other) {}

void ChLinkMateFix::Initialize(std::shared_ptr<ChBodyFrame> body1, std::shared_ptr<ChBodyFrame> body2) {
    ChLinkMateGeneric::Initialize(
        body1, body2,
        false,              // constraint reference frame in abs coords
        ChFrame<>(*body1),  // defaults to have constraint reference frame as body1 coordsystem
        ChFrame<>(*body1)   // defaults to have constraint reference frame as body1 coordsystem
    );
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateRackPinion)

ChLinkMateRackPinion::ChLinkMateRackPinion()
    : ChLinkMateGeneric(true, false, false, false, false, false),
      R(0.1),
      alpha(0),
      beta(0),
      phase(0),
      checkphase(false),
      a1(0),
      contact_pt(VNULL) {
    local_pinion.SetIdentity();
    local_rack.SetIdentity();
}

ChLinkMateRackPinion::ChLinkMateRackPinion(const ChLinkMateRackPinion& other) : ChLinkMateGeneric(other) {
    R = other.R;
    alpha = other.alpha;
    beta = other.beta;
    phase = other.phase;
    a1 = other.a1;
    checkphase = other.checkphase;

    contact_pt = other.contact_pt;
    local_pinion = other.local_pinion;
    local_rack = other.local_rack;
}

ChVector3d ChLinkMateRackPinion::GetAbsPinionDir() {
    if (this->m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_pinion);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkMateRackPinion::GetAbsPinionPos() {
    if (this->m_body1) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_pinion);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector3d ChLinkMateRackPinion::GetAbsRackDir() {
    if (this->m_body2) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_rack);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkMateRackPinion::GetAbsRackPos() {
    if (this->m_body2) {
        ChFrame<double> absframe = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_rack);
        return absframe.GetPos();
    } else
        return VNULL;
}

void ChLinkMateRackPinion::Update(double time, bool update_assets) {
    // First, inherit to parent class
    ChLinkMateGeneric::Update(time, update_assets);

    ChFrame<double> abs_pinion = ((ChFrame<double>*)m_body1)->TransformLocalToParent(local_pinion);
    ChFrame<double> abs_rack = ((ChFrame<double>*)m_body2)->TransformLocalToParent(local_rack);

    ChVector3d abs_distpr = abs_pinion.GetPos() - abs_rack.GetPos();  // vector from rack to pinion
    ChVector3d abs_Dpin = abs_pinion.GetRotMat().GetAxisZ();          // direction of pinion shaft
    ChVector3d abs_Dx;
    ChVector3d abs_Dy;
    ChVector3d abs_Dz;
    abs_Dpin.GetDirectionAxesAsX(
        abs_Dz, abs_Dx, abs_Dy,
        abs_rack.GetRotMat().GetAxisX());  // with z as pinion shaft and x as suggested rack X dir

    /*
    std::cout << "abs_distpr " << abs_distpr << std::endl;
    std::cout << "abs_rack Xaxis()" << abs_rack.GetRotMat()->GetAxisX() << std::endl;
    std::cout << "abs_Dpin " << abs_Dpin << std::endl;
    std::cout << "abs_Dx " << abs_Dx << std::endl;
    */

    ChVector3d abs_Ro = abs_Dy * Vdot(abs_Dy, abs_distpr);

    if (Vdot(abs_Ro, abs_distpr) > 0)
        abs_Ro = -abs_Ro;

    ChVector3d abs_Dr = abs_Ro.GetNormalized();
    ChVector3d abs_R = abs_Dr * this->GetPinionRadius();
    this->contact_pt = abs_pinion.GetPos() + abs_R;

    // Absolute frame of link reference
    ChMatrix33<> ma1(abs_Dx, abs_Dy, abs_Dz);
    ChFrame<> abs_contact(this->contact_pt, ma1);

    ChMatrix33<> mrot;

    // rotate link frame on its Y because of beta
    mrot.SetFromCardanAnglesXYZ(ChVector3d(0, this->beta, 0));
    ChFrame<> mrotframe(VNULL, mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // rotate link frame on its Z because of alpha
    if (this->react_force.x() < 0)
        mrot.SetFromCardanAnglesXYZ(ChVector3d(0, 0, this->alpha));
    else
        mrot.SetFromCardanAnglesXYZ(ChVector3d(0, 0, -this->alpha));
    mrotframe.SetRot(mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // Set the link frame 'abs_contact' to relative frames to the two connected ChBodyFrame
    m_frame1 = ((ChFrame<double>*)m_body1)->TransformParentToLocal(abs_contact);
    m_frame2 = ((ChFrame<double>*)m_body2)->TransformParentToLocal(abs_contact);
}

void ChLinkMateRackPinion::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMateRackPinion>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(R);
    archive_out << CHNVP(alpha);
    archive_out << CHNVP(beta);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(checkphase);
    archive_out << CHNVP(a1);
    archive_out << CHNVP(local_pinion);
    archive_out << CHNVP(local_rack);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateRackPinion::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMateRackPinion>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(R);
    archive_in >> CHNVP(alpha);
    archive_in >> CHNVP(beta);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(checkphase);
    archive_in >> CHNVP(a1);
    archive_in >> CHNVP(local_pinion);
    archive_in >> CHNVP(local_rack);
}

}  // end namespace chrono
