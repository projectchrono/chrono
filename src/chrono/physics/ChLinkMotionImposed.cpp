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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/functions/ChFunctionRotationABCFunctions.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotionImposed)

ChLinkMotionImposed::ChLinkMotionImposed() : ChLinkMateGeneric(true, true, true, true, true, true) {
    // default motion and rotation: no rotation no translation
    position_function = chrono_types::make_shared<ChFunctionPositionXYZFunctions>();
    rotation_function = chrono_types::make_shared<ChFunctionRotationABCFunctions>();
}

ChLinkMotionImposed::ChLinkMotionImposed(const ChLinkMotionImposed& other) : ChLinkMateGeneric(other) {
    position_function = other.position_function;
    rotation_function = other.rotation_function;
}

ChLinkMotionImposed::~ChLinkMotionImposed() {}

void ChLinkMotionImposed::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMateGeneric::Update(time, update_assets);

    // Override the rotational jacobian [Cq] and the rotational residual C,
    // by assuming an additional hidden frame that rotates about frame2:

    if (this->m_body1 && this->m_body2) {
        ChFrame<> frame1W = m_frame1 >> (*this->m_body1);
        ChFrame<> frame2W = m_frame2 >> (*this->m_body2);

        frameM2.SetRot(rotation_function->GetQuat(time));
        frameM2.SetPos(position_function->GetPos(time));

        frameMb2 = frameM2 >> m_frame2;

        ChFrame<> frameMW = frameM2 >> frame2W;  // "moving" auxiliary frame M which is coincident with frame1

        ChFrame<> frame1M = frameMW.TransformParentToLocal(frame1W);
        // Now, frame1M is the relative frame of frame1 respect to the "moving" auxiliary frame M,
        // which should be a unit frame (VNULL,QUNIT) in case of constraint satisfied.

        ChMatrix33<> planeMW = frameMW.GetRotMat();

        ChMatrix33<> Jx1 = planeMW.transpose();
        ChMatrix33<> Jx2 = -planeMW.transpose();

        ChMatrix33<> Jr1 = -planeMW.transpose() * m_body1->GetRotMat() * ChStarMatrix33<>(m_frame1.GetPos());
        ChMatrix33<> Jr2 = planeMW.transpose() * m_body2->GetRotMat() * ChStarMatrix33<>(frameMb2.GetPos());

        ChVector3d p2p1_base2 = m_body2->GetRotMat().transpose() * (frame1W.GetPos() - frameMW.GetPos());
        Jr2 += planeMW.transpose() * m_body2->GetRotMat() * ChStarMatrix33<>(p2p1_base2);

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        this->P = 0.5 * (ChMatrix33<>(frame1M.GetRot().e0()) + ChStarMatrix33<>(frame1M.GetRot().GetVector()));

        ChMatrix33<> Jw1 = this->P * frame1W.GetRotMat().transpose() * m_body1->GetRotMat();
        ChMatrix33<> Jw2 = -this->P * frame1W.GetRotMat().transpose() * m_body2->GetRotMat();

        // Another equivalent expression:
        // ChMatrix33<> Jw1 = this->P.transpose() * planeMW.transpose() * m_body1->GetRotMat();
        // ChMatrix33<> Jw2 = -this->P.transpose() * planeMW.transpose() * m_body2->GetRotMat();

        int nc = 0;

        if (c_x) {
            C(nc) = frame1M.GetPos().x();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(0);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(0);
            nc++;
        }
        if (c_y) {
            C(nc) = frame1M.GetPos().y();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(1);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(1);
            nc++;
        }
        if (c_z) {
            C(nc) = frame1M.GetPos().z();
            mask.GetConstraint(nc).Get_Cq_a().segment(0, 3) = Jx1.row(2);
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jr1.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(0, 3) = Jx2.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jr2.row(2);
            nc++;
        }
        if (c_rx) {
            C(nc) = frame1M.GetRot().e1();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = frame1M.GetRot().e2();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = frame1M.GetRot().e3();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
    }
}

void ChLinkMotionImposed::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (!this->IsActive())
        return;

    if (this->Kmatr) {
        // The algorithm is quite similar as ChLinkMateGeneric(),
        // just replacing F2_W with the "moving" auxiliary frame M here.
        ChFrame<> F1_W = m_frame1 >> (*this->m_body1);
        ChFrame<> frame2W = m_frame2 >> (*this->m_body2);
        frameMb2 = frameM2 >> m_frame2;
        ChFrame<> F2_W = frameM2 >> frame2W;  // "moving" auxiliary frame M which is coincident with frame1

        ChMatrix33<> R_B1_W = m_body1->GetRotMat();
        ChMatrix33<> R_B2_W = m_body2->GetRotMat();
        ChMatrix33<> R_F1_W = F1_W.GetRotMat();
        ChMatrix33<> R_F2_W = F2_W.GetRotMat();
        ChVector3d P12_B2 = R_B2_W.transpose() * (F1_W.GetPos() - F2_W.GetPos());
        ChFrame<> F1_wrt_F2 = F2_W.TransformParentToLocal(F1_W);

        ChVector3d r_F1_B1 = m_frame1.GetPos();
        ChVector3d r_F2_B2 = this->frameMb2.GetPos();
        ChStarMatrix33<> rtilde_F1_B1(r_F1_B1);
        ChStarMatrix33<> rtilde_F2_B2(r_F2_B2);

        // Main part
        ChMatrixDynamic<> Km;
        Km.setZero(12, 12);
        Km.block<3, 3>(0, 9) = -R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B2_W;
        Km.block<3, 3>(3, 3) =
            rtilde_F1_B1 * R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B1_W +
            R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(this->P * gamma_m) * R_F2_W.transpose() * R_B1_W;
        Km.block<3, 3>(3, 9) =
            -rtilde_F1_B1 * R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B2_W -
            R_B1_W.transpose() * R_F2_W * ChStarMatrix33<>(this->P * gamma_m) * R_F2_W.transpose() * R_B2_W;
        Km.block<3, 3>(6, 9) = R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B2_W;

        Km.block<3, 3>(9, 0) = R_B2_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose();
        Km.block<3, 3>(9, 3) =
            -R_B2_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B1_W * rtilde_F1_B1;
        Km.block<3, 3>(9, 6) = -R_B2_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose();
        Km.block<3, 3>(9, 9) = R_B2_W.transpose() * R_F2_W * ChStarMatrix33<>(gamma_f) * R_F2_W.transpose() * R_B2_W *
                               ChStarMatrix33<>(P12_B2 + r_F2_B2);

        double s_F1_F2 = F1_wrt_F2.GetRot().e0();
        ChVector3d v_F1_F2 = F1_wrt_F2.GetRot().GetVector();
        ChMatrix33<> I33;
        I33.setIdentity();
        ChMatrix33<> G = -0.25 * TensorProduct(gamma_m, v_F1_F2) -
                         0.25 * ChStarMatrix33<>(gamma_m) * (s_F1_F2 * I33 + ChStarMatrix33<>(v_F1_F2));

        // Stabilization part
        ChMatrixDynamic<> Ks;
        Ks.setZero(12, 12);
        Ks.block<3, 3>(3, 3) = R_B1_W.transpose() * R_F2_W * G * R_F1_W.transpose() * R_B1_W;
        Ks.block<3, 3>(3, 9) = -R_B1_W.transpose() * R_F2_W * G * R_F1_W.transpose() * R_B2_W;
        Ks.block<3, 3>(9, 3) = -R_B2_W.transpose() * R_F2_W * G * R_F1_W.transpose() * R_B1_W;
        Ks.block<3, 3>(9, 9) = R_B2_W.transpose() * R_F2_W * G * R_F1_W.transpose() * R_B2_W;

        // The complete tangent stiffness matrix
        this->Kmatr->GetMatrix() = (Km + Ks) * Kfactor;
    }
}

void ChLinkMotionImposed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double T = this->GetChTime();
    ChVector3d mv = -position_function->GetLinVel(T);
    ChVector3d mw_loc = -rotation_function->GetAngVel(T);
    ChVector3d mv_rot = rotation_function->GetQuat(T).RotateBack(mv);  // need velocity in local rotated system

    int nc = 0;
    if (c_x) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * mv_rot.x();
        nc++;
    }
    if (c_y) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * mv_rot.y();
        nc++;
    }
    if (c_z) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * mv_rot.z();
        nc++;
    }
    if (c_rx) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.x();
        nc++;
    }
    if (c_ry) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.y();
        nc++;
    }
    if (c_rz) {
        if (mask.GetConstraint(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.z();
        nc++;
    }
}

// OLD
void ChLinkMotionImposed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double T = this->GetChTime();
    ChVector3d mv = -position_function->GetLinVel(T);
    ChVector3d mw_loc = -rotation_function->GetAngVel(T);
    ChVector3d mv_rot = rotation_function->GetQuat(T).RotateBack(mv);  // need velocity in local rotated system

    if (mask.GetConstraint(0).IsActive()) {
        mask.GetConstraint(0).SetRightHandSide(mask.GetConstraint(0).GetRightHandSide() + factor * mv_rot.x());
    }
    if (mask.GetConstraint(1).IsActive()) {
        mask.GetConstraint(1).SetRightHandSide(mask.GetConstraint(1).GetRightHandSide() + factor * mv_rot.y());
    }
    if (mask.GetConstraint(2).IsActive()) {
        mask.GetConstraint(2).SetRightHandSide(mask.GetConstraint(2).GetRightHandSide() + factor * mv_rot.z());
    }
    if (mask.GetConstraint(3).IsActive()) {
        mask.GetConstraint(3).SetRightHandSide(mask.GetConstraint(3).GetRightHandSide() + factor * 0.5 * mw_loc.x());
    }
    if (mask.GetConstraint(4).IsActive()) {
        mask.GetConstraint(4).SetRightHandSide(mask.GetConstraint(4).GetRightHandSide() + factor * 0.5 * mw_loc.y());
    }
    if (mask.GetConstraint(5).IsActive()) {
        mask.GetConstraint(5).SetRightHandSide(mask.GetConstraint(5).GetRightHandSide() + factor * 0.5 * mw_loc.z());
    }
}

void ChLinkMotionImposed::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotionImposed>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(position_function);
    archive_out << CHNVP(rotation_function);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotionImposed::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotionImposed>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(position_function);
    archive_in >> CHNVP(rotation_function);
}

}  // end namespace chrono
