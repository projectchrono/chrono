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

#include "chrono/physics/ChLinkMotorRotationAngle.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationAngle)

ChLinkMotorRotationAngle::ChLinkMotorRotationAngle() {
    this->c_rz = true;
    SetupLinkMask();

    // default motion function
    m_func = chrono_types::make_shared<ChFunctionConst>(0.0);

    rot_offset = 0;
}

ChLinkMotorRotationAngle::ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other) : ChLinkMotorRotation(other) {
    rot_offset = other.rot_offset;
}

ChLinkMotorRotationAngle::~ChLinkMotorRotationAngle() {}

void ChLinkMotorRotationAngle::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorRotation::Update(time, update_assets);

    // Override the rotational jacobian [Cq] and the rotational residual C,
    // by assuming an additional hidden frame that rotates about frame1:

    if (this->m_body1 && this->m_body2) {
        ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
        ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);

        // ChFrame<> aframe12 = aframe2.TransformParentToLocal(aframe1);

        double aux_rotation;
        aux_rotation = m_func->GetVal(time) + rot_offset;

        ChFrame<> aframe1rotating;
        aframe1rotating.SetPos(aframe1.GetPos());  // for safe
        aframe1rotating.SetRot(aframe1.GetRot() * QuatFromAngleZ(aux_rotation).GetConjugate());

        ChFrame<> aframe1rotating2 = aframe2.TransformParentToLocal(aframe1rotating);

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        this->P = 0.5 * (ChMatrix33<>(aframe1rotating2.GetRot().e0()) +
                         ChStarMatrix33<>(aframe1rotating2.GetRot().GetVector()));

        ChMatrix33<> Jw1 = this->P.transpose() * aframe2.GetRotMat().transpose() * m_body1->GetRotMat();
        ChMatrix33<> Jw2 = -this->P.transpose() * aframe2.GetRotMat().transpose() * m_body2->GetRotMat();

        // Another equivalent expression:
        // ChMatrix33<> Jw1 = this->P * aframe1rotating.GetRotMat().transpose() * m_body1->GetRotMat();
        // ChMatrix33<> Jw2 = -this->P * aframe1rotating.GetRotMat().transpose() * m_body2->GetRotMat();

        int nc = 0;

        if (c_x) {
            nc++;
        }
        if (c_y) {
            nc++;
        }
        if (c_z) {
            nc++;
        }
        if (c_rx) {
            C(nc) = aframe1rotating2.GetRot().e1();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = aframe1rotating2.GetRot().e2();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = aframe1rotating2.GetRot().e3();
            mask.GetConstraint(nc).Get_Cq_a().setZero();
            mask.GetConstraint(nc).Get_Cq_b().setZero();
            mask.GetConstraint(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.GetConstraint(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
    }
}

void ChLinkMotorRotationAngle::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
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
        ChVector3d P12_B2 = R_B2_W.transpose() * (F1_W.GetPos() - F2_W.GetPos());
        // ChFrame<> F1_wrt_F2;
        // F2_W.TransformParentToLocal(F1_W, F1_wrt_F2);

        ChVector3d r_F1_B1 = m_frame1.GetPos();
        ChVector3d r_F2_B2 = m_frame2.GetPos();
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

        // Recover the quaternion of the shadow frame 'aframe1rotating2' from the projection matrix this->P
        ChQuaternion<> q_F1M_F2;
        q_F1M_F2.e0() = 2.0 * this->P(0, 0);
        q_F1M_F2.e1() = -2.0 * this->P(1, 2);
        q_F1M_F2.e2() = 2.0 * this->P(0, 2);
        q_F1M_F2.e3() = -2.0 * this->P(0, 1);
        double s_F1M_F2 = q_F1M_F2.e0();
        ChVector3d v_F1M_F2 = q_F1M_F2.GetVector();
        ChMatrix33<> I33;
        I33.setIdentity();
        ChMatrix33<> G = -0.25 * TensorProduct(gamma_m, v_F1M_F2) -
                         0.25 * ChStarMatrix33<>(gamma_m) * (s_F1M_F2 * I33 + ChStarMatrix33<>(v_F1M_F2));

        // Stabilization part
        ChMatrixDynamic<> Ks;
        Ks.setZero(12, 12);
        ChMatrix33<> R_F1M_W = R_F2_W * ChMatrix33<>(q_F1M_F2);
        Ks.block<3, 3>(3, 3) = R_B1_W.transpose() * R_F2_W * G * R_F1M_W.transpose() * R_B1_W;
        Ks.block<3, 3>(3, 9) = -R_B1_W.transpose() * R_F2_W * G * R_F1M_W.transpose() * R_B2_W;
        Ks.block<3, 3>(9, 3) = -R_B2_W.transpose() * R_F2_W * G * R_F1M_W.transpose() * R_B1_W;
        Ks.block<3, 3>(9, 9) = R_B2_W.transpose() * R_F2_W * G * R_F1M_W.transpose() * R_B2_W;

        // The complete tangent stiffness matrix
        this->Kmatr->GetMatrix() = (Km + Ks) * Kfactor;
    }
}

void ChLinkMotorRotationAngle::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -0.5 * m_func->GetDer(this->GetChTime());
    unsigned int ncrz = mask.GetNumConstraints() - 1;
    if (mask.GetConstraint(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt;
    }
}

void ChLinkMotorRotationAngle::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -0.5 * m_func->GetDer(this->GetChTime());
    unsigned int ncrz = mask.GetNumConstraints() - 1;
    if (mask.GetConstraint(ncrz).IsActive()) {
        mask.GetConstraint(ncrz).SetRightHandSide(mask.GetConstraint(ncrz).GetRightHandSide() + factor * mCt);
    }
}

void ChLinkMotorRotationAngle::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorRotationAngle>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(rot_offset);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationAngle::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorRotationAngle>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(rot_offset);
}

}  // end namespace chrono
