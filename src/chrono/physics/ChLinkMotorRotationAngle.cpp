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
    // default motion function: ramp with initial value y(0) = 0 and slope dy/dt = 1
    m_func = chrono_types::make_shared<ChFunction_Ramp>(0.0, 1.0);

    rot_offset = 0;
}

ChLinkMotorRotationAngle::ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other) : ChLinkMotorRotation(other) {
    rot_offset = other.rot_offset;
}

ChLinkMotorRotationAngle::~ChLinkMotorRotationAngle() {}

void ChLinkMotorRotationAngle::Update(double mytime, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    // Override the rotational jacobian [Cq] and the rotational residual C,
    // by assuming an additional hidden frame that rotates about frame1:

    if (this->Body1 && this->Body2) {
        ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
        ChFrame<> aframe2 = this->frame2 >> (*this->Body2);

        //ChFrame<> aframe12;
        //aframe2.TransformParentToLocal(aframe1, aframe12);

        double aux_rotation;
        aux_rotation = m_func->Get_y(mytime) + rot_offset;

        ChFrame<> aframe1rotating;
        aframe1rotating.SetPos(aframe1.GetPos()); // for safe
        aframe1rotating.SetRot(aframe1.GetRot() * Q_from_AngAxis(aux_rotation, VECT_Z).GetConjugate());

        ChFrame<> aframe1rotating2;
        aframe2.TransformParentToLocal(aframe1rotating, aframe1rotating2);

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        this->P = 0.5 * (ChMatrix33<>(aframe1rotating2.GetRot().e0()) +
                         ChStarMatrix33<>(aframe1rotating2.GetRot().GetVector()));

        ChMatrix33<> Jw1 = this->P.transpose() * aframe2.GetA().transpose() * Body1->GetA();
        ChMatrix33<> Jw2 = -this->P.transpose() * aframe2.GetA().transpose() * Body2->GetA();

        // Another equivalent expression:
        // ChMatrix33<> Jw1 = this->P * aframe1rotating.GetA().transpose() * Body1->GetA();
        // ChMatrix33<> Jw2 = -this->P * aframe1rotating.GetA().transpose() * Body2->GetA();

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
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = aframe1rotating2.GetRot().e2();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = aframe1rotating2.GetRot().e3();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
    }
}


void ChLinkMotorRotationAngle::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (!this->IsActive())
        return;

    if (this->Kmatr) {
        ChMatrix33<> R_B1_W = Body1->GetA();
        ChMatrix33<> R_B2_W = Body2->GetA();
        // ChMatrix33<> R_F1_B1 = frame1.GetA();
        // ChMatrix33<> R_F2_B2 = frame2.GetA();
        ChFrame<> F1_W = this->frame1 >> (*this->Body1);
        ChFrame<> F2_W = this->frame2 >> (*this->Body2);
        ChMatrix33<> R_F1_W = F1_W.GetA();
        ChMatrix33<> R_F2_W = F2_W.GetA();
        ChVector<> P12_B2 = R_B2_W.transpose() * (F1_W.GetPos() - F2_W.GetPos());
        //ChFrame<> F1_wrt_F2;
        //F2_W.TransformParentToLocal(F1_W, F1_wrt_F2);

        ChVector<> r_F1_B1 = this->frame1.GetPos();
        ChVector<> r_F2_B2 = this->frame2.GetPos();
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
        ChVector<> v_F1M_F2 = q_F1M_F2.GetVector();
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
        this->Kmatr->Get_K() = (Km + Ks) * Kfactor;
    }
}

void ChLinkMotorRotationAngle::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -0.5 * m_func->Get_y_dx(this->GetChTime());
    int ncrz = mask.nconstr - 1;
    if (mask.Constr_N(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt;
    }
}

void ChLinkMotorRotationAngle::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -0.5 * m_func->Get_y_dx(this->GetChTime());
    int ncrz = mask.nconstr - 1;
    if (mask.Constr_N(ncrz).IsActive()) {
        mask.Constr_N(ncrz).Set_b_i(mask.Constr_N(ncrz).Get_b_i() + factor * mCt);
    }
}

void ChLinkMotorRotationAngle::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationAngle>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(rot_offset);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationAngle::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorRotationAngle>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(rot_offset);
}

}  // end namespace chrono
