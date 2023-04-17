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

        ChFrame<> aframe12;
        aframe2.TransformParentToLocal(aframe1, aframe12);

        double aux_rotation;
        aux_rotation = m_func->Get_y(mytime) + rot_offset;

        ChFrame<> aframe1rotating;
        aframe1rotating.SetRot(aframe1.GetRot() * Q_from_AngAxis(aux_rotation, VECT_Z).GetConjugate());

        ChFrame<> aframe21rotating;
        aframe1rotating.TransformParentToLocal(aframe2, aframe21rotating);

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        this->P = 0.5 * (ChMatrix33<>(aframe21rotating.GetRot().e0()) +
                         ChStarMatrix33<>(aframe21rotating.GetRot().GetVector()));

        ChMatrix33<> Jw1 = this->P * aframe2.GetA().transpose() * Body1->GetA();
        ChMatrix33<> Jw2 = -this->P * aframe2.GetA().transpose() * Body2->GetA();

        // Another equivalent expression:
        // ChMatrix33<> Jw1 = this->P.transpose() * aframe1rotating.GetA().transpose() * Body1->GetA();
        // ChMatrix33<> Jw2 = -this->P.transpose() * aframe1rotating.GetA().transpose() * Body2->GetA();

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
            C(nc) = -aframe21rotating.GetRot().e1();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = -aframe21rotating.GetRot().e2();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = -aframe21rotating.GetRot().e3();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
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

void ChLinkMotorRotationAngle::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationAngle>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(rot_offset);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationAngle::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorRotationAngle>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(rot_offset);
}

}  // end namespace chrono
