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
    // by assuming an additional hidden frame that rotates about frame2:

    if (this->Body1 && this->Body2) {
        ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
        ChFrame<> aframe2 = this->frame2 >> (*this->Body2);

        ChFrame<> aframe12;
        aframe2.TransformParentToLocal(aframe1, aframe12);

        ChFrame<> aframe2rotating;

        double aux_rotation;

        aux_rotation = m_func->Get_y(mytime) + rot_offset;

        aframe2rotating.SetRot(aframe2.GetRot() * Q_from_AngAxis(aux_rotation, VECT_Z));

        ChFrame<> aframe12rotating;
        aframe2rotating.TransformParentToLocal(aframe1, aframe12rotating);

        ChMatrix33<> abs_plane_rotating = aframe2rotating.GetA();

        ChMatrix33<> Jw1 = abs_plane_rotating.transpose() * Body1->GetA();
        ChMatrix33<> Jw2 = -abs_plane_rotating.transpose() * Body2->GetA();

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        ChStarMatrix33<> mtempM(aframe12rotating.GetRot().GetVector() * 0.5);
        mtempM(0, 0) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(1, 1) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(2, 2) = 0.5 * aframe12rotating.GetRot().e0();

        ChMatrix33<> mtempQ;
        mtempQ = mtempM.transpose() * Jw1;
        Jw1 = mtempQ;
        mtempQ = mtempM.transpose() * Jw2;
        Jw2 = mtempQ;

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
            C(nc) = aframe12rotating.GetRot().e1();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = aframe12rotating.GetRot().e2();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = aframe12rotating.GetRot().e3();
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
    int version = marchive.VersionRead<ChLinkMotorRotationAngle>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(rot_offset);
}

}  // end namespace chrono
