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

#include "chrono/physics/ChLinkMotorRotationSpeed.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationSpeed)

ChLinkMotorRotationSpeed::ChLinkMotorRotationSpeed() {
    variable.GetMass()(0, 0) = 1.0;
    variable.GetInvMass()(0, 0) = 1.0;

    this->c_rz = true;
    SetupLinkMask();

    m_func = chrono_types::make_shared<ChFunctionConst>(0.0);

    rot_offset = 0;

    aux_dt = 0;  // used for integrating speed, = rot
    aux_dtdt = 0;

    avoid_angle_drift = true;
}

ChLinkMotorRotationSpeed::ChLinkMotorRotationSpeed(const ChLinkMotorRotationSpeed& other) : ChLinkMotorRotation(other) {
    variable = other.variable;
    rot_offset = other.rot_offset;
    aux_dt = other.aux_dt;
    aux_dtdt = other.aux_dtdt;
    avoid_angle_drift = other.avoid_angle_drift;
}

ChLinkMotorRotationSpeed::~ChLinkMotorRotationSpeed() {}

void ChLinkMotorRotationSpeed::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorRotation::Update(time, update_assets);

    // Override the rotational jacobian [Cq] and the rotational residual C,
    // by assuming an additional hidden frame that rotates about frame1:

    if (this->m_body1 && this->m_body2) {
        ChFrame<> aframe1 = m_frame1 >> (*this->m_body1);
        ChFrame<> aframe2 = m_frame2 >> (*this->m_body2);

        double aux_rotation;

        if (this->avoid_angle_drift) {
            aux_rotation = this->aux_dt + this->rot_offset;
        } else {
            ChFrame<> aframe12 = aframe2.TransformParentToLocal(aframe1);

            // to have it aligned to current rot, to allow C=0.
            aux_rotation = aframe12.GetRot().GetRotVec().z();
        }

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

void ChLinkMotorRotationSpeed::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
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
        // ChFrame<> F1_wrt_F2 = F2_W.TransformParentToLocal(F1_W);

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

void ChLinkMotorRotationSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -0.5 * m_func->GetVal(this->GetChTime());

    unsigned int ncrz = mask.GetNumConstraints() - 1;
    if (mask.GetConstraint(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt;
    }
}

void ChLinkMotorRotationSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -0.5 * m_func->GetVal(this->GetChTime());
    unsigned int ncrz = mask.GetNumConstraints() - 1;
    if (mask.GetConstraint(ncrz).IsActive()) {
        mask.GetConstraint(ncrz).SetRightHandSide(mask.GetConstraint(ncrz).GetRightHandSide() + factor * mCt);
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMotorRotationSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                              ChState& x,                // state vector, position part
                                              const unsigned int off_v,  // offset in v state vector
                                              ChStateDelta& v,           // state vector, speed part
                                              double& T                  // time
) {
    x(off_x) = 0;  // aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChLinkMotorRotationSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                               const ChState& x,          // state vector, position part
                                               const unsigned int off_v,  // offset in v state vector
                                               const ChStateDelta& v,     // state vector, speed part
                                               const double T,            // time
                                               bool full_update           // perform complete update
) {
    // aux = x(off_x);
    aux_dt = v(off_v);

    Update(T, full_update);
}

void ChLinkMotorRotationSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChLinkMotorRotationSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChLinkMotorRotationSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                                 ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                                 const double c           // a scaling factor
) {
    double imposed_speed = m_func->GetVal(this->GetChTime());
    R(off) += imposed_speed * c;
}

void ChLinkMotorRotationSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                                  ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                                  const ChVectorDynamic<>& w,  // the w vector
                                                  const double c               // a scaling factor
) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorRotationSpeed::IntLoadLumpedMass_Md(const unsigned int off,
                                                    ChVectorDynamic<>& Md,
                                                    double& err,
                                                    const double c) {
    Md(off) += c * 1.0;
}

void ChLinkMotorRotationSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                               const ChStateDelta& v,
                                               const ChVectorDynamic<>& R,
                                               const unsigned int off_L,  // offset in L, Qc
                                               const ChVectorDynamic<>& L,
                                               const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorRotation::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    this->variable.State()(0, 0) = v(off_v);
    this->variable.Force()(0, 0) = R(off_v);
}

void ChLinkMotorRotationSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                                 ChStateDelta& v,
                                                 const unsigned int off_L,  // offset in L
                                                 ChVectorDynamic<>& L) {
    // inherit parent
    ChLinkMotorRotation::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.State()(0, 0);
}

////
void ChLinkMotorRotationSpeed::InjectVariables(ChSystemDescriptor& descriptor) {
    variable.SetDisabled(!IsActive());

    descriptor.InsertVariables(&variable);
}

void ChLinkMotorRotationSpeed::VariablesFbReset() {
    variable.Force().setZero();
}

void ChLinkMotorRotationSpeed::VariablesFbLoadForces(double factor) {
    double imposed_speed = m_func->GetVal(this->GetChTime());
    variable.Force()(0) += imposed_speed * factor;
}

void ChLinkMotorRotationSpeed::VariablesFbIncrementMq() {
    variable.AddMassTimesVector(variable.Force(), variable.State());
}

void ChLinkMotorRotationSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.State()(0) = aux_dt;
}

void ChLinkMotorRotationSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.State()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}

void ChLinkMotorRotationSpeed::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorRotationSpeed>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(rot_offset);
    archive_out << CHNVP(avoid_angle_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationSpeed::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorRotationSpeed>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(rot_offset);
    archive_in >> CHNVP(avoid_angle_drift);
}

}  // end namespace chrono
