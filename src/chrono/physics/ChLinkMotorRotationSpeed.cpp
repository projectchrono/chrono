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

    m_func = chrono_types::make_shared<ChFunction_Const>(1.0);

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

void ChLinkMotorRotationSpeed::Update(double mytime, bool update_assets) {
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

        if (this->avoid_angle_drift) {
            aux_rotation = this->aux_dt + this->rot_offset;
        } else {
            // to have it aligned to current rot, to allow C=0.
            aux_rotation = aframe12.GetRot().Q_to_Rotv().z();
        }

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

void ChLinkMotorRotationSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -0.5 * m_func->Get_y(this->GetChTime());

    int ncrz = mask.nconstr - 1;
    if (mask.Constr_N(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt;
    }
}

void ChLinkMotorRotationSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -0.5 * m_func->Get_y(this->GetChTime());
    int ncrz = mask.nconstr - 1;
    if (mask.Constr_N(ncrz).IsActive()) {
        mask.Constr_N(ncrz).Set_b_i(mask.Constr_N(ncrz).Get_b_i() + factor * mCt);
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
    double imposed_speed = m_func->Get_y(this->GetChTime());
    R(off) += imposed_speed * c;
}

void ChLinkMotorRotationSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                                  ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                                  const ChVectorDynamic<>& w,  // the w vector
                                                  const double c               // a scaling factor
) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorRotationSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                               const ChStateDelta& v,
                                               const ChVectorDynamic<>& R,
                                               const unsigned int off_L,  // offset in L, Qc
                                               const ChVectorDynamic<>& L,
                                               const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorRotation::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    this->variable.Get_qb()(0, 0) = v(off_v);
    this->variable.Get_fb()(0, 0) = R(off_v);
}

void ChLinkMotorRotationSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                                 ChStateDelta& v,
                                                 const unsigned int off_L,  // offset in L
                                                 ChVectorDynamic<>& L) {
    // inherit parent
    ChLinkMotorRotation::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.Get_qb()(0, 0);
}

////
void ChLinkMotorRotationSpeed::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variable.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variable);
}

void ChLinkMotorRotationSpeed::VariablesFbReset() {
    variable.Get_fb().setZero();
}

void ChLinkMotorRotationSpeed::VariablesFbLoadForces(double factor) {
    double imposed_speed = m_func->Get_y(this->GetChTime());
    variable.Get_fb()(0) += imposed_speed * factor;
}

void ChLinkMotorRotationSpeed::VariablesFbIncrementMq() {
    variable.Compute_inc_Mb_v(variable.Get_fb(), variable.Get_qb());
}

void ChLinkMotorRotationSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.Get_qb()(0) = aux_dt;
}

void ChLinkMotorRotationSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.Get_qb()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}

void ChLinkMotorRotationSpeed::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationSpeed>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(rot_offset);
    marchive << CHNVP(avoid_angle_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationSpeed::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorRotationSpeed>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(rot_offset);
    marchive >> CHNVP(avoid_angle_drift);
}

}  // end namespace chrono
