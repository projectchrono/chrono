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

#include "chrono/physics/ChLinkMotorLinearSpeed.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearSpeed)

ChLinkMotorLinearSpeed::ChLinkMotorLinearSpeed() {
    variable.GetMass()(0, 0) = 1.0;
    variable.GetInvMass()(0, 0) = 1.0;

    m_func = chrono_types::make_shared<ChFunction_Const>(1.0);

    pos_offset = 0;

    aux_dt = 0;  // used for integrating speed, = pos
    aux_dtdt = 0;

    avoid_position_drift = true;
}

ChLinkMotorLinearSpeed::ChLinkMotorLinearSpeed(const ChLinkMotorLinearSpeed& other) : ChLinkMotorLinear(other) {
    variable = other.variable;
    pos_offset = other.pos_offset;
    aux_dt = other.aux_dt;
    aux_dtdt = other.aux_dtdt;
    avoid_position_drift = other.avoid_position_drift;
}

ChLinkMotorLinearSpeed::~ChLinkMotorLinearSpeed() {}

void ChLinkMotorLinearSpeed::Update(double mytime, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    // Add the time-dependent term in residual C as
    //   C = d_error - d_setpoint - d_offset
    // with d_error = x_pos_A- x_pos_B, and d_setpoint = x(t)
    if (this->avoid_position_drift)
        C(0) = this->mpos - aux_dt - this->pos_offset;
    else
        C(0) = 0.0;
}

void ChLinkMotorLinearSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -m_func->Get_y(this->GetChTime());
    if (mask.Constr_N(0).IsActive()) {
        Qc(off_L + 0) += c * mCt;
    }
}

void ChLinkMotorLinearSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -m_func->Get_y(this->GetChTime());
    if (mask.Constr_N(0).IsActive()) {
        mask.Constr_N(0).Set_b_i(mask.Constr_N(0).Get_b_i() + factor * mCt);
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMotorLinearSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                            ChState& x,                // state vector, position part
                                            const unsigned int off_v,  // offset in v state vector
                                            ChStateDelta& v,           // state vector, speed part
                                            double& T                  // time
) {
    x(off_x) = 0;  // aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChLinkMotorLinearSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
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

void ChLinkMotorLinearSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChLinkMotorLinearSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChLinkMotorLinearSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                               ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                               const double c           // a scaling factor
) {
    double imposed_speed = m_func->Get_y(this->GetChTime());
    R(off) += imposed_speed * c;
}

void ChLinkMotorLinearSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                                const ChVectorDynamic<>& w,  // the w vector
                                                const double c               // a scaling factor
) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorLinearSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                             const ChStateDelta& v,
                                             const ChVectorDynamic<>& R,
                                             const unsigned int off_L,  // offset in L, Qc
                                             const ChVectorDynamic<>& L,
                                             const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorLinear::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    this->variable.Get_qb()(0, 0) = v(off_v);
    this->variable.Get_fb()(0, 0) = R(off_v);
}

void ChLinkMotorLinearSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                               ChStateDelta& v,
                                               const unsigned int off_L,  // offset in L
                                               ChVectorDynamic<>& L) {
    // inherit parent
    ChLinkMotorLinear::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.Get_qb()(0, 0);
}

////
void ChLinkMotorLinearSpeed::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variable.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variable);
}

void ChLinkMotorLinearSpeed::VariablesFbReset() {
    variable.Get_fb().setZero();
}

void ChLinkMotorLinearSpeed::VariablesFbLoadForces(double factor) {
    double imposed_speed = m_func->Get_y(this->GetChTime());
    variable.Get_fb()(0) += imposed_speed * factor;
}

void ChLinkMotorLinearSpeed::VariablesFbIncrementMq() {
    variable.Compute_inc_Mb_v(variable.Get_fb(), variable.Get_qb());
}

void ChLinkMotorLinearSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.Get_qb()(0) = aux_dt;
}

void ChLinkMotorLinearSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.Get_qb()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}

void ChLinkMotorLinearSpeed::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearSpeed>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(pos_offset);
    marchive << CHNVP(avoid_position_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearSpeed::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotorLinearSpeed>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(pos_offset);
    marchive >> CHNVP(avoid_position_drift);
}

}  // end namespace chrono
