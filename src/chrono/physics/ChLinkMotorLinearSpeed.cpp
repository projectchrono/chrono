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

    this->c_z = true;
    SetupLinkMask();

    m_func = chrono_types::make_shared<ChFunctionConst>(0.0);

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

void ChLinkMotorLinearSpeed::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorLinear::Update(time, update_assets);

    // Add the time-dependent term in residual C as
    //   C = d_error - d_setpoint - d_offset
    // with d_error = z_pos_1 - z_pos_2, and d_setpoint = x(t)
    if (this->avoid_position_drift)
        C(m_actuated_idx) = this->mpos - aux_dt - this->pos_offset;
    else
        C(m_actuated_idx) = 0.0;
}

void ChLinkMotorLinearSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    double mCt = -m_func->GetVal(this->GetChTime());
    if (mask.GetConstraint(m_actuated_idx).IsActive()) {
        Qc(off_L + m_actuated_idx) += c * mCt;
    }
}

void ChLinkMotorLinearSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = -m_func->GetVal(this->GetChTime());
    if (mask.GetConstraint(m_actuated_idx).IsActive()) {
        mask.GetConstraint(m_actuated_idx).SetRightHandSide(mask.GetConstraint(m_actuated_idx).GetRightHandSide() + factor * mCt);
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
    double imposed_speed = m_func->GetVal(this->GetChTime());
    R(off) += imposed_speed * c;
}

void ChLinkMotorLinearSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                                const ChVectorDynamic<>& w,  // the w vector
                                                const double c               // a scaling factor
) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorLinearSpeed::IntLoadLumpedMass_Md(const unsigned int off,
                                                  ChVectorDynamic<>& Md,
                                                  double& err,
                                                  const double c) {
    Md(off) += c * 1.0;
}

void ChLinkMotorLinearSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                             const ChStateDelta& v,
                                             const ChVectorDynamic<>& R,
                                             const unsigned int off_L,  // offset in L, Qc
                                             const ChVectorDynamic<>& L,
                                             const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorLinear::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    this->variable.State()(0, 0) = v(off_v);
    this->variable.Force()(0, 0) = R(off_v);
}

void ChLinkMotorLinearSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                               ChStateDelta& v,
                                               const unsigned int off_L,  // offset in L
                                               ChVectorDynamic<>& L) {
    // inherit parent
    ChLinkMotorLinear::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.State()(0, 0);
}

////
void ChLinkMotorLinearSpeed::InjectVariables(ChSystemDescriptor& descriptor) {
    variable.SetDisabled(!IsActive());

    descriptor.InsertVariables(&variable);
}

void ChLinkMotorLinearSpeed::VariablesFbReset() {
    variable.Force().setZero();
}

void ChLinkMotorLinearSpeed::VariablesFbLoadForces(double factor) {
    double imposed_speed = m_func->GetVal(this->GetChTime());
    variable.Force()(0) += imposed_speed * factor;
}

void ChLinkMotorLinearSpeed::VariablesFbIncrementMq() {
    variable.AddMassTimesVector(variable.Force(), variable.State());
}

void ChLinkMotorLinearSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.State()(0) = aux_dt;
}

void ChLinkMotorLinearSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.State()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}

void ChLinkMotorLinearSpeed::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorLinearSpeed>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(pos_offset);
    archive_out << CHNVP(avoid_position_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearSpeed::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorLinearSpeed>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(pos_offset);
    archive_in >> CHNVP(avoid_position_drift);
}

}  // end namespace chrono
