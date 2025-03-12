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

#include "chrono/physics/ChShaftsMotorSpeed.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotorSpeed)

ChShaftsMotorSpeed::ChShaftsMotorSpeed() : motor_load(0) {
    variable.GetMass()(0, 0) = 1.0;
    variable.GetInvMass()(0, 0) = 1.0;

    motor_function = chrono_types::make_shared<ChFunctionConst>(1.0);

    rot_offset = 0;

    aux_dt = 0;  // used for integrating speed, = pos
    aux_dtdt = 0;

    avoid_drift = true;
}

ChShaftsMotorSpeed::ChShaftsMotorSpeed(const ChShaftsMotorSpeed& other) : ChShaftsMotor(other), motor_load(0) {
    variable = other.variable;

    motor_function = other.motor_function;

    rot_offset = other.rot_offset;

    aux_dt = other.aux_dt;
    aux_dtdt = other.aux_dtdt;

    avoid_drift = other.avoid_drift;
}

bool ChShaftsMotorSpeed::Initialize(std::shared_ptr<ChShaft> shaft_1, std::shared_ptr<ChShaft> shaft_2) {
    // Parent class initialize
    if (!ChShaftsMotor::Initialize(shaft_1, shaft_2))
        return false;

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsMotorSpeed::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsMotor::Update(time, update_assets);

    // update class data

    motor_function->Update(time);  // call callbacks if any
}

void ChShaftsMotorSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                        ChState& x,                // state vector, position part
                                        const unsigned int off_v,  // offset in v state vector
                                        ChStateDelta& v,           // state vector, speed part
                                        double& T                  // time
) {
    x(off_x) = 0;  // aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChShaftsMotorSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
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

void ChShaftsMotorSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChShaftsMotorSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChShaftsMotorSpeed::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = motor_load;
}

void ChShaftsMotorSpeed::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    motor_load = L(off_L);
}

void ChShaftsMotorSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                           ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                           const double c           // a scaling factor
) {
    double imposed_speed = motor_function->GetVal(GetChTime());
    R(off) += imposed_speed * c;
}

void ChShaftsMotorSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                            const ChVectorDynamic<>& w,  // the w vector
                                            const double c               // a scaling factor
) {
    R(off) += c * 1.0 * w(off);
}

void ChShaftsMotorSpeed::IntLoadLumpedMass_Md(const unsigned int off,
                                              ChVectorDynamic<>& Md,
                                              double& err,
                                              const double c) {
    Md(off) += c * 1.0;
}

void ChShaftsMotorSpeed::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                             ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                             const ChVectorDynamic<>& L,  // the L vector
                                             const double c               // a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsMotorSpeed::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                             ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                             const double c,            // a scaling factor
                                             bool do_clamp,             // apply clamping to c*C?
                                             double recovery_clamp      // value for min/max clamping of c*C
) {
    // Add the time-dependent term in residual C as
    //   C = d_error - d_setpoint - d_offset
    // with d_error = x_pos_A-x_pos_B, and d_setpoint = x(t)
    double C;
    if (avoid_drift)
        C = GetMotorPos() - aux_dt - rot_offset;
    else
        C = 0.0;

    double res = c * C;

    if (do_clamp) {
        res = std::min(std::max(res, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L) += res;
}

void ChShaftsMotorSpeed::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                              ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                              const double c             // a scaling factor
) {
    double ct = -motor_function->GetVal(GetChTime());
    Qc(off_L) += c * ct;
}

void ChShaftsMotorSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                         const ChStateDelta& v,
                                         const ChVectorDynamic<>& R,
                                         const unsigned int off_L,  // offset in L, Qc
                                         const ChVectorDynamic<>& L,
                                         const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));
    constraint.SetRightHandSide(Qc(off_L));

    variable.State()(0, 0) = v(off_v);
    variable.Force()(0, 0) = R(off_v);
}

void ChShaftsMotorSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                           ChStateDelta& v,
                                           const unsigned int off_L,  // offset in L
                                           ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();

    v(off_v) = variable.State()(0, 0);
}

void ChShaftsMotorSpeed::InjectConstraints(ChSystemDescriptor& descriptor) {
    descriptor.InsertConstraint(&constraint);
}

void ChShaftsMotorSpeed::InjectVariables(ChSystemDescriptor& descriptor) {
    descriptor.InsertVariables(&variable);
}

void ChShaftsMotorSpeed::VariablesFbReset() {
    variable.Force().setZero();
}

void ChShaftsMotorSpeed::VariablesFbLoadForces(double factor) {
    double imposed_speed = motor_function->GetVal(GetChTime());
    variable.Force()(0) += imposed_speed * factor;
}

void ChShaftsMotorSpeed::VariablesFbIncrementMq() {
    variable.AddMassTimesVector(variable.Force(), variable.State());
}

void ChShaftsMotorSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.State()(0) = aux_dt;
}

void ChShaftsMotorSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.State()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}

void ChShaftsMotorSpeed::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsMotorSpeed::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    double C;
    if (avoid_drift)
        C = GetMotorPos() - aux_dt - rot_offset;
    else
        C = 0.0;

    double res = factor * C;

    if (do_clamp) {
        res = std::min(std::max(res, -recovery_clamp), recovery_clamp);
    }

    constraint.SetRightHandSide(constraint.GetRightHandSide() + res);
}

void ChShaftsMotorSpeed::ConstraintsBiLoad_Ct(double factor) {
    double ct = -motor_function->GetVal(GetChTime());
    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * ct);
}

void ChShaftsMotorSpeed::LoadConstraintJacobians() {
    constraint.Get_Cq_a()(0) = 1;
    constraint.Get_Cq_b()(0) = -1;
}

void ChShaftsMotorSpeed::ConstraintsFetch_react(double factor) {
    motor_load = -constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsMotorSpeed::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsMotorSpeed>();

    // serialize parent class
    ChShaftsMotor::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(motor_load);
    archive_out << CHNVP(motor_function);
    archive_out << CHNVP(rot_offset);
    archive_out << CHNVP(avoid_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotorSpeed::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsMotorSpeed>();

    // deserialize parent class:
    ChShaftsMotor::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(motor_load);
    archive_in >> CHNVP(motor_function);
    archive_in >> CHNVP(rot_offset);
    archive_in >> CHNVP(avoid_drift);
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables());
}

}  // end namespace chrono
