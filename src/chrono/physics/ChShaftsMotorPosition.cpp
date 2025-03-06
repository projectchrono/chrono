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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaftsMotorPosition.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotorPosition)

ChShaftsMotorPosition::ChShaftsMotorPosition() : rot_offset(0), violation(0), motor_load(0) {
    // default motion function : a ramp
    motor_function = chrono_types::make_shared<ChFunctionRamp>(0.0,  // default y(0)
                                                               1.0   // default dy/dx , i.e.   1 [rad/s]
    );
}

ChShaftsMotorPosition::ChShaftsMotorPosition(const ChShaftsMotorPosition& other)
    : ChShaftsMotor(other), violation(0), motor_load(0) {
    motor_function = other.motor_function;
    rot_offset = other.rot_offset;
}

bool ChShaftsMotorPosition::Initialize(std::shared_ptr<ChShaft> shaft_1, std::shared_ptr<ChShaft> shaft_2) {
    // Parent class initialize
    if (!ChShaftsMotor::Initialize(shaft_1, shaft_2))
        return false;

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsMotorPosition::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsMotor::Update(time, update_assets);

    // Update class data
    motor_function->Update(time);  // call callbacks if any
    violation = GetMotorPos() - motor_function->GetVal(time) - rot_offset;
}

void ChShaftsMotorPosition::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = motor_load;
}

void ChShaftsMotorPosition::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    motor_load = L(off_L);
}

void ChShaftsMotorPosition::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  // the L vector
                                                const double c               // a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsMotorPosition::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                                ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                                const double c,            // a scaling factor
                                                bool do_clamp,             // apply clamping to c*C?
                                                double recovery_clamp      // value for min/max clamping of c*C
) {
    double res = c * (GetMotorPos() - motor_function->GetVal(GetChTime()) - rot_offset);

    if (do_clamp) {
        res = std::min(std::max(res, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L) += res;
}

void ChShaftsMotorPosition::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                                 ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                                 const double c             // a scaling factor
) {
    double ct = -motor_function->GetDer(GetChTime());
    Qc(off_L) += c * ct;
}

void ChShaftsMotorPosition::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,  // offset in L, Qc
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));
    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsMotorPosition::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                              ChStateDelta& v,
                                              const unsigned int off_L,  // offset in L
                                              ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftsMotorPosition::InjectConstraints(ChSystemDescriptor& descriptor) {
    descriptor.InsertConstraint(&constraint);
}

void ChShaftsMotorPosition::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsMotorPosition::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    double res = factor * (GetMotorPos() - motor_function->GetVal(GetChTime()) - rot_offset);

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftsMotorPosition::ConstraintsBiLoad_Ct(double factor) {
    double ct = -motor_function->GetDer(GetChTime());
    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * ct);
}

void ChShaftsMotorPosition::LoadConstraintJacobians() {
    constraint.Get_Cq_a()(0) = 1;
    constraint.Get_Cq_b()(0) = -1;
}

void ChShaftsMotorPosition::ConstraintsFetch_react(double factor) {
    motor_load = -constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsMotorPosition::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsMotorPosition>();

    // serialize parent class
    ChShaftsMotor::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(motor_load);
    archive_out << CHNVP(rot_offset);
    archive_out << CHNVP(motor_function);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotorPosition::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsMotorPosition>();

    // deserialize parent class:
    ChShaftsMotor::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(motor_load);
    archive_in >> CHNVP(rot_offset);
    archive_in >> CHNVP(motor_function);
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables());
}

}  // end namespace chrono
