// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotor)

ChShaftsMotor::ChShaftsMotor() : motor_torque(0), motor_mode(MOT_MODE_TORQUE), motor_set_rot(0), motor_set_rot_dt(0) {
}

ChShaftsMotor::ChShaftsMotor(const ChShaftsMotor& other) : ChShaftsCouple(other) {
    motor_torque = other.motor_torque;
    motor_mode = other.motor_mode;
    motor_set_rot = other.motor_set_rot;
    motor_set_rot_dt = other.motor_set_rot_dt;
}

bool ChShaftsMotor::Initialize(std::shared_ptr<ChShaft> mshaft1, std::shared_ptr<ChShaft> mshaft2) {
    // Parent class initialize
    if (!ChShaftsCouple::Initialize(mshaft1, mshaft2))
        return false;

    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsMotor::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsMotor::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (motor_mode != MOT_MODE_TORQUE)
        L(off_L) = motor_torque;
}

void ChShaftsMotor::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    if (motor_mode != MOT_MODE_TORQUE)
        motor_torque = L(off_L);
}

void ChShaftsMotor::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                      ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                      const double c           // a scaling factor
                                      ) {
    if (motor_mode == MOT_MODE_TORQUE) {
        if (shaft1->IsActive())
            R(shaft1->GetOffset_w()) += motor_torque * c;
        if (shaft2->IsActive())
            R(shaft2->GetOffset_w()) += -motor_torque * c;
    }
}

void ChShaftsMotor::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                        ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                        const ChVectorDynamic<>& L,  // the L vector
                                        const double c               // a scaling factor
                                        ) {
    if (motor_mode != MOT_MODE_TORQUE)
        constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsMotor::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                        ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                        const double c,            // a scaling factor
                                        bool do_clamp,             // apply clamping to c*C?
                                        double recovery_clamp      // value for min/max clamping of c*C
                                        ) {
    if (motor_mode != MOT_MODE_TORQUE) {
        double res;

        if (motor_mode == MOT_MODE_SPEED)
            res = 0;  // no need to stabilize positions

        if (motor_mode == MOT_MODE_ROTATION)
            res = c * (GetMotorRot() - motor_set_rot);

        if (do_clamp) {
            res = ChMin(ChMax(res, -recovery_clamp), recovery_clamp);
        }

        Qc(off_L) += res;
    }
}

void ChShaftsMotor::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                         const double c             // a scaling factor
                                         ) {
    if (motor_mode == MOT_MODE_SPEED) {
        double ct = motor_set_rot_dt;
        Qc(off_L) += c * ct;
    }
}

void ChShaftsMotor::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                    const ChStateDelta& v,
                                    const ChVectorDynamic<>& R,
                                    const unsigned int off_L,  // offset in L, Qc
                                    const ChVectorDynamic<>& L,
                                    const ChVectorDynamic<>& Qc) {
    if (motor_mode != MOT_MODE_TORQUE) {
        constraint.Set_l_i(L(off_L));

        constraint.Set_b_i(Qc(off_L));
    }
}

void ChShaftsMotor::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                      ChStateDelta& v,
                                      const unsigned int off_L,  // offset in L
                                      ChVectorDynamic<>& L) {
    if (motor_mode != MOT_MODE_TORQUE) {
        L(off_L) = constraint.Get_l_i();
    }
}

// SOLVER INTERFACES

void ChShaftsMotor::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;
    if (motor_mode != MOT_MODE_TORQUE)
        mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsMotor::ConstraintsBiReset() {
    if (motor_mode != MOT_MODE_TORQUE)
        constraint.Set_b_i(0.);
}

void ChShaftsMotor::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;
    if (motor_mode != MOT_MODE_TORQUE) {
        double res;

        if (motor_mode == MOT_MODE_SPEED)
            res = 0;  // no need to stabilize positions

        if (motor_mode == MOT_MODE_ROTATION)
            res = GetMotorRot() - motor_set_rot;

        constraint.Set_b_i(constraint.Get_b_i() + factor * res);
    }
}

void ChShaftsMotor::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    if (motor_mode == MOT_MODE_SPEED) {
        double ct = motor_set_rot_dt;
        constraint.Set_b_i(constraint.Get_b_i() + factor * ct);
    }
}

void ChShaftsMotor::ConstraintsFbLoadForces(double factor) {
    if (motor_mode == MOT_MODE_TORQUE) {
        shaft1->Variables().Get_fb().ElementN(0) += motor_torque * factor;
        shaft2->Variables().Get_fb().ElementN(0) += -motor_torque * factor;
    }
}

void ChShaftsMotor::ConstraintsLoadJacobians() {
    if (motor_mode != MOT_MODE_TORQUE) {
        constraint.Get_Cq_a()->SetElement(0, 0, 1);
        constraint.Get_Cq_b()->SetElement(0, 0, -1);
    }
}

void ChShaftsMotor::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    if (motor_mode != MOT_MODE_TORQUE)
        motor_torque = constraint.Get_l_i() * factor;
}

//////// FILE I/O

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
class my_enum_mappers : public ChShaftsMotor {
  public:
    CH_ENUM_MAPPER_BEGIN(eCh_shaftsmotor_mode);
    CH_ENUM_VAL(MOT_MODE_ROTATION);
    CH_ENUM_VAL(MOT_MODE_SPEED);
    CH_ENUM_VAL(MOT_MODE_TORQUE);
    CH_ENUM_MAPPER_END(eCh_shaftsmotor_mode);
};

void ChShaftsMotor::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsMotor>();

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    my_enum_mappers::eCh_shaftsmotor_mode_mapper mmapper;
    marchive << CHNVP(mmapper(motor_mode), "motor_mode");
    marchive << CHNVP(motor_torque);
    marchive << CHNVP(motor_set_rot);
    marchive << CHNVP(motor_set_rot_dt);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotor::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsMotor>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    my_enum_mappers::eCh_shaftsmotor_mode_mapper mmapper;
    marchive >> CHNVP(mmapper(motor_mode), "motor_mode");
    marchive >> CHNVP(motor_torque);
    marchive >> CHNVP(motor_set_rot);
    marchive >> CHNVP(motor_set_rot_dt);
}

}  // end namespace chrono
