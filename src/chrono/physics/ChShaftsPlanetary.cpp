//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChShaftsPlanetary.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsPlanetary.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsPlanetary> a_registration_ChShaftsPlanetary;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS

ChShaftsPlanetary::ChShaftsPlanetary() {
    this->r1 = 1;
    this->r2 = 1;
    this->r3 = 1;

    this->torque_react = 0;

    this->shaft1 = 0;
    this->shaft2 = 0;
    this->shaft3 = 0;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    // variables.SetUserData((void*)this);
}

ChShaftsPlanetary::~ChShaftsPlanetary() {
}

void ChShaftsPlanetary::Copy(ChShaftsPlanetary* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    // copy class data
    r1 = source->r1;
    r2 = source->r2;
    r3 = source->r3;

    torque_react = source->torque_react;
    this->shaft1 = 0;
    this->shaft2 = 0;
    this->shaft3 = 0;
}

int ChShaftsPlanetary::Initialize(std::shared_ptr<ChShaft> mshaft1,
                                  std::shared_ptr<ChShaft> mshaft2,
                                  std::shared_ptr<ChShaft> mshaft3) {
    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();
    ChShaft* mm3 = mshaft3.get();
    assert(mm1 && mm2 && mm3);
    assert(mm1 != mm2);
    assert(mm1 != mm3);
    assert(mm3 != mm2);
    assert((mm1->GetSystem() == mm2->GetSystem()) && (mm1->GetSystem() == mm3->GetSystem()));

    this->shaft1 = mm1;
    this->shaft2 = mm2;
    this->shaft3 = mm3;

    this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables(), &mm3->Variables());

    this->SetSystem(this->shaft1->GetSystem());
    return true;
}

void ChShaftsPlanetary::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsPlanetary::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                            ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                            const ChVectorDynamic<>& L,  ///< the L vector
                                            const double c               ///< a scaling factor
                                            ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsPlanetary::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                            ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                            const double c,            ///< a scaling factor
                                            bool do_clamp,             ///< apply clamping to c*C?
                                            double recovery_clamp      ///< value for min/max clamping of c*C
                                            ) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = ChMin(ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsPlanetary::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,  ///< offset in L, Qc
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsPlanetary::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                   ChStateDelta& v,
                                   const unsigned int off_L,  ///< offset in L
                                   ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

////////// LCP INTERFACES ////

void ChShaftsPlanetary::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = this->torque_react;
}

void ChShaftsPlanetary::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    this->torque_react = L(off_L);
}

void ChShaftsPlanetary::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    // if (!this->IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsPlanetary::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsPlanetary::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!this->IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsPlanetary::ConstraintsBiLoad_Ct(double factor) {
    // if (!this->IsActive())
    //	return;

    // nothing
}

/*
void ChShaftsPlanetary::ConstraintsFbLoadForces(double factor)
{
    // no forces
}
*/

void ChShaftsPlanetary::ConstraintsLoadJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()->SetElement(0, 0, (float)this->r1);
    constraint.Get_Cq_b()->SetElement(0, 0, (float)this->r2);
    constraint.Get_Cq_c()->SetElement(0, 0, (float)this->r3);
}

void ChShaftsPlanetary::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    this->torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsPlanetary::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(r1);
    marchive << CHNVP(r2);
    marchive << CHNVP(r3);
    //marchive << CHNVP(shaft1); //***TODO*** serialize with shared ptr
    //marchive << CHNVP(shaft2); //***TODO*** serialize with shared ptr
    //marchive << CHNVP(shaft3); //***TODO*** serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsPlanetary::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(r1);
    marchive >> CHNVP(r2);
    marchive >> CHNVP(r3);
    //marchive >> CHNVP(shaft1); //***TODO*** serialize with shared ptr
    //marchive >> CHNVP(shaft2); //***TODO*** serialize with shared ptr
    //marchive >> CHNVP(shaft3); //***TODO*** serialize with shared ptr
} 


}  // END_OF_NAMESPACE____

/////////////////////
