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
//   ChShaftsGear.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsGear.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsGear> a_registration_ChShaftsGear;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS

ChShaftsGear::ChShaftsGear() {
    this->ratio = 1;
    this->torque_react = 0;
    this->cache_li_speed = 0.f;
    this->cache_li_pos = 0.f;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    // variables.SetUserData((void*)this);
}

ChShaftsGear::~ChShaftsGear() {
}

void ChShaftsGear::Copy(ChShaftsGear* source) {
    // copy the parent class data...
    ChShaftsCouple::Copy(source);

    // copy class data
    ratio = source->ratio;
    torque_react = source->torque_react;
    cache_li_speed = source->cache_li_speed;
    cache_li_pos = source->cache_li_pos;
}

bool ChShaftsGear::Initialize(std::shared_ptr<ChShaft> mshaft1, std::shared_ptr<ChShaft> mshaft2) {
    // Parent initialization
    if (!ChShaftsCouple::Initialize(mshaft1, mshaft2))
        return false;

    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();

    this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    this->SetSystem(this->shaft1->GetSystem());
    return true;
}

void ChShaftsGear::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsGear::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = this->torque_react;
}

void ChShaftsGear::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    this->torque_react = L(off_L);
}

void ChShaftsGear::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                       ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                       const ChVectorDynamic<>& L,  ///< the L vector
                                       const double c               ///< a scaling factor
                                       ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsGear::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
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

void ChShaftsGear::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                            const ChStateDelta& v,
                            const ChVectorDynamic<>& R,
                            const unsigned int off_L,  ///< offset in L, Qc
                            const ChVectorDynamic<>& L,
                            const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsGear::IntFromLCP(const unsigned int off_v,  ///< offset in v
                              ChStateDelta& v,
                              const unsigned int off_L,  ///< offset in L
                              ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

////////// LCP INTERFACES ////

void ChShaftsGear::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    // if (!this->IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsGear::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsGear::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!this->IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsGear::ConstraintsBiLoad_Ct(double factor) {
    // if (!this->IsActive())
    //	return;

    // nothing
}

/*
void ChShaftsGear::ConstraintsFbLoadForces(double factor)
{
    // no forces
}
*/

void ChShaftsGear::ConstraintsLoadJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()->SetElement(0, 0, (float)this->ratio);
    constraint.Get_Cq_b()->SetElement(0, 0, -1);
}

void ChShaftsGear::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    this->torque_react = constraint.Get_l_i() * factor;
}

// Following functions are for exploiting the contact persistence

void ChShaftsGear::ConstraintsLiLoadSuggestedSpeedSolution() {
    constraint.Set_l_i(this->cache_li_speed);
}

void ChShaftsGear::ConstraintsLiLoadSuggestedPositionSolution() {
    constraint.Set_l_i(this->cache_li_pos);
}

void ChShaftsGear::ConstraintsLiFetchSuggestedSpeedSolution() {
    this->cache_li_speed = (float)constraint.Get_l_i();
}

void ChShaftsGear::ConstraintsLiFetchSuggestedPositionSolution() {
    this->cache_li_pos = (float)constraint.Get_l_i();
}

//////// FILE I/O

void ChShaftsGear::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(ratio);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsGear::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(ratio);
} 


}  // END_OF_NAMESPACE____

/////////////////////
