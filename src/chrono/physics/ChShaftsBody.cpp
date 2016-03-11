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
//   ChShaftsBody.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaftsBody.h"
#include "physics/ChGlobal.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsBody> a_registration_ChShaftsBody;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsBody::ChShaftsBody() {
    this->torque_react = 0;

    this->shaft = 0;
    this->body = 0;
    this->shaft_dir = VECT_Z;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsBody::~ChShaftsBody() {
}

void ChShaftsBody::Copy(ChShaftsBody* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    // copy class data

    torque_react = source->torque_react;
    this->shaft_dir = source->shaft_dir;
    this->shaft = 0;
    this->body = 0;
}

bool ChShaftsBody::Initialize(std::shared_ptr<ChShaft> mshaft,
                              std::shared_ptr<ChBodyFrame> mbody,
                              const ChVector<>& mdir) {
    ChShaft* mm1 = mshaft.get();
    ChBodyFrame* mm2 = mbody.get();
    assert(mm1 && mm2);

    this->shaft = mm1;
    this->body = mm2;
    this->shaft_dir = Vnorm(mdir);

    this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    this->SetSystem(this->shaft->GetSystem());
    return true;
}

void ChShaftsBody::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsBody::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = this->torque_react;
}

void ChShaftsBody::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    this->torque_react = L(off_L);
}

void ChShaftsBody::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                       ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                       const ChVectorDynamic<>& L,  ///< the L vector
                                       const double c               ///< a scaling factor
                                       ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsBody::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
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

void ChShaftsBody::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                            const ChStateDelta& v,
                            const ChVectorDynamic<>& R,
                            const unsigned int off_L,  ///< offset in L, Qc
                            const ChVectorDynamic<>& L,
                            const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsBody::IntFromLCP(const unsigned int off_v,  ///< offset in v
                              ChStateDelta& v,
                              const unsigned int off_L,  ///< offset in L
                              ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

////////// LCP INTERFACES ////

void ChShaftsBody::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    // if (!this->IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsBody::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsBody::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!this->IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsBody::ConstraintsBiLoad_Ct(double factor) {
    // if (!this->IsActive())
    //	return;

    // nothing
}

void ChShaftsBody::ConstraintsLoadJacobians() {
    // compute jacobians
    // ChVector<> jacw = this->body->TransformDirectionParentToLocal(shaft_dir);
    ChVector<> jacw = shaft_dir;

    this->constraint.Get_Cq_a()->ElementN(0) = -1;

    this->constraint.Get_Cq_b()->ElementN(0) = 0;
    this->constraint.Get_Cq_b()->ElementN(1) = 0;
    this->constraint.Get_Cq_b()->ElementN(2) = 0;
    this->constraint.Get_Cq_b()->ElementN(3) = (float)jacw.x;
    this->constraint.Get_Cq_b()->ElementN(4) = (float)jacw.y;
    this->constraint.Get_Cq_b()->ElementN(5) = (float)jacw.z;
}

void ChShaftsBody::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    this->torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsBody::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(shaft_dir);
    //marchive << CHNVP(shaft);  //***TODO*** serialize, with shared ptr
    //marchive << CHNVP(body); //***TODO*** serialize, with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsBody::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(shaft_dir);
    //marchive >> CHNVP(shaft);  //***TODO*** serialize, with shared ptr
    //marchive >> CHNVP(body); //***TODO*** serialize, with shared ptr
}



}  // END_OF_NAMESPACE____

/////////////////////
