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
//   ChLinkDistance.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkDistance.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkDistance> a_registration_ChLinkDistance;

ChLinkDistance::ChLinkDistance() {
    pos1 = pos2 = VNULL;
    distance = 0;
    curr_dist = 0;
    cache_li_speed = cache_li_pos = 0;
}

ChLinkDistance::~ChLinkDistance() {
}

int ChLinkDistance::Initialize(
    std::shared_ptr<ChBodyFrame> mbody1,  ///< first body to link
    std::shared_ptr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,                ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpos1,                     ///< position of distance endpoint, for 1st body (rel. or abs., see flag above)
    ChVector<> mpos2,                     ///< position of distance endpoint, for 2nd body (rel. or abs., see flag above)
    bool auto_distance,                   ///< if true, initializes the imposed distance as the distance between mpos1 and mpos2
    double mdistance                      ///< imposed distance (no need to define, if auto_distance=true.)
    ) {
    this->Body1 = mbody1.get();
    this->Body2 = mbody2.get();
    this->Cx.SetVariables(&this->Body1->Variables(), &this->Body2->Variables());

    if (pos_are_relative) {
        this->pos1 = mpos1;
        this->pos2 = mpos2;
    } else {
        this->pos1 = this->Body1->TransformPointParentToLocal(mpos1);
        this->pos2 = this->Body2->TransformPointParentToLocal(mpos2);
    }

    ChVector<> AbsDist = Body1->TransformPointLocalToParent(pos1) - Body2->TransformPointLocalToParent(pos2);
    this->curr_dist = AbsDist.Length();

    if (auto_distance) {
        this->distance = this->curr_dist;
    } else {
        this->distance = mdistance;
    }

    return true;
}

void ChLinkDistance::Copy(ChLinkDistance* source) {
    // first copy the parent class data...
    //
    ChLink::Copy(source);

    // copy custom data:
    Body1 = source->Body1;
    Body2 = source->Body2;
    system = source->system;
    Cx.SetVariables(&source->Body1->Variables(), &source->Body2->Variables());
    pos1 = source->pos1;
    pos2 = source->pos2;
    distance = source->distance;
    curr_dist = source->curr_dist;
    cache_li_speed = source->cache_li_speed;
    cache_li_pos = source->cache_li_pos;
}

ChLink* ChLinkDistance::new_Duplicate() {
    ChLinkDistance* m_l;
    m_l = new ChLinkDistance;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

ChCoordsys<> ChLinkDistance::GetLinkRelativeCoords() {
    ChVector<> D2local;
    ChVector<> D2temp = (Vnorm(Body1->TransformPointLocalToParent(pos1) - Body2->TransformPointLocalToParent(pos2)));
    ChVector<> D2rel = Body2->TransformDirectionParentToLocal(D2temp);
    ChVector<> Vx, Vy, Vz;
    ChVector<> Vsingul(VECT_Y);
    ChMatrix33<> rel_matrix;
    XdirToDxDyDz(&D2rel, &Vsingul, &Vx, &Vy, &Vz);
    rel_matrix.Set_A_axis(Vx, Vy, Vz);

    Quaternion Ql2 = rel_matrix.Get_A_quaternion();
    return ChCoordsys<>(pos2, Ql2);
}

void ChLinkDistance::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::UpdateTime(mytime);

    // compute jacobians
    ChVector<> AbsDist = Body1->TransformPointLocalToParent(pos1) - Body2->TransformPointLocalToParent(pos2);
    curr_dist = AbsDist.Length();
    ChVector<> D2abs = Vnorm(AbsDist);
    ChVector<> D2relB = Body2->TransformDirectionParentToLocal(D2abs);
    ChVector<> D2relA = Body1->TransformDirectionParentToLocal(D2abs);

    ChVector<> CqAx = D2abs;
    ChVector<> CqBx = -D2abs;

    ChVector<> CqAr = -Vcross(D2relA, pos1);
    ChVector<> CqBr = Vcross(D2relB, pos2);

    Cx.Get_Cq_a()->ElementN(0) = (float)CqAx.x;
    Cx.Get_Cq_a()->ElementN(1) = (float)CqAx.y;
    Cx.Get_Cq_a()->ElementN(2) = (float)CqAx.z;
    Cx.Get_Cq_a()->ElementN(3) = (float)CqAr.x;
    Cx.Get_Cq_a()->ElementN(4) = (float)CqAr.y;
    Cx.Get_Cq_a()->ElementN(5) = (float)CqAr.z;

    Cx.Get_Cq_b()->ElementN(0) = (float)CqBx.x;
    Cx.Get_Cq_b()->ElementN(1) = (float)CqBx.y;
    Cx.Get_Cq_b()->ElementN(2) = (float)CqBx.z;
    Cx.Get_Cq_b()->ElementN(3) = (float)CqBr.x;
    Cx.Get_Cq_b()->ElementN(4) = (float)CqBr.y;
    Cx.Get_Cq_b()->ElementN(5) = (float)CqBr.z;

    //***TO DO***  C_dt? C_dtdt? (may be never used..)
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkDistance::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -this->react_force.x;
}

void ChLinkDistance::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    this->react_force.x = -L(off_L);
    this->react_force.y = 0;
    this->react_force.z = 0;

    this->react_torque = VNULL;
}

void ChLinkDistance::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
                                         ) {
    if (!IsActive())
        return;

    Cx.MultiplyTandAdd(R, L(off_L) * c);
}

void ChLinkDistance::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
                                         ) {
    if (!IsActive())
        return;

    if (do_clamp)
        Qc(off_L) += ChMin(ChMax(c * (curr_dist - distance), -recovery_clamp), recovery_clamp);
    else
        Qc(off_L) += c * (curr_dist - distance);
}

void ChLinkDistance::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                              const ChStateDelta& v,
                              const ChVectorDynamic<>& R,
                              const unsigned int off_L,  ///< offset in L, Qc
                              const ChVectorDynamic<>& L,
                              const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    Cx.Set_l_i(L(off_L));

    Cx.Set_b_i(Qc(off_L));
}

void ChLinkDistance::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                ChStateDelta& v,
                                const unsigned int off_L,  ///< offset in L
                                ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L) = Cx.Get_l_i();
}

////////// LCP INTERFACES ////

void ChLinkDistance::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    if (!this->IsActive())
        return;

    mdescriptor.InsertConstraint(&Cx);
}

void ChLinkDistance::ConstraintsBiReset() {
    Cx.Set_b_i(0.);
}

void ChLinkDistance::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!this->IsActive())
        return;

    if (do_clamp)
        Cx.Set_b_i(Cx.Get_b_i() + ChMin(ChMax(factor * (curr_dist - distance), -recovery_clamp), recovery_clamp));
    else
        Cx.Set_b_i(Cx.Get_b_i() + factor * (curr_dist - distance));
}

void ChLinkDistance::ConstraintsLoadJacobians() {
    // already loaded when doing Update (which used the matrices of the scalar constraint objects)
}

void ChLinkDistance::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react_force.x = -Cx.Get_l_i() * factor;
    react_force.y = 0;
    react_force.z = 0;

    react_torque = VNULL;
}

//
// Following functions are for exploiting the contact persistence
//

void ChLinkDistance::ConstraintsLiLoadSuggestedSpeedSolution() {
    Cx.Set_l_i(this->cache_li_speed);
}

void ChLinkDistance::ConstraintsLiLoadSuggestedPositionSolution() {
    Cx.Set_l_i(this->cache_li_pos);
}

void ChLinkDistance::ConstraintsLiFetchSuggestedSpeedSolution() {
    this->cache_li_speed = (float)Cx.Get_l_i();
}

void ChLinkDistance::ConstraintsLiFetchSuggestedPositionSolution() {
    this->cache_li_pos = (float)Cx.Get_l_i();
}



void ChLinkDistance::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(distance);
    marchive << CHNVP(pos1);
    marchive << CHNVP(pos2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkDistance::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(distance);
    marchive >> CHNVP(pos1);
    marchive >> CHNVP(pos2);
}


}  // END_OF_NAMESPACE____
