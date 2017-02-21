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
// Authors: Alesandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkDistance.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkDistance)

ChLinkDistance::ChLinkDistance() : pos1(VNULL), pos2(VNULL), distance(0), curr_dist(0) {}

ChLinkDistance::ChLinkDistance(const ChLinkDistance& other) : ChLink(other) {
    Body1 = other.Body1;
    Body2 = other.Body2;
    system = other.system;
    Cx.SetVariables(&other.Body1->Variables(), &other.Body2->Variables());
    pos1 = other.pos1;
    pos2 = other.pos2;
    distance = other.distance;
    curr_dist = other.curr_dist;
}

int ChLinkDistance::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                               std::shared_ptr<ChBodyFrame> mbody2,
                               bool pos_are_relative,
                               ChVector<> mpos1,
                               ChVector<> mpos2,
                               bool auto_distance,
                               double mdistance) {
    Body1 = mbody1.get();
    Body2 = mbody2.get();
    Cx.SetVariables(&Body1->Variables(), &Body2->Variables());

    if (pos_are_relative) {
        pos1 = mpos1;
        pos2 = mpos2;
    } else {
        pos1 = Body1->TransformPointParentToLocal(mpos1);
        pos2 = Body2->TransformPointParentToLocal(mpos2);
    }

    ChVector<> AbsDist = Body1->TransformPointLocalToParent(pos1) - Body2->TransformPointLocalToParent(pos2);
    curr_dist = AbsDist.Length();

    if (auto_distance) {
        distance = curr_dist;
    } else {
        distance = mdistance;
    }

    return true;
}

double ChLinkDistance::GetCurrentDistance() const {
    return (((ChFrame<double>*)Body1)->TransformLocalToParent(pos1) -
            ((ChFrame<double>*)Body2)->TransformLocalToParent(pos2))
        .Length();
}

ChCoordsys<> ChLinkDistance::GetLinkRelativeCoords() {
    ChVector<> D2local;
    ChVector<> D2temp = (Vnorm(Body1->TransformPointLocalToParent(pos1) - Body2->TransformPointLocalToParent(pos2)));
    ChVector<> D2rel = Body2->TransformDirectionParentToLocal(D2temp);
    ChVector<> Vx, Vy, Vz;
    ChMatrix33<> rel_matrix;
    XdirToDxDyDz(D2rel, VECT_Y, Vx, Vy, Vz);
    rel_matrix.Set_A_axis(Vx, Vy, Vz);

    Quaternion Ql2 = rel_matrix.Get_A_quaternion();
    return ChCoordsys<>(pos2, Ql2);
}

void ChLinkDistance::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::Update(mytime, update_assets);

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

    Cx.Get_Cq_a()->ElementN(0) = CqAx.x();
    Cx.Get_Cq_a()->ElementN(1) = CqAx.y();
    Cx.Get_Cq_a()->ElementN(2) = CqAx.z();
    Cx.Get_Cq_a()->ElementN(3) = CqAr.x();
    Cx.Get_Cq_a()->ElementN(4) = CqAr.y();
    Cx.Get_Cq_a()->ElementN(5) = CqAr.z();

    Cx.Get_Cq_b()->ElementN(0) = CqBx.x();
    Cx.Get_Cq_b()->ElementN(1) = CqBx.y();
    Cx.Get_Cq_b()->ElementN(2) = CqBx.z();
    Cx.Get_Cq_b()->ElementN(3) = CqBr.x();
    Cx.Get_Cq_b()->ElementN(4) = CqBr.y();
    Cx.Get_Cq_b()->ElementN(5) = CqBr.z();

    //***TO DO***  C_dt? C_dtdt? (may be never used..)
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkDistance::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -react_force.x();
}

void ChLinkDistance::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force.x() = -L(off_L);
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque = VNULL;
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

void ChLinkDistance::IntToDescriptor(const unsigned int off_v,  ///< offset in v, R
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

void ChLinkDistance::IntFromDescriptor(const unsigned int off_v,  ///< offset in v
                                       ChStateDelta& v,
                                       const unsigned int off_L,  ///< offset in L
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L) = Cx.Get_l_i();
}

// SOLVER INTERFACES

void ChLinkDistance::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!IsActive())
        return;

    mdescriptor.InsertConstraint(&Cx);
}

void ChLinkDistance::ConstraintsBiReset() {
    Cx.Set_b_i(0.);
}

void ChLinkDistance::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
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
    react_force.x() = -Cx.Get_l_i() * factor;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque = VNULL;
}

void ChLinkDistance::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkDistance>();

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(distance);
    marchive << CHNVP(pos1);
    marchive << CHNVP(pos2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkDistance::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkDistance>();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(distance);
    marchive >> CHNVP(pos1);
    marchive >> CHNVP(pos2);
}

}  // END_OF_NAMESPACE____
