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

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChLinkDirFrame.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegister<ChLinkDirFrame> a_registration_ChLinkDirFrame;

ChLinkDirFrame::ChLinkDirFrame() : react(VNULL), direction(VECT_X) {}

ChLinkDirFrame::ChLinkDirFrame(const ChLinkDirFrame& other) : ChLinkBase(other) {
    react = other.react;
    direction = other.direction;
}

ChCoordsys<> ChLinkDirFrame::GetLinkAbsoluteCoords() {
    if (body) {
        ChCoordsys<> linkcsys(mnode->GetPos(), csys_direction.rot >> (*body).GetRot());
        return linkcsys;
    }
    return CSYSNORM;
}

void ChLinkDirFrame::SetDirectionInBodyCoords(ChVector<> mattach) {
    direction = mattach;
    ChMatrix33<> rot;
    rot.Set_A_Xdir(direction);
    csys_direction.rot = rot.Get_A_quaternion();
    csys_direction.pos = VNULL;
}

void ChLinkDirFrame::SetDirectionInAbsoluteCoords(ChVector<> mattach) {
    direction = body->TransformDirectionParentToLocal(mattach);
    SetDirectionInBodyCoords(direction);
}

int ChLinkDirFrame::Initialize(std::shared_ptr<ChNodeFEAxyzD> anode,
                               std::shared_ptr<ChBodyFrame> mbody,
                               ChVector<>* mdir) {
    assert(anode && mbody);

    body = mbody;
    mnode = anode;

    constraint1.SetVariables(&(mnode->Variables_D()), &(body->Variables()));
    constraint2.SetVariables(&(mnode->Variables_D()), &(body->Variables()));

    // SetSystem(body->GetSystem());

    if (mdir) {
        direction = body->TransformDirectionParentToLocal(*mdir);
    } else {
        // downcasting
        if (!mnode)
            return false;

        ChVector<> temp = mnode->GetD();
        direction = body->TransformDirectionParentToLocal(temp);
    }

    return true;
}

void ChLinkDirFrame::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // ...
}

ChMatrixNM<double, 2, 1> ChLinkDirFrame::GetC() const {
    ChMatrix33<> Arw = csys_direction.rot >> body->coord.rot;
    ChVector<> res = Arw.MatrT_x_Vect(mnode->GetD());
    ChMatrixNM<double, 2, 1> C;
    C(0, 0) = res.y;
    C(1, 0) = res.z;
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkDirFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = 2 * react.y;
    L(off_L + 1) = 2 * react.z;
}

void ChLinkDirFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.y = 0.5 * L(off_L + 0);
    react.z = 0.5 * L(off_L + 1);
}

void ChLinkDirFrame::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
                                         ) {
    if (!IsActive())
        return;

    constraint1.MultiplyTandAdd(R, L(off_L + 0) * c);
    constraint2.MultiplyTandAdd(R, L(off_L + 1) * c);
}

void ChLinkDirFrame::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
                                         ) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw = csys_direction.rot >> body->coord.rot;
    ChVector<> res = Arw.MatrT_x_Vect(mnode->GetD());

    ChVector<> cres = res * c;

    if (do_clamp) {
        cres.y = ChMin(ChMax(cres.y, -recovery_clamp), recovery_clamp);
        cres.z = ChMin(ChMax(cres.z, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.y;
    Qc(off_L + 1) += cres.z;
}

void ChLinkDirFrame::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    constraint1.Set_l_i(L(off_L + 0));
    constraint2.Set_l_i(L(off_L + 1));

    constraint1.Set_b_i(Qc(off_L + 0));
    constraint2.Set_b_i(Qc(off_L + 1));
}

void ChLinkDirFrame::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.Get_l_i();
    L(off_L + 1) = constraint2.Get_l_i();
}

// SOLVER INTERFACES

void ChLinkDirFrame::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
}

void ChLinkDirFrame::ConstraintsBiReset() {
    constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
}

void ChLinkDirFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!mnode)
        return;
    ChMatrix33<> Arw = csys_direction.rot >> body->coord.rot;
    ChVector<> res = Arw.MatrT_x_Vect(mnode->GetD());

    constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.y);
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.z);
}

void ChLinkDirFrame::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkDirFrame::ConstraintsLoadJacobians() {
    // compute jacobians
    ChMatrix33<> Aow(body->coord.rot);
    ChMatrix33<> Aro(csys_direction.rot);
    ChMatrix33<> Arw(csys_direction.rot >> body->coord.rot);

    ChVector<> Zo = Aow.MatrT_x_Vect(mnode->GetD());

    ChMatrix33<> ztilde;
    ztilde.Set_X_matrix(Zo);

    ChMatrix33<> Jrb;
    Jrb.MatrTMultiply(Aro, ztilde);

    ChMatrix33<> Jra;
    Jra.CopyFromMatrixT(Arw);

    constraint1.Get_Cq_a()->PasteClippedMatrix(&Jra, 1, 0, 1, 3, 0, 0);
    constraint2.Get_Cq_a()->PasteClippedMatrix(&Jra, 2, 0, 1, 3, 0, 0);

    constraint1.Get_Cq_b()->PasteClippedMatrix(&Jrb, 1, 0, 1, 3, 0, 3);
    constraint2.Get_Cq_b()->PasteClippedMatrix(&Jrb, 2, 0, 1, 3, 0, 3);
}

void ChLinkDirFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.y = constraint1.Get_l_i() * factor;
    react.z = constraint2.Get_l_i() * factor;
}

// FILE I/O

void ChLinkDirFrame::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkDirFrame::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono