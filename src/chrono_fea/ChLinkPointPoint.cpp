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
#include "chrono_fea/ChLinkPointPoint.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointPoint)

ChLinkPointPoint::ChLinkPointPoint() : react(VNULL) {}

ChLinkPointPoint::ChLinkPointPoint(const ChLinkPointPoint& other) : ChLinkBase(other) {
    react = other.react;
}

int ChLinkPointPoint::Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA, std::shared_ptr<ChNodeFEAxyz> anodeB) {
    assert(anodeA && anodeB);

    mnodeA = anodeA;
    mnodeB = anodeB;

    constraint1.SetVariables(&(mnodeA->Variables()), &(mnodeB->Variables()));
    constraint2.SetVariables(&(mnodeA->Variables()), &(mnodeB->Variables()));
    constraint3.SetVariables(&(mnodeA->Variables()), &(mnodeB->Variables()));

    // SetSystem(body->GetSystem());

    return true;
}

void ChLinkPointPoint::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

ChMatrixNM<double, 3, 1> ChLinkPointPoint::GetC() const {
    ChVector<> res = mnodeA->GetPos() - mnodeB->GetPos();
    ChMatrixNM<double, 3, 1> C;
    C(0, 0) = res.x();
    C(1, 0) = res.y();
    C(2, 0) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointPoint::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkPointPoint::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkPointPoint::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                           ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                           const ChVectorDynamic<>& L,  // the L vector
                                           const double c               // a scaling factor
                                           ) {
    if (!IsActive())
        return;

    constraint1.MultiplyTandAdd(R, L(off_L + 0) * c);
    constraint2.MultiplyTandAdd(R, L(off_L + 1) * c);
    constraint3.MultiplyTandAdd(R, L(off_L + 2) * c);
}

void ChLinkPointPoint::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                           ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                           const double c,            // a scaling factor
                                           bool do_clamp,             // apply clamping to c*C?
                                           double recovery_clamp      // value for min/max clamping of c*C
                                           ) {
    if (!IsActive())
        return;

    ChVector<> res = mnodeA->GetPos() - mnodeB->GetPos();
    ChVector<> cres = res * c;

    if (do_clamp) {
        cres.x() = ChMin(ChMax(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = ChMin(ChMax(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = ChMin(ChMax(cres.z(), -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.x();
    Qc(off_L + 1) += cres.y();
    Qc(off_L + 2) += cres.z();
}

void ChLinkPointPoint::IntToDescriptor(const unsigned int off_v,
                                       const ChStateDelta& v,
                                       const ChVectorDynamic<>& R,
                                       const unsigned int off_L,
                                       const ChVectorDynamic<>& L,
                                       const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    constraint1.Set_l_i(L(off_L + 0));
    constraint2.Set_l_i(L(off_L + 1));
    constraint3.Set_l_i(L(off_L + 2));

    constraint1.Set_b_i(Qc(off_L + 0));
    constraint2.Set_b_i(Qc(off_L + 1));
    constraint3.Set_b_i(Qc(off_L + 2));
}

void ChLinkPointPoint::IntFromDescriptor(const unsigned int off_v,
                                         ChStateDelta& v,
                                         const unsigned int off_L,
                                         ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.Get_l_i();
    L(off_L + 1) = constraint2.Get_l_i();
    L(off_L + 2) = constraint3.Get_l_i();
}

// SOLVER INTERFACES

void ChLinkPointPoint::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointPoint::ConstraintsBiReset() {
    constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
    constraint3.Set_b_i(0.);
}

void ChLinkPointPoint::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    ChVector<> res = mnodeA->GetPos() - mnodeB->GetPos();

    constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.x());
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.y());
    constraint3.Set_b_i(constraint3.Get_b_i() + factor * res.z());
}

void ChLinkPointPoint::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkPointPoint::ConstraintsLoadJacobians() {
    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.FillDiag(1.0);

    ChMatrix33<> Jxb;
    Jxb.FillDiag(-1.0);

    constraint1.Get_Cq_a()->PasteClippedMatrix(Jxa, 0, 0, 1, 3, 0, 0);
    constraint2.Get_Cq_a()->PasteClippedMatrix(Jxa, 1, 0, 1, 3, 0, 0);
    constraint3.Get_Cq_a()->PasteClippedMatrix(Jxa, 2, 0, 1, 3, 0, 0);

    constraint1.Get_Cq_b()->PasteClippedMatrix(Jxb, 0, 0, 1, 3, 0, 0);
    constraint2.Get_Cq_b()->PasteClippedMatrix(Jxb, 1, 0, 1, 3, 0, 0);
    constraint3.Get_Cq_b()->PasteClippedMatrix(Jxb, 2, 0, 1, 3, 0, 0);
}

void ChLinkPointPoint::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.Get_l_i() * factor;
    react.y() = constraint2.Get_l_i() * factor;
    react.z() = constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointPoint::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPointPoint::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
