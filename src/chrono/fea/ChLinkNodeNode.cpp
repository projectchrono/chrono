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

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChLinkNodeNode.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeNode)

ChLinkNodeNode::ChLinkNodeNode() : react(VNULL) {}

ChLinkNodeNode::ChLinkNodeNode(const ChLinkNodeNode& other) : ChLinkBase(other) {
    react = other.react;
}

int ChLinkNodeNode::Initialize(std::shared_ptr<ChNodeFEAxyz> node1, std::shared_ptr<ChNodeFEAxyz> node2) {
    assert(node1 && node2);

    m_node1 = node1;
    m_node2 = node2;

    m_constraint1.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint2.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint3.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));

    // SetSystem(body->GetSystem());

    return true;
}

void ChLinkNodeNode::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkNodeNode::GetConstraintViolation() const {
    ChVector3d res = m_node1->GetPos() - m_node2->GetPos();
    ChVectorN<double, 3> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeNode::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkNodeNode::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkNodeNode::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    m_constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
}

void ChLinkNodeNode::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChVector3d res = m_node1->GetPos() - m_node2->GetPos();
    ChVector3d cres = res * c;

    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.x();
    Qc(off_L + 1) += cres.y();
    Qc(off_L + 2) += cres.z();
}

void ChLinkNodeNode::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_constraint1.SetLagrangeMultiplier(L(off_L + 0));
    m_constraint2.SetLagrangeMultiplier(L(off_L + 1));
    m_constraint3.SetLagrangeMultiplier(L(off_L + 2));

    m_constraint1.SetRightHandSide(Qc(off_L + 0));
    m_constraint2.SetRightHandSide(Qc(off_L + 1));
    m_constraint3.SetRightHandSide(Qc(off_L + 2));
}

void ChLinkNodeNode::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = m_constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = m_constraint3.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeNode::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_constraint1);
    descriptor.InsertConstraint(&m_constraint2);
    descriptor.InsertConstraint(&m_constraint3);
}

void ChLinkNodeNode::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
    m_constraint2.SetRightHandSide(0.);
    m_constraint3.SetRightHandSide(0.);
}

void ChLinkNodeNode::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    ChVector3d res = m_node1->GetPos() - m_node2->GetPos();

    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res.x());
    m_constraint2.SetRightHandSide(m_constraint2.GetRightHandSide() + factor * res.y());
    m_constraint3.SetRightHandSide(m_constraint3.GetRightHandSide() + factor * res.z());
}

void ChLinkNodeNode::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeNode::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Jxa(1.0);

    ChMatrix33<> Jxb(-1.0);

    m_constraint1.Get_Cq_a().segment(0, 3) = Jxa.row(0);
    m_constraint2.Get_Cq_a().segment(0, 3) = Jxa.row(1);
    m_constraint3.Get_Cq_a().segment(0, 3) = Jxa.row(2);

    m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
    m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
    m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);
}

void ChLinkNodeNode::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = m_constraint1.GetLagrangeMultiplier() * factor;
    react.y() = m_constraint2.GetLagrangeMultiplier() * factor;
    react.z() = m_constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeNode::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeNode::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
