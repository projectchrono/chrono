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
#include "chrono_ldpm/ChLinkNodeNodeRot.h"

using namespace chrono::fea;

namespace chrono {
namespace ldpm {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeNodeRot)

ChLinkNodeNodeRot::ChLinkNodeNodeRot() : react(VNULL) {}

ChLinkNodeNodeRot::ChLinkNodeNodeRot(const ChLinkNodeNodeRot& other) : ChLinkBase(other) {
    react = other.react;
    torque = other.torque;
}

int ChLinkNodeNodeRot::Initialize(std::shared_ptr<ChNodeFEAxyzrot> node1, std::shared_ptr<ChNodeFEAxyzrot> node2) {
    assert(node1 && node2);

    m_node1 = node1;
    m_node2 = node2;

    m_constraint1.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint2.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint3.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    
    m_constraint4.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint5.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));
    m_constraint6.SetVariables(&(m_node1->Variables()), &(m_node2->Variables()));

    // SetSystem(body->GetSystem());

    return true;
}

void ChLinkNodeNodeRot::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkNodeNodeRot::GetConstraintViolation() const {
    ChVector3d res = (m_node1->GetPos()-m_node1->GetX0().GetPos()) - (m_node2->GetPos()-m_node2->GetX0().GetPos());
    ChQuaternion<> q_error = m_node1->GetRot() - m_node2->GetRot();
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};
    ChVectorN<double, 6> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
    C(4) = res_rot.x();
    C(5) = res_rot.y();
    C(6) = res_rot.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeNodeRot::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
    //
    L(off_L + 3) = torque.x();
    L(off_L + 4) = torque.y();
    L(off_L + 5) = torque.z();
}

void ChLinkNodeNodeRot::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
    //
    torque.x() = L(off_L + 3);
    torque.y() = L(off_L + 4);
    torque.z() = L(off_L + 5);
}

void ChLinkNodeNodeRot::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    m_constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
    //
    m_constraint4.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
    m_constraint5.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
    m_constraint6.AddJacobianTransposedTimesScalarInto(R, L(off_L + 5) * c);
}

void ChLinkNodeNodeRot::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    //ChVector3d res = m_node1->GetPos() - m_node2->GetPos();
    ChVector3d res = (m_node1->GetPos()-m_node1->GetX0().GetPos()) - (m_node2->GetPos()-m_node2->GetX0().GetPos());
    ChVector3d cres = res * c;
    //
    ChQuaternion<> q_error = m_node1->GetRot() - m_node2->GetRot();
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};
    ChVector3d cres_rot = res_rot * c;

    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
        
        cres_rot.x() = std::min(std::max(cres_rot.x(), -recovery_clamp), recovery_clamp);
        cres_rot.y() = std::min(std::max(cres_rot.y(), -recovery_clamp), recovery_clamp);
        cres_rot.z() = std::min(std::max(cres_rot.z(), -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.x();
    Qc(off_L + 1) += cres.y();
    Qc(off_L + 2) += cres.z();
    //
    Qc(off_L + 3) += cres_rot.x();
    Qc(off_L + 4) += cres_rot.y();
    Qc(off_L + 5) += cres_rot.z();
}

void ChLinkNodeNodeRot::IntToDescriptor(const unsigned int off_v,
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
    //
    //
    m_constraint4.SetLagrangeMultiplier(L(off_L + 3));
    m_constraint5.SetLagrangeMultiplier(L(off_L + 4));
    m_constraint6.SetLagrangeMultiplier(L(off_L + 5));

    m_constraint4.SetRightHandSide(Qc(off_L + 3));
    m_constraint5.SetRightHandSide(Qc(off_L + 4));
    m_constraint6.SetRightHandSide(Qc(off_L + 5));
}

void ChLinkNodeNodeRot::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = m_constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = m_constraint3.GetLagrangeMultiplier();
    //
    L(off_L + 3) = m_constraint4.GetLagrangeMultiplier();
    L(off_L + 4) = m_constraint5.GetLagrangeMultiplier();
    L(off_L + 5) = m_constraint6.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeNodeRot::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_constraint1);
    descriptor.InsertConstraint(&m_constraint2);
    descriptor.InsertConstraint(&m_constraint3);
    //
    descriptor.InsertConstraint(&m_constraint4);
    descriptor.InsertConstraint(&m_constraint5);
    descriptor.InsertConstraint(&m_constraint6);
}

void ChLinkNodeNodeRot::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
    m_constraint2.SetRightHandSide(0.);
    m_constraint3.SetRightHandSide(0.);
    //
    m_constraint4.SetRightHandSide(0.);
    m_constraint5.SetRightHandSide(0.);
    m_constraint6.SetRightHandSide(0.);
}

void ChLinkNodeNodeRot::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    //ChVector3d res = m_node1->GetPos() - m_node2->GetPos();
    ChVector3d res = (m_node1->GetPos()-m_node1->GetX0().GetPos()) - (m_node2->GetPos()-m_node2->GetX0().GetPos()); 

    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res.x());
    m_constraint2.SetRightHandSide(m_constraint2.GetRightHandSide() + factor * res.y());
    m_constraint3.SetRightHandSide(m_constraint3.GetRightHandSide() + factor * res.z());
    //    
    ChQuaternion<> q_error = m_node1->GetRot() - m_node2->GetRot();
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};
    
    m_constraint4.SetRightHandSide(m_constraint4.GetRightHandSide() + factor * res_rot.x());
    m_constraint5.SetRightHandSide(m_constraint5.GetRightHandSide() + factor * res_rot.y());
    m_constraint6.SetRightHandSide(m_constraint6.GetRightHandSide() + factor * res_rot.z());
}

void ChLinkNodeNodeRot::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeNodeRot::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Jxa(1.0);

    ChMatrix33<> Jxb(-1.0);

    m_constraint1.Get_Cq_a().segment(0, 3) = Jxa.row(0);
    m_constraint2.Get_Cq_a().segment(0, 3) = Jxa.row(1);
    m_constraint3.Get_Cq_a().segment(0, 3) = Jxa.row(2);

    m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
    m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
    m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);
    //
    //
    m_constraint4.Get_Cq_a().segment(3, 3) = Jxa.row(0);
    m_constraint5.Get_Cq_a().segment(3, 3) = Jxa.row(1);
    m_constraint6.Get_Cq_a().segment(3, 3) = Jxa.row(2);

    m_constraint4.Get_Cq_b().segment(3, 3) = Jxb.row(0);
    m_constraint5.Get_Cq_b().segment(3, 3) = Jxb.row(1);
    m_constraint6.Get_Cq_b().segment(3, 3) = Jxb.row(2);
}

void ChLinkNodeNodeRot::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = m_constraint1.GetLagrangeMultiplier() * factor;
    react.y() = m_constraint2.GetLagrangeMultiplier() * factor;
    react.z() = m_constraint3.GetLagrangeMultiplier() * factor;
    //
    torque.x() = m_constraint4.GetLagrangeMultiplier() * factor;
    torque.y() = m_constraint5.GetLagrangeMultiplier() * factor;
    torque.z() = m_constraint6.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeNodeRot::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeNodeRot::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace ldpm
}  // end namespace chrono
