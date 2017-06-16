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
#include "chrono_fea/ChLinkPointFrame.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointFrame)

ChLinkPointFrame::ChLinkPointFrame() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkPointFrame::ChLinkPointFrame(const ChLinkPointFrame& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
}

ChCoordsys<> ChLinkPointFrame::GetLinkAbsoluteCoords() {
    if (m_body) {
        ChCoordsys<> linkcsys = m_csys >> (*m_body);
        return linkcsys;
    }
    return CSYSNORM;
}

int ChLinkPointFrame::Initialize(std::shared_ptr<ChIndexedNodes> nodes,
                                 unsigned int node_index,
                                 std::shared_ptr<ChBodyFrame> body,
                                 ChVector<>* pos) {
    assert(nodes);

    if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(nodes->GetNode(node_index)))
        return Initialize(node, body, pos);

    return false;
}

int ChLinkPointFrame::Initialize(std::shared_ptr<ChNodeFEAxyz> node,
                                 std::shared_ptr<ChBodyFrame> body,
                                 ChVector<>* pos) {
    assert(node && body);

    m_body = body;
    m_node = node;

    constraint1.SetVariables(&(node->Variables()), &(body->Variables()));
    constraint2.SetVariables(&(node->Variables()), &(body->Variables()));
    constraint3.SetVariables(&(node->Variables()), &(body->Variables()));

    ChVector<> pos_abs = pos ? *pos : node->GetPos();
    SetAttachPositionInAbsoluteCoords(pos_abs);

    return true;
}

void ChLinkPointFrame::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

ChMatrixNM<double, 3, 1> ChLinkPointFrame::GetC() const {
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector<> res = Arw.MatrT_x_Vect(m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChMatrixNM<double, 3, 1> C;
    C(0, 0) = res.x();
    C(1, 0) = res.y();
    C(2, 0) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = m_react.x();
    L(off_L + 1) = m_react.y();
    L(off_L + 2) = m_react.z();
}

void ChLinkPointFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    m_react.x() = L(off_L + 0);
    m_react.y() = L(off_L + 1);
    m_react.z() = L(off_L + 2);
}

void ChLinkPointFrame::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void ChLinkPointFrame::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                           ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                           const double c,            // a scaling factor
                                           bool do_clamp,             // apply clamping to c*C?
                                           double recovery_clamp      // value for min/max clamping of c*C
                                           ) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector<> res = Arw.MatrT_x_Vect(m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
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

void ChLinkPointFrame::IntToDescriptor(const unsigned int off_v,
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

void ChLinkPointFrame::IntFromDescriptor(const unsigned int off_v,
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

void ChLinkPointFrame::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointFrame::ConstraintsBiReset() {
    constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
    constraint3.Set_b_i(0.);
}

void ChLinkPointFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_node)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector<> res = Arw.MatrT_x_Vect(m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));

    constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.x());
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.y());
    constraint3.Set_b_i(constraint3.Get_b_i() + factor * res.z());
}

void ChLinkPointFrame::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkPointFrame::ConstraintsLoadJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> Jxn;
    Jxn.CopyFromMatrixT(Arw);

    ChMatrix33<> Jxb;
    Jxb.CopyFromMatrixT(Arw);
    Jxb.MatrNeg();

    ChMatrix33<> atilde;
    atilde.Set_X_matrix(Aow.MatrT_x_Vect(m_node->GetPos() - m_body->GetPos()));
    ChMatrix33<> Jrb;
    Jrb.MatrTMultiply(Aro, atilde);

    constraint1.Get_Cq_a()->PasteClippedMatrix(Jxn, 0, 0, 1, 3, 0, 0);
    constraint2.Get_Cq_a()->PasteClippedMatrix(Jxn, 1, 0, 1, 3, 0, 0);
    constraint3.Get_Cq_a()->PasteClippedMatrix(Jxn, 2, 0, 1, 3, 0, 0);

    constraint1.Get_Cq_b()->PasteClippedMatrix(Jxb, 0, 0, 1, 3, 0, 0);
    constraint2.Get_Cq_b()->PasteClippedMatrix(Jxb, 1, 0, 1, 3, 0, 0);
    constraint3.Get_Cq_b()->PasteClippedMatrix(Jxb, 2, 0, 1, 3, 0, 0);
    constraint1.Get_Cq_b()->PasteClippedMatrix(Jrb, 0, 0, 1, 3, 0, 3);
    constraint2.Get_Cq_b()->PasteClippedMatrix(Jrb, 1, 0, 1, 3, 0, 3);
    constraint3.Get_Cq_b()->PasteClippedMatrix(Jrb, 2, 0, 1, 3, 0, 3);
}

void ChLinkPointFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = constraint1.Get_l_i() * factor;
    m_react.y() = constraint2.Get_l_i() * factor;
    m_react.z() = constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointFrame::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPointFrame::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
