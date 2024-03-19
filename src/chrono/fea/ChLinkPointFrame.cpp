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
#include "chrono/fea/ChLinkPointFrame.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointFrameGeneric)

ChLinkPointFrameGeneric::ChLinkPointFrameGeneric(bool mc_x, bool mc_y, bool mc_z) : m_react(VNULL), m_csys(CSYSNORM) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
}

ChLinkPointFrameGeneric::ChLinkPointFrameGeneric(const ChLinkPointFrameGeneric& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
    c_x = other.c_x;
    c_y = other.c_y;
    c_z = other.c_z;
}

ChFrame<> ChLinkPointFrameGeneric::GetFrameNodeAbs() const {
        return ChFrame<>(m_csys >> *m_body);
}

int ChLinkPointFrameGeneric::Initialize(std::shared_ptr<ChNodeFEAxyz> node,  ///< xyz node (point) to join
                                        std::shared_ptr<ChBodyFrame> body,   ///< body (frame) to join
                                        chrono::ChCoordsys<> csys_abs) {
    assert(node && body);

    m_body = body;
    m_node = node;

    m_constraint1.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint2.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint3.SetVariables(&(node->Variables()), &(body->Variables()));

    m_csys = m_body->GetCoordsys().TransformParentToLocal(csys_abs);

    return true;
}

int ChLinkPointFrameGeneric::Initialize(std::shared_ptr<ChNodeFEAxyz> node,
                                        std::shared_ptr<ChBodyFrame> body,
                                        ChVector3d* pos) {
    ChVector3d pos_abs = pos ? *pos : node->GetPos();

    return this->Initialize(node, body, ChCoordsys<>(pos_abs));
}

void ChLinkPointFrameGeneric::SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
}

void ChLinkPointFrameGeneric::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkPointFrameGeneric::GetConstraintViolation() const {
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVectorN<double, 3> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointFrameGeneric::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    int nc = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        L(off_L + nc) = m_react.x();
        nc++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        L(off_L + nc) = m_react.y();
        nc++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        L(off_L + nc) = m_react.z();
        // nc++;
    }
}

void ChLinkPointFrameGeneric::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    int nc = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_react.x() = L(off_L + nc);
        nc++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_react.y() = L(off_L + nc);
        nc++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_react.z() = L(off_L + nc);
        // nc++;
    }
}

void ChLinkPointFrameGeneric::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                  ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                  const ChVectorDynamic<>& L,  // the L vector
                                                  const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.MultiplyTandAdd(R, L(off_L + cnt) * c);
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint1.MultiplyTandAdd(R, L(off_L + cnt) * c);
        cnt++;
    }
    if (c_y && this->m_constraint3.IsActive()) {
        m_constraint1.MultiplyTandAdd(R, L(off_L + cnt) * c);
        // cnt++;
    }
}

void ChLinkPointFrameGeneric::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                                  ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                                  const double c,            // a scaling factor
                                                  bool do_clamp,             // apply clamping to c*C?
                                                  double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVector3d cres = res * c;

    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        Qc(off_L + cnt) += cres.x();
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        Qc(off_L + cnt) += cres.y();
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        Qc(off_L + cnt) += cres.z();
        // cnt++;
    }
}

void ChLinkPointFrameGeneric::IntToDescriptor(const unsigned int off_v,
                                              const ChStateDelta& v,
                                              const ChVectorDynamic<>& R,
                                              const unsigned int off_L,
                                              const ChVectorDynamic<>& L,
                                              const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.Set_l_i(L(off_L + cnt));
        m_constraint1.Set_b_i(Qc(off_L + cnt));
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint2.Set_l_i(L(off_L + cnt));
        m_constraint2.Set_b_i(Qc(off_L + cnt));
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_constraint3.Set_l_i(L(off_L + cnt));
        m_constraint3.Set_b_i(Qc(off_L + cnt));
        // cnt++;
    }
}

void ChLinkPointFrameGeneric::IntFromDescriptor(const unsigned int off_v,
                                                ChStateDelta& v,
                                                const unsigned int off_L,
                                                ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        L(off_L + cnt) = m_constraint1.Get_l_i();
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        L(off_L + cnt) = m_constraint2.Get_l_i();
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        L(off_L + cnt) = m_constraint3.Get_l_i();
        // cnt++;
    }
}

// SOLVER INTERFACES

void ChLinkPointFrameGeneric::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!IsActive())
        return;

    if (c_x && this->m_constraint1.IsActive())
        mdescriptor.InsertConstraint(&m_constraint1);

    if (c_y && this->m_constraint2.IsActive())
        mdescriptor.InsertConstraint(&m_constraint2);

    if (c_z && this->m_constraint3.IsActive())
        mdescriptor.InsertConstraint(&m_constraint3);
}

void ChLinkPointFrameGeneric::ConstraintsBiReset() {
    m_constraint1.Set_b_i(0.);
    m_constraint2.Set_b_i(0.);
    m_constraint3.Set_b_i(0.);
}

void ChLinkPointFrameGeneric::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_node)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));

    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.Set_b_i(m_constraint1.Get_b_i() + factor * res.x());
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint2.Set_b_i(m_constraint2.Get_b_i() + factor * res.y());
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_constraint3.Set_b_i(m_constraint3.Get_b_i() + factor * res.z());
    }
}

void ChLinkPointFrameGeneric::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkPointFrameGeneric::ConstraintsLoadJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> Jxn = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChStarMatrix33<> atilde(Aow.transpose() * (m_node->GetPos() - m_body->GetPos()));

    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    if (c_x) {
        m_constraint1.Get_Cq_a().segment(0, 3) = Jxn.row(0);
        m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
        m_constraint1.Get_Cq_b().segment(3, 3) = Jrb.row(0);
    }
    if (c_y) {
        m_constraint2.Get_Cq_a().segment(0, 3) = Jxn.row(1);
        m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
        m_constraint2.Get_Cq_b().segment(3, 3) = Jrb.row(1);
    }
    if (c_z) {
        m_constraint3.Get_Cq_a().segment(0, 3) = Jxn.row(2);
        m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);
        m_constraint3.Get_Cq_b().segment(3, 3) = Jrb.row(2);
    }
}

void ChLinkPointFrameGeneric::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = m_constraint1.Get_l_i() * factor;
    m_react.y() = m_constraint2.Get_l_i() * factor;
    m_react.z() = m_constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointFrameGeneric::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkPointFrameGeneric::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointFrame)

ChLinkPointFrame::ChLinkPointFrame() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkPointFrame::ChLinkPointFrame(const ChLinkPointFrame& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
}

ChFrame<> ChLinkPointFrame::GetFrameNodeAbs() const {
        return ChFrame<>(m_csys >> *m_body);
}

int ChLinkPointFrame::Initialize(std::shared_ptr<ChNodeFEAxyz> node,
                                 std::shared_ptr<ChBodyFrame> body,
                                 const ChVector3d* pos) {
    assert(node && body);

    m_body = body;
    m_node = node;

    m_constraint1.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint2.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint3.SetVariables(&(node->Variables()), &(body->Variables()));

    ChVector3d pos_abs = pos ? *pos : node->GetPos();
    SetAttachPositionInAbsoluteCoords(pos_abs);

    return true;
}

void ChLinkPointFrame::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkPointFrame::GetConstraintViolation() const {
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVectorN<double, 3> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
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

    m_constraint1.MultiplyTandAdd(R, L(off_L + 0) * c);
    m_constraint2.MultiplyTandAdd(R, L(off_L + 1) * c);
    m_constraint3.MultiplyTandAdd(R, L(off_L + 2) * c);
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

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
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

void ChLinkPointFrame::IntToDescriptor(const unsigned int off_v,
                                       const ChStateDelta& v,
                                       const ChVectorDynamic<>& R,
                                       const unsigned int off_L,
                                       const ChVectorDynamic<>& L,
                                       const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_constraint1.Set_l_i(L(off_L + 0));
    m_constraint2.Set_l_i(L(off_L + 1));
    m_constraint3.Set_l_i(L(off_L + 2));

    m_constraint1.Set_b_i(Qc(off_L + 0));
    m_constraint2.Set_b_i(Qc(off_L + 1));
    m_constraint3.Set_b_i(Qc(off_L + 2));
}

void ChLinkPointFrame::IntFromDescriptor(const unsigned int off_v,
                                         ChStateDelta& v,
                                         const unsigned int off_L,
                                         ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.Get_l_i();
    L(off_L + 1) = m_constraint2.Get_l_i();
    L(off_L + 2) = m_constraint3.Get_l_i();
}

// SOLVER INTERFACES

void ChLinkPointFrame::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!IsActive())
        return;

    mdescriptor.InsertConstraint(&m_constraint1);
    mdescriptor.InsertConstraint(&m_constraint2);
    mdescriptor.InsertConstraint(&m_constraint3);
}

void ChLinkPointFrame::ConstraintsBiReset() {
    m_constraint1.Set_b_i(0.);
    m_constraint2.Set_b_i(0.);
    m_constraint3.Set_b_i(0.);
}

void ChLinkPointFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_node)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));

    m_constraint1.Set_b_i(m_constraint1.Get_b_i() + factor * res.x());
    m_constraint2.Set_b_i(m_constraint2.Get_b_i() + factor * res.y());
    m_constraint3.Set_b_i(m_constraint3.Get_b_i() + factor * res.z());
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

    ChMatrix33<> Jxn = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChStarMatrix33<> atilde(Aow.transpose() * (m_node->GetPos() - m_body->GetPos()));
    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    m_constraint1.Get_Cq_a().segment(0, 3) = Jxn.row(0);
    m_constraint2.Get_Cq_a().segment(0, 3) = Jxn.row(1);
    m_constraint3.Get_Cq_a().segment(0, 3) = Jxn.row(2);

    m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
    m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
    m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);

    m_constraint1.Get_Cq_b().segment(3, 3) = Jrb.row(0);
    m_constraint2.Get_Cq_b().segment(3, 3) = Jrb.row(1);
    m_constraint3.Get_Cq_b().segment(3, 3) = Jrb.row(2);
}

void ChLinkPointFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = m_constraint1.Get_l_i() * factor;
    m_react.y() = m_constraint2.Get_l_i() * factor;
    m_react.z() = m_constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointFrame::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkPointFrame::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
