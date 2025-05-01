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
#include "chrono/fea/ChLinkNodeFace.h"
#include "chrono/utils/ChUtilsGeometry.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeFace)

ChLinkNodeFace::ChLinkNodeFace() : react(VNULL), s2(0), s3(0), d(0) {}

ChLinkNodeFace::ChLinkNodeFace(const ChLinkNodeFace& other) : ChLinkBase(other) {
    react = other.react;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int ChLinkNodeFace::Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,
                               std::shared_ptr<ChNodeFEAxyz> anodeB1,
                               std::shared_ptr<ChNodeFEAxyz> anodeB2,
                               std::shared_ptr<ChNodeFEAxyz> anodeB3) {
    assert(anodeA && anodeB1 && anodeB2 && anodeB3);

    m_node = anodeA;
    m_triangle.node1 = anodeB1;
    m_triangle.node2 = anodeB2;
    m_triangle.node3 = anodeB3;

    constraint1.Get_tuple_a().SetVariables(*m_node);
    constraint1.Get_tuple_b().SetVariables(m_triangle);

    constraint2.Get_tuple_a().SetVariables(*m_node);
    constraint2.Get_tuple_b().SetVariables(m_triangle);

    constraint3.Get_tuple_a().SetVariables(*m_node);
    constraint3.Get_tuple_b().SetVariables(m_triangle);

    bool is_into;
    ChVector3d p_projected;
    this->d = utils::PointTriangleDistance(m_node->pos, m_triangle.node1->pos, m_triangle.node2->pos,
                                           m_triangle.node3->pos, s2, s3, is_into, p_projected);

    // double s1 = 1 - s2 - s3;

    return true;
}

void ChLinkNodeFace::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeFace::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkNodeFace::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkNodeFace::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
}

void ChLinkNodeFace::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N = Vcross(m_triangle.node2->pos - m_triangle.node1->pos, m_triangle.node3->pos - m_triangle.node1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res =
        m_node->GetPos() - s1 * m_triangle.node1->pos - s2 * m_triangle.node2->pos - s3 * m_triangle.node3->pos - N * d;

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

void ChLinkNodeFace::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    constraint1.SetLagrangeMultiplier(L(off_L + 0));
    constraint2.SetLagrangeMultiplier(L(off_L + 1));
    constraint3.SetLagrangeMultiplier(L(off_L + 2));

    constraint1.SetRightHandSide(Qc(off_L + 0));
    constraint2.SetRightHandSide(Qc(off_L + 1));
    constraint3.SetRightHandSide(Qc(off_L + 2));
}

void ChLinkNodeFace::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = constraint3.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeFace::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void ChLinkNodeFace::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
}

//// OBSOLETE will be removed in favor of IntLoadConstraint_C
void ChLinkNodeFace::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N = Vcross(m_triangle.node2->pos - m_triangle.node1->pos, m_triangle.node3->pos - m_triangle.node1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res =
        m_node->GetPos() - s1 * m_triangle.node1->pos - s2 * m_triangle.node2->pos - s3 * m_triangle.node3->pos - N * d;

    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());
}

//// OBSOLETE will be removed in favor of Int... functions
void ChLinkNodeFace::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

int mysgn(double val) {
    return (0 < val) - (val < 0);
}

void ChLinkNodeFace::LoadConstraintJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    if (d != 0) {
        double t2 = m_triangle.node1->pos.x() - m_triangle.node2->pos.x();
        double t3 = m_triangle.node1->pos.y() - m_triangle.node3->pos.y();
        double t4 = t2 * t3;
        double t5 = m_triangle.node1->pos.y() - m_triangle.node2->pos.y();
        double t6 = m_triangle.node1->pos.x() - m_triangle.node3->pos.x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = m_triangle.node1->pos.z() - m_triangle.node3->pos.z();
        double t9 = t2 * t8;
        double t10 = m_triangle.node1->pos.z() - m_triangle.node2->pos.z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = m_triangle.node2->pos.z() - m_triangle.node3->pos.z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = m_triangle.node2->pos.y() - m_triangle.node3->pos.y();
        double t28 = 1.0 / std::sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = m_triangle.node2->pos.x() - m_triangle.node3->pos.x();
        double t31 = mysgn(t17);
        double t32 = d * t19 * t28;
        double t33 = t13 * t25 * t27 * 2.0;
        double t34 = t15 * t19 * t29 * 2.0;
        double t35 = t33 + t34;
        double t36 = t13 * t25 * t30 * 2.0;
        double t59 = t18 * t19 * t31 * 2.0;
        double t37 = t36 - t59;
        double t38 = t15 * t29 * t30 * 2.0;
        double t39 = t18 * t27 * t31 * 2.0;
        double t40 = t38 + t39;
        double t41 = t3 * t13 * t25 * 2.0;
        double t42 = t8 * t15 * t29 * 2.0;
        double t43 = t41 + t42;
        double t44 = t6 * t13 * t25 * 2.0;
        double t61 = t8 * t18 * t31 * 2.0;
        double t45 = t44 - t61;
        double t46 = t6 * t15 * t29 * 2.0;
        double t47 = t3 * t18 * t31 * 2.0;
        double t48 = t46 + t47;
        double t49 = d * t10 * t28;
        double t50 = t5 * t13 * t25 * 2.0;
        double t51 = t10 * t15 * t29 * 2.0;
        double t52 = t50 + t51;
        double t53 = t2 * t13 * t25 * 2.0;
        double t63 = t10 * t18 * t31 * 2.0;
        double t54 = t53 - t63;
        double t55 = t2 * t15 * t29 * 2.0;
        double t56 = t5 * t18 * t31 * 2.0;
        double t57 = t55 + t56;
        double t58 = d * t28 * t30;
        double t60 = d * t3 * t28;
        double t62 = d * t2 * t28;

        Jxb1(0, 0) = -s1 + (d * t17 * t26 * t35) / 2;
        Jxb1(0, 1) = -t32 - (d * t17 * t26 * t37) / 2;
        Jxb1(0, 2) = -(d * t17 * t26 * t40) / 2 + d * t27 * t28;
        Jxb1(1, 0) = -(d * t11 * t26 * t35) / 2 + t32;
        Jxb1(1, 1) = -s1 + (d * t11 * t26 * t37) / 2;
        Jxb1(1, 2) = -t58 + (d * t11 * t26 * t40) / 2;
        Jxb1(2, 0) = -d * t27 * t28 + (d * t7 * t26 * t35) / 2;
        Jxb1(2, 1) = -(d * t7 * t26 * t37) / 2 + t58;
        Jxb1(2, 2) = -s1 - (d * t7 * t26 * t40) / 2;

        Jxb2(0, 0) = -s2 - (d * t17 * t26 * t43) / 2;
        Jxb2(0, 1) = d * t8 * t28 + (d * t17 * t26 * t45) / 2;
        Jxb2(0, 2) = -t60 + (d * t17 * t26 * t48) / 2;
        Jxb2(1, 0) = -d * t8 * t28 + (d * t11 * t26 * t43) / 2, Jxb2(1, 1) = -s2 - (d * t11 * t26 * t45) / 2;
        Jxb2(1, 2) = -(d * t11 * t26 * t48) / 2 + d * t6 * t28;
        Jxb2(2, 0) = -(d * t7 * t26 * t43) / 2 + t60;
        Jxb2(2, 1) = -d * t6 * t28 + (d * t7 * t26 * t45) / 2;
        Jxb2(2, 2) = -s2 + (d * t7 * t26 * t48) / 2;

        Jxb3(0, 0) = -s3 + (d * t17 * t26 * t52) / 2;
        Jxb3(0, 1) = -t49 - (d * t17 * t26 * t54) / 2;
        Jxb3(0, 2) = -(d * t17 * t26 * t57) / 2 + d * t5 * t28;
        Jxb3(1, 0) = -(d * t11 * t26 * t52) / 2 + t49;
        Jxb3(1, 1) = -s3 + (d * t11 * t26 * t54) / 2;
        Jxb3(1, 2) = -t62 + (d * t11 * t26 * t57) / 2;
        Jxb3(2, 0) = -d * t5 * t28 + (d * t7 * t26 * t52) / 2;
        Jxb3(2, 1) = -(d * t7 * t26 * t54) / 2 + t62;
        Jxb3(2, 2) = -s3 - (d * t7 * t26 * t57) / 2;
    } else {
        // simplified jacobian when offset d = 0;
        Jxb1.setZero();
        Jxb2.setZero();
        Jxb3.setZero();
        Jxb1.fillDiagonal(-s1);
        Jxb2.fillDiagonal(-s2);
        Jxb3.fillDiagonal(-s3);
    }

    constraint1.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(0);
    constraint2.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(1);
    constraint3.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(2);

    constraint1.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(0);
    constraint2.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(1);
    constraint3.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(2);

    constraint1.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(0);
    constraint2.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(1);
    constraint3.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(2);

    constraint1.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(0);
    constraint2.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(1);
    constraint3.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(2);
}

//// OBSOLETE will be removed in favor of Int... functions
void ChLinkNodeFace::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeFace::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeFace::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeFaceRot)

ChLinkNodeFaceRot::ChLinkNodeFaceRot() : react(VNULL), s2(0), s3(0), d(0) {}

ChLinkNodeFaceRot::ChLinkNodeFaceRot(const ChLinkNodeFaceRot& other) : ChLinkBase(other) {
    react = other.react;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int ChLinkNodeFaceRot::Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeB1,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeB2,
                                  std::shared_ptr<ChNodeFEAxyzrot> nodeB3) {
    assert(nodeA && nodeB1 && nodeB2 && nodeB3);

    m_node = nodeA;
    m_triangle.node1 = nodeB1;
    m_triangle.node2 = nodeB2;
    m_triangle.node3 = nodeB3;

    constraint1.Get_tuple_a().SetVariables(*m_node);
    constraint1.Get_tuple_b().SetVariables(m_triangle);

    constraint2.Get_tuple_a().SetVariables(*m_node);
    constraint2.Get_tuple_b().SetVariables(m_triangle);

    constraint3.Get_tuple_a().SetVariables(*m_node);
    constraint3.Get_tuple_b().SetVariables(m_triangle);

    bool is_into;
    ChVector3d p_projected;
    this->d = utils::PointTriangleDistance(m_node->pos, m_triangle.node1->GetPos(), m_triangle.node2->GetPos(),
                                           m_triangle.node3->GetPos(), s2, s3, is_into, p_projected);

    // double s1 = 1 - s2 - s3;

    return true;
}

void ChLinkNodeFaceRot::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeFaceRot::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkNodeFaceRot::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkNodeFaceRot::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                            const ChVectorDynamic<>& L,  // the L vector
                                            const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
}

void ChLinkNodeFaceRot::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                            ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                            const double c,            // a scaling factor
                                            bool do_clamp,             // apply clamping to c*C?
                                            double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N = Vcross(m_triangle.node2->GetPos() - m_triangle.node1->GetPos(),
                          m_triangle.node3->GetPos() - m_triangle.node1->GetPos());
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res = m_node->GetPos() - s1 * m_triangle.node1->GetPos() - s2 * m_triangle.node2->GetPos() -
                     s3 * m_triangle.node3->GetPos() - N * d;

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

void ChLinkNodeFaceRot::IntToDescriptor(const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,
                                        const ChVectorDynamic<>& L,
                                        const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    constraint1.SetLagrangeMultiplier(L(off_L + 0));
    constraint2.SetLagrangeMultiplier(L(off_L + 1));
    constraint3.SetLagrangeMultiplier(L(off_L + 2));

    constraint1.SetRightHandSide(Qc(off_L + 0));
    constraint2.SetRightHandSide(Qc(off_L + 1));
    constraint3.SetRightHandSide(Qc(off_L + 2));
}

void ChLinkNodeFaceRot::IntFromDescriptor(const unsigned int off_v,
                                          ChStateDelta& v,
                                          const unsigned int off_L,
                                          ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = constraint3.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeFaceRot::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void ChLinkNodeFaceRot::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
}

//// OBSOLETE will be removed in favor of IntLoadConstraint_C
void ChLinkNodeFaceRot::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N = Vcross(m_triangle.node2->GetPos() - m_triangle.node1->GetPos(),
                          m_triangle.node3->GetPos() - m_triangle.node1->GetPos());
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res = m_node->GetPos() - s1 * m_triangle.node1->GetPos() - s2 * m_triangle.node2->GetPos() -
                     s3 * m_triangle.node3->GetPos() - N * d;

    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());
}

//// OBSOLETE will be removed in favor of Int... functions
void ChLinkNodeFaceRot::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeFaceRot::LoadConstraintJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    if (d != 0) {
        double t2 = m_triangle.node1->GetPos().x() - m_triangle.node2->GetPos().x();
        double t3 = m_triangle.node1->GetPos().y() - m_triangle.node3->GetPos().y();
        double t4 = t2 * t3;
        double t5 = m_triangle.node1->GetPos().y() - m_triangle.node2->GetPos().y();
        double t6 = m_triangle.node1->GetPos().x() - m_triangle.node3->GetPos().x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = m_triangle.node1->GetPos().z() - m_triangle.node3->GetPos().z();
        double t9 = t2 * t8;
        double t10 = m_triangle.node1->GetPos().z() - m_triangle.node2->GetPos().z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = m_triangle.node2->GetPos().z() - m_triangle.node3->GetPos().z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = m_triangle.node2->GetPos().y() - m_triangle.node3->GetPos().y();
        double t28 = 1.0 / std::sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = m_triangle.node2->GetPos().x() - m_triangle.node3->GetPos().x();
        double t31 = mysgn(t17);
        double t32 = d * t19 * t28;
        double t33 = t13 * t25 * t27 * 2.0;
        double t34 = t15 * t19 * t29 * 2.0;
        double t35 = t33 + t34;
        double t36 = t13 * t25 * t30 * 2.0;
        double t59 = t18 * t19 * t31 * 2.0;
        double t37 = t36 - t59;
        double t38 = t15 * t29 * t30 * 2.0;
        double t39 = t18 * t27 * t31 * 2.0;
        double t40 = t38 + t39;
        double t41 = t3 * t13 * t25 * 2.0;
        double t42 = t8 * t15 * t29 * 2.0;
        double t43 = t41 + t42;
        double t44 = t6 * t13 * t25 * 2.0;
        double t61 = t8 * t18 * t31 * 2.0;
        double t45 = t44 - t61;
        double t46 = t6 * t15 * t29 * 2.0;
        double t47 = t3 * t18 * t31 * 2.0;
        double t48 = t46 + t47;
        double t49 = d * t10 * t28;
        double t50 = t5 * t13 * t25 * 2.0;
        double t51 = t10 * t15 * t29 * 2.0;
        double t52 = t50 + t51;
        double t53 = t2 * t13 * t25 * 2.0;
        double t63 = t10 * t18 * t31 * 2.0;
        double t54 = t53 - t63;
        double t55 = t2 * t15 * t29 * 2.0;
        double t56 = t5 * t18 * t31 * 2.0;
        double t57 = t55 + t56;
        double t58 = d * t28 * t30;
        double t60 = d * t3 * t28;
        double t62 = d * t2 * t28;

        Jxb1(0, 0) = -s1 + (d * t17 * t26 * t35) / 2;
        Jxb1(0, 1) = -t32 - (d * t17 * t26 * t37) / 2;
        Jxb1(0, 2) = -(d * t17 * t26 * t40) / 2 + d * t27 * t28;
        Jxb1(1, 0) = -(d * t11 * t26 * t35) / 2 + t32;
        Jxb1(1, 1) = -s1 + (d * t11 * t26 * t37) / 2;
        Jxb1(1, 2) = -t58 + (d * t11 * t26 * t40) / 2;
        Jxb1(2, 0) = -d * t27 * t28 + (d * t7 * t26 * t35) / 2;
        Jxb1(2, 1) = -(d * t7 * t26 * t37) / 2 + t58;
        Jxb1(2, 2) = -s1 - (d * t7 * t26 * t40) / 2;

        Jxb2(0, 0) = -s2 - (d * t17 * t26 * t43) / 2;
        Jxb2(0, 1) = d * t8 * t28 + (d * t17 * t26 * t45) / 2;
        Jxb2(0, 2) = -t60 + (d * t17 * t26 * t48) / 2;
        Jxb2(1, 0) = -d * t8 * t28 + (d * t11 * t26 * t43) / 2, Jxb2(1, 1) = -s2 - (d * t11 * t26 * t45) / 2;
        Jxb2(1, 2) = -(d * t11 * t26 * t48) / 2 + d * t6 * t28;
        Jxb2(2, 0) = -(d * t7 * t26 * t43) / 2 + t60;
        Jxb2(2, 1) = -d * t6 * t28 + (d * t7 * t26 * t45) / 2;
        Jxb2(2, 2) = -s2 + (d * t7 * t26 * t48) / 2;

        Jxb3(0, 0) = -s3 + (d * t17 * t26 * t52) / 2;
        Jxb3(0, 1) = -t49 - (d * t17 * t26 * t54) / 2;
        Jxb3(0, 2) = -(d * t17 * t26 * t57) / 2 + d * t5 * t28;
        Jxb3(1, 0) = -(d * t11 * t26 * t52) / 2 + t49;
        Jxb3(1, 1) = -s3 + (d * t11 * t26 * t54) / 2;
        Jxb3(1, 2) = -t62 + (d * t11 * t26 * t57) / 2;
        Jxb3(2, 0) = -d * t5 * t28 + (d * t7 * t26 * t52) / 2;
        Jxb3(2, 1) = -(d * t7 * t26 * t54) / 2 + t62;
        Jxb3(2, 2) = -s3 - (d * t7 * t26 * t57) / 2;
    } else {
        // simplified jacobian when offset d = 0;
        Jxb1.setZero();
        Jxb2.setZero();
        Jxb3.setZero();
        Jxb1.fillDiagonal(-s1);
        Jxb2.fillDiagonal(-s2);
        Jxb3.fillDiagonal(-s3);
    }

    constraint1.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(0);
    constraint2.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(1);
    constraint3.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(2);

    constraint1.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(0);
    constraint2.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(1);
    constraint3.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(2);

    constraint1.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(0);
    constraint2.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(1);
    constraint3.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(2);

    constraint1.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(0);
    constraint2.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(1);
    constraint3.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(2);
}

//// OBSOLETE will be removed in favor of Int... functions
void ChLinkNodeFaceRot::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeFaceRot::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeFaceRot::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono