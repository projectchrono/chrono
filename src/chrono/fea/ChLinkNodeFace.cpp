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

// -----------------------------------------------------------------------------
// TODO:
// Obsolete PointNodeXYZrot and TriangleNodesXYZrot when ChNodeFEAxyzrot is derived from ChNodeFEAxyz

// Class for a point given by an xyz FEA node.
struct PointNodeXYZ : public ChLinkNodeFace::PointNode {
    std::shared_ptr<ChNodeFEAxyz> node;

    virtual ChLinkNodeFace::FeaNodeType GetType() const override { return ChLinkNodeFace::FeaNodeType::XYZ; }
    virtual unsigned int GetNumCoords() const override { return 3; }
    virtual const ChVector3d& GetPos() const override { return node->GetPos(); }
    virtual ChConstraintTuple* CreateConstraintTuple() override {
        return new ChConstraintTuple_1vars<3>(&node->Variables());
    }
};

// Class for a point given by an xyzrot FEA node.
struct PointNodeXYZrot : public ChLinkNodeFace::PointNode {
    std::shared_ptr<ChNodeFEAxyzrot> node;

    virtual ChLinkNodeFace::FeaNodeType GetType() const override { return ChLinkNodeFace::FeaNodeType::XYZrot; }
    virtual unsigned int GetNumCoords() const override { return 6; }
    virtual const ChVector3d& GetPos() const override { return node->GetPos(); }
    virtual ChConstraintTuple* CreateConstraintTuple() override {
        return new ChConstraintTuple_1vars<6>(&node->Variables());
    }
};

// Class encapsulating a triangle given by three xyz FEA nodes.
struct TriangleNodesXYZ : public ChLinkNodeFace::TriangleNodes {
    std::shared_ptr<ChNodeFEAxyz> node1;
    std::shared_ptr<ChNodeFEAxyz> node2;
    std::shared_ptr<ChNodeFEAxyz> node3;

    virtual ChLinkNodeFace::FeaNodeType GetType() const override { return ChLinkNodeFace::FeaNodeType::XYZ; }
    virtual const ChVector3d& GetPos1() const override { return node1->GetPos(); }
    virtual const ChVector3d& GetPos2() const override { return node2->GetPos(); }
    virtual const ChVector3d& GetPos3() const override { return node3->GetPos(); }
    virtual unsigned int GetNumCoords() const override { return 3 * 3; }
    virtual ChConstraintTuple* CreateConstraintTuple() override {
        return new ChConstraintTuple_3vars<3, 3, 3>(&node1->Variables(), &node2->Variables(), &node3->Variables());
    }
};

// Class encapsulating a triangle given by three xyzrot FEA nodes.
struct TriangleNodesXYZrot : public ChLinkNodeFace::TriangleNodes {
    std::shared_ptr<ChNodeFEAxyzrot> node1;
    std::shared_ptr<ChNodeFEAxyzrot> node2;
    std::shared_ptr<ChNodeFEAxyzrot> node3;

    virtual ChLinkNodeFace::FeaNodeType GetType() const override { return ChLinkNodeFace::FeaNodeType::XYZrot; }
    virtual const ChVector3d& GetPos1() const override { return node1->GetPos(); }
    virtual const ChVector3d& GetPos2() const override { return node2->GetPos(); }
    virtual const ChVector3d& GetPos3() const override { return node3->GetPos(); }
    virtual unsigned int GetNumCoords() const override { return 3 * 6; }
    virtual ChConstraintTuple* CreateConstraintTuple() override {
        return new ChConstraintTuple_3vars<6, 6, 6>(&node1->Variables(), &node2->Variables(), &node3->Variables());
    }
};

// -----------------------------------------------------------------------------

ChLinkNodeFace::ChLinkNodeFace() : m_initialized(false), m_react(VNULL), m_s2(0), m_s3(0), m_d(0) {}

ChLinkNodeFace::ChLinkNodeFace(const ChLinkNodeFace& other) : ChLinkBase(other), m_initialized(false) {
    m_react = other.m_react;
    m_s2 = other.m_s2;
    m_s3 = other.m_s3;
    m_d = other.m_d;
}

void ChLinkNodeFace::Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                std::shared_ptr<ChNodeFEAxyz> nodeB1,
                                std::shared_ptr<ChNodeFEAxyz> nodeB2,
                                std::shared_ptr<ChNodeFEAxyz> nodeB3) {
    assert(nodeA && nodeB1 && nodeB2 && nodeB3);
    assert(!m_initialized);

    auto point = chrono_types::make_unique<PointNodeXYZ>();
    point->node = nodeA;
    m_point = std::move(point);

    auto triangle = chrono_types::make_unique<TriangleNodesXYZ>();
    triangle->node1 = nodeB1;
    triangle->node2 = nodeB2;
    triangle->node3 = nodeB3;
    m_triangle = std::move(triangle);

    CompleteInitialization();

    m_initialized = true;
}

void ChLinkNodeFace::Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB1,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB2,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB3) {
    assert(nodeA && nodeB1 && nodeB2 && nodeB3);
    assert(!m_initialized);

    auto point = chrono_types::make_unique<PointNodeXYZ>();
    point->node = nodeA;
    m_point = std::move(point);

    auto triangle = chrono_types::make_unique<TriangleNodesXYZrot>();
    triangle->node1 = nodeB1;
    triangle->node2 = nodeB2;
    triangle->node3 = nodeB3;
    m_triangle = std::move(triangle);

    CompleteInitialization();

    m_initialized = true;
}

void ChLinkNodeFace::Initialize(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                std::shared_ptr<ChNodeFEAxyz> nodeB1,
                                std::shared_ptr<ChNodeFEAxyz> nodeB2,
                                std::shared_ptr<ChNodeFEAxyz> nodeB3) {
    assert(nodeA && nodeB1 && nodeB2 && nodeB3);
    assert(!m_initialized);

    auto point = chrono_types::make_unique<PointNodeXYZrot>();
    point->node = nodeA;
    m_point = std::move(point);

    auto triangle = chrono_types::make_unique<TriangleNodesXYZ>();
    triangle->node1 = nodeB1;
    triangle->node2 = nodeB2;
    triangle->node3 = nodeB3;
    m_triangle = std::move(triangle);

    CompleteInitialization();

    m_initialized = true;
}

void ChLinkNodeFace::Initialize(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB1,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB2,
                                std::shared_ptr<ChNodeFEAxyzrot> nodeB3) {
    assert(nodeA && nodeB1 && nodeB2 && nodeB3);
    assert(!m_initialized);

    auto point = chrono_types::make_unique<PointNodeXYZrot>();
    point->node = nodeA;
    m_point = std::move(point);

    auto triangle = chrono_types::make_unique<TriangleNodesXYZrot>();
    triangle->node1 = nodeB1;
    triangle->node2 = nodeB2;
    triangle->node3 = nodeB3;
    m_triangle = std::move(triangle);

    CompleteInitialization();

    m_initialized = true;
}

void ChLinkNodeFace::CompleteInitialization() {
    // Create and set the two tuples for the 3 embedded constraints
    constraint1.SetTuples(m_point->CreateConstraintTuple(), m_triangle->CreateConstraintTuple());
    constraint2.SetTuples(m_point->CreateConstraintTuple(), m_triangle->CreateConstraintTuple());
    constraint3.SetTuples(m_point->CreateConstraintTuple(), m_triangle->CreateConstraintTuple());

    bool is_into;
    ChVector3d p_projected;
    m_d = utils::PointTriangleDistance(m_point->GetPos(), m_triangle->GetPos1(), m_triangle->GetPos2(),
                                       m_triangle->GetPos3(), m_s2, m_s3, is_into, p_projected);
    m_s1 = 1 - m_s2 - m_s3;
}


void ChLinkNodeFace::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);
}

// -----------------------------------------------------------------------------

void ChLinkNodeFace::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = m_react.x();
    L(off_L + 1) = m_react.y();
    L(off_L + 2) = m_react.z();
}

void ChLinkNodeFace::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    m_react.x() = L(off_L + 0);
    m_react.y() = L(off_L + 1);
    m_react.z() = L(off_L + 2);
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

    ChVector3d N = Vcross(m_triangle->GetPos2() - m_triangle->GetPos1(), m_triangle->GetPos3() - m_triangle->GetPos1());
    N.Normalize();

    ChVector3d res = m_point->GetPos() - m_s1 * m_triangle->GetPos1() - m_s2 * m_triangle->GetPos2() -
                     m_s3 * m_triangle->GetPos3() - N * m_d;

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

// -----------------------------------------------------------------------------

void ChLinkNodeFace::InjectConstraints(ChSystemDescriptor& descriptor) {
    ////if (!IsActive())
    ////    return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void ChLinkNodeFace::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0);
    constraint2.SetRightHandSide(0);
    constraint3.SetRightHandSide(0);
}

//// OBSOLETE will be removed in favor of IntLoadConstraint_C
void ChLinkNodeFace::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N = Vcross(m_triangle->GetPos2() - m_triangle->GetPos1(), m_triangle->GetPos3() - m_triangle->GetPos1());
    N.Normalize();

    ChVector3d res = m_point->GetPos() - m_s1 * m_triangle->GetPos1() - m_s2 * m_triangle->GetPos2() -
                     m_s3 * m_triangle->GetPos3() - N * m_d;

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
    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    if (m_d != 0) {
        double t2 = m_triangle->GetPos1().x() - m_triangle->GetPos2().x();
        double t3 = m_triangle->GetPos1().y() - m_triangle->GetPos3().y();
        double t4 = t2 * t3;
        double t5 = m_triangle->GetPos1().y() - m_triangle->GetPos2().y();
        double t6 = m_triangle->GetPos1().x() - m_triangle->GetPos3().x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = m_triangle->GetPos1().z() - m_triangle->GetPos3().z();
        double t9 = t2 * t8;
        double t10 = m_triangle->GetPos1().z() - m_triangle->GetPos2().z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = m_triangle->GetPos2().z() - m_triangle->GetPos3().z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = m_triangle->GetPos2().y() - m_triangle->GetPos3().y();
        double t28 = 1.0 / std::sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = m_triangle->GetPos2().x() - m_triangle->GetPos3().x();
        double t31 = mysgn(t17);
        double t32 = m_d * t19 * t28;
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
        double t49 = m_d * t10 * t28;
        double t50 = t5 * t13 * t25 * 2.0;
        double t51 = t10 * t15 * t29 * 2.0;
        double t52 = t50 + t51;
        double t53 = t2 * t13 * t25 * 2.0;
        double t63 = t10 * t18 * t31 * 2.0;
        double t54 = t53 - t63;
        double t55 = t2 * t15 * t29 * 2.0;
        double t56 = t5 * t18 * t31 * 2.0;
        double t57 = t55 + t56;
        double t58 = m_d * t28 * t30;
        double t60 = m_d * t3 * t28;
        double t62 = m_d * t2 * t28;

        Jxb1(0, 0) = -m_s1 + (m_d * t17 * t26 * t35) / 2;
        Jxb1(0, 1) = -t32 - (m_d * t17 * t26 * t37) / 2;
        Jxb1(0, 2) = -(m_d * t17 * t26 * t40) / 2 + m_d * t27 * t28;
        Jxb1(1, 0) = -(m_d * t11 * t26 * t35) / 2 + t32;
        Jxb1(1, 1) = -m_s1 + (m_d * t11 * t26 * t37) / 2;
        Jxb1(1, 2) = -t58 + (m_d * t11 * t26 * t40) / 2;
        Jxb1(2, 0) = -m_d * t27 * t28 + (m_d * t7 * t26 * t35) / 2;
        Jxb1(2, 1) = -(m_d * t7 * t26 * t37) / 2 + t58;
        Jxb1(2, 2) = -m_s1 - (m_d * t7 * t26 * t40) / 2;

        Jxb2(0, 0) = -m_s2 - (m_d * t17 * t26 * t43) / 2;
        Jxb2(0, 1) = m_d * t8 * t28 + (m_d * t17 * t26 * t45) / 2;
        Jxb2(0, 2) = -t60 + (m_d * t17 * t26 * t48) / 2;
        Jxb2(1, 0) = -m_d * t8 * t28 + (m_d * t11 * t26 * t43) / 2;
        Jxb2(1, 1) = -m_s2 - (m_d * t11 * t26 * t45) / 2;
        Jxb2(1, 2) = -(m_d * t11 * t26 * t48) / 2 + m_d * t6 * t28;
        Jxb2(2, 0) = -(m_d * t7 * t26 * t43) / 2 + t60;
        Jxb2(2, 1) = -m_d * t6 * t28 + (m_d * t7 * t26 * t45) / 2;
        Jxb2(2, 2) = -m_s2 + (m_d * t7 * t26 * t48) / 2;

        Jxb3(0, 0) = -m_s3 + (m_d * t17 * t26 * t52) / 2;
        Jxb3(0, 1) = -t49 - (m_d * t17 * t26 * t54) / 2;
        Jxb3(0, 2) = -(m_d * t17 * t26 * t57) / 2 + m_d * t5 * t28;
        Jxb3(1, 0) = -(m_d * t11 * t26 * t52) / 2 + t49;
        Jxb3(1, 1) = -m_s3 + (m_d * t11 * t26 * t54) / 2;
        Jxb3(1, 2) = -t62 + (m_d * t11 * t26 * t57) / 2;
        Jxb3(2, 0) = -m_d * t5 * t28 + (m_d * t7 * t26 * t52) / 2;
        Jxb3(2, 1) = -(m_d * t7 * t26 * t54) / 2 + t62;
        Jxb3(2, 2) = -m_s3 - (m_d * t7 * t26 * t57) / 2;
    } else {
        // simplified jacobian when offset d = 0;
        Jxb1.setZero();
        Jxb2.setZero();
        Jxb3.setZero();
        Jxb1.fillDiagonal(-m_s1);
        Jxb2.fillDiagonal(-m_s2);
        Jxb3.fillDiagonal(-m_s3);
    }

    switch (m_point->GetType()) {
        case FeaNodeType::XYZ: {
            auto tuple1_a = static_cast<ChConstraintTuple_1vars<3>*>(constraint1.TupleA());
            auto tuple2_a = static_cast<ChConstraintTuple_1vars<3>*>(constraint2.TupleA());
            auto tuple3_a = static_cast<ChConstraintTuple_1vars<3>*>(constraint3.TupleA());

            tuple1_a->Cq1().segment(0, 3) = Jxa.row(0);
            tuple2_a->Cq1().segment(0, 3) = Jxa.row(1);
            tuple3_a->Cq1().segment(0, 3) = Jxa.row(2);

            break;
        }
        case FeaNodeType::XYZrot: {
            auto tuple1_a = static_cast<ChConstraintTuple_1vars<6>*>(constraint1.TupleA());
            auto tuple2_a = static_cast<ChConstraintTuple_1vars<6>*>(constraint2.TupleA());
            auto tuple3_a = static_cast<ChConstraintTuple_1vars<6>*>(constraint3.TupleA());

            tuple1_a->Cq1().segment(0, 3) = Jxa.row(0);
            tuple2_a->Cq1().segment(0, 3) = Jxa.row(1);
            tuple3_a->Cq1().segment(0, 3) = Jxa.row(2);

            break;
        }
    }

    switch (m_triangle->GetType()) {
        case FeaNodeType::XYZ: {
            auto tuple1_b = static_cast<ChConstraintTuple_3vars<3, 3, 3>*>(constraint1.TupleB());
            auto tuple2_b = static_cast<ChConstraintTuple_3vars<3, 3, 3>*>(constraint2.TupleB());
            auto tuple3_b = static_cast<ChConstraintTuple_3vars<3, 3, 3>*>(constraint3.TupleB());

            tuple1_b->Cq1().segment(0, 3) = Jxb1.row(0);
            tuple2_b->Cq1().segment(0, 3) = Jxb1.row(1);
            tuple3_b->Cq1().segment(0, 3) = Jxb1.row(2);

            tuple1_b->Cq2().segment(0, 3) = Jxb2.row(0);
            tuple2_b->Cq2().segment(0, 3) = Jxb2.row(1);
            tuple3_b->Cq2().segment(0, 3) = Jxb2.row(2);

            tuple1_b->Cq3().segment(0, 3) = Jxb3.row(0);
            tuple2_b->Cq3().segment(0, 3) = Jxb3.row(1);
            tuple3_b->Cq3().segment(0, 3) = Jxb3.row(2);

            break;
        }
        case FeaNodeType::XYZrot: {
            auto tuple1_b = static_cast<ChConstraintTuple_3vars<6, 6, 6>*>(constraint1.TupleB());
            auto tuple2_b = static_cast<ChConstraintTuple_3vars<6, 6, 6>*>(constraint2.TupleB());
            auto tuple3_b = static_cast<ChConstraintTuple_3vars<6, 6, 6>*>(constraint3.TupleB());

            tuple1_b->Cq1().segment(0, 3) = Jxb1.row(0);
            tuple2_b->Cq1().segment(0, 3) = Jxb1.row(1);
            tuple3_b->Cq1().segment(0, 3) = Jxb1.row(2);

            tuple1_b->Cq2().segment(0, 3) = Jxb2.row(0);
            tuple2_b->Cq2().segment(0, 3) = Jxb2.row(1);
            tuple3_b->Cq2().segment(0, 3) = Jxb2.row(2);

            tuple1_b->Cq3().segment(0, 3) = Jxb3.row(0);
            tuple2_b->Cq3().segment(0, 3) = Jxb3.row(1);
            tuple3_b->Cq3().segment(0, 3) = Jxb3.row(2);

            break;
        }
    }
}

//// OBSOLETE will be removed in favor of Int... functions
void ChLinkNodeFace::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = constraint1.GetLagrangeMultiplier() * factor;
    m_react.y() = constraint2.GetLagrangeMultiplier() * factor;
    m_react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeFace::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeFace::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono