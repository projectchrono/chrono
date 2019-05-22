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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChLinkPointTriface.h"
#include "chrono/collision/ChCCollisionUtils.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointTriface)

ChLinkPointTriface::ChLinkPointTriface() : react(VNULL), s2(0), s3(0), d(0) {}

ChLinkPointTriface::ChLinkPointTriface(const ChLinkPointTriface& other) : ChLinkBase(other) {
    react = other.react;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int ChLinkPointTriface::Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,
                                   std::shared_ptr<ChNodeFEAxyz> anodeB1,
                                   std::shared_ptr<ChNodeFEAxyz> anodeB2,
                                   std::shared_ptr<ChNodeFEAxyz> anodeB3) {
    assert(anodeA && anodeB1 && anodeB2 && anodeB3);

    mnodeA = anodeA;
    mtriangle.mnodeB1 = anodeB1;
    mtriangle.mnodeB2 = anodeB2;
    mtriangle.mnodeB3 = anodeB3;

    constraint1.Get_tuple_a().SetVariables(*mnodeA);
    constraint1.Get_tuple_b().SetVariables(mtriangle);

    constraint2.Get_tuple_a().SetVariables(*mnodeA);
    constraint2.Get_tuple_b().SetVariables(mtriangle);

    constraint3.Get_tuple_a().SetVariables(*mnodeA);
    constraint3.Get_tuple_b().SetVariables(mtriangle);

    int is_into;
    ChVector<> p_projected;
    this->d = collision::ChCollisionUtils::PointTriangleDistance(
        this->mnodeA->pos, this->mtriangle.mnodeB1->pos, this->mtriangle.mnodeB2->pos, this->mtriangle.mnodeB3->pos, s2,
        s3, is_into, p_projected);

    // double s1 = 1 - s2 - s3;

    return true;
}

void ChLinkPointTriface::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointTriface::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkPointTriface::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkPointTriface::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void ChLinkPointTriface::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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

    ChVector<> N =
        Vcross(mtriangle.mnodeB2->pos - mtriangle.mnodeB1->pos, mtriangle.mnodeB3->pos - mtriangle.mnodeB1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector<> res = mnodeA->GetPos() - s1 * mtriangle.mnodeB1->pos - s2 * mtriangle.mnodeB2->pos -
                     s3 * mtriangle.mnodeB3->pos - N * d;

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

void ChLinkPointTriface::IntToDescriptor(const unsigned int off_v,
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

void ChLinkPointTriface::IntFromDescriptor(const unsigned int off_v,
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

void ChLinkPointTriface::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!IsActive())
        return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointTriface::ConstraintsBiReset() {
    constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
    constraint3.Set_b_i(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void ChLinkPointTriface::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector<> N =
        Vcross(mtriangle.mnodeB2->pos - mtriangle.mnodeB1->pos, mtriangle.mnodeB3->pos - mtriangle.mnodeB1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector<> res = mnodeA->GetPos() - s1 * mtriangle.mnodeB1->pos - s2 * mtriangle.mnodeB2->pos -
                     s3 * mtriangle.mnodeB3->pos - N * d;

    constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.x());
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.y());
    constraint3.Set_b_i(constraint3.Get_b_i() + factor * res.z());
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPointTriface::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

int mysgn(double val) {
    return (0 < val) - (val < 0);
}

void ChLinkPointTriface::ConstraintsLoadJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.Set33Identity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    if (d != 0) {
        double t2 = mtriangle.mnodeB1->pos.x() - mtriangle.mnodeB2->pos.x();
        double t3 = mtriangle.mnodeB1->pos.y() - mtriangle.mnodeB3->pos.y();
        double t4 = t2 * t3;
        double t5 = mtriangle.mnodeB1->pos.y() - mtriangle.mnodeB2->pos.y();
        double t6 = mtriangle.mnodeB1->pos.x() - mtriangle.mnodeB3->pos.x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = mtriangle.mnodeB1->pos.z() - mtriangle.mnodeB3->pos.z();
        double t9 = t2 * t8;
        double t10 = mtriangle.mnodeB1->pos.z() - mtriangle.mnodeB2->pos.z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = mtriangle.mnodeB2->pos.z() - mtriangle.mnodeB3->pos.z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = mtriangle.mnodeB2->pos.y() - mtriangle.mnodeB3->pos.y();
        double t28 = 1.0 / sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = mtriangle.mnodeB2->pos.x() - mtriangle.mnodeB3->pos.x();
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
        Jxb1.Reset();
        Jxb2.Reset();
        Jxb3.Reset();
        Jxb1.FillDiag(-s1);
        Jxb2.FillDiag(-s2);
        Jxb3.FillDiag(-s3);
    }

    constraint1.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 2, 0, 1, 3, 0, 0);
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPointTriface::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.Get_l_i() * factor;
    react.y() = constraint2.Get_l_i() * factor;
    react.z() = constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointTriface::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPointTriface::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPointTrifaceRot)

ChLinkPointTrifaceRot::ChLinkPointTrifaceRot() : react(VNULL), s2(0), s3(0), d(0) {}

ChLinkPointTrifaceRot::ChLinkPointTrifaceRot(const ChLinkPointTrifaceRot& other) : ChLinkBase(other) {
    react = other.react;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int ChLinkPointTrifaceRot::Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB1,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB2,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB3) {
    assert(anodeA && anodeB1 && anodeB2 && anodeB3);

    mnodeA = anodeA;
    mtriangle.mnodeB1 = anodeB1;
    mtriangle.mnodeB2 = anodeB2;
    mtriangle.mnodeB3 = anodeB3;

    constraint1.Get_tuple_a().SetVariables(*mnodeA);
    constraint1.Get_tuple_b().SetVariables(mtriangle);

    constraint2.Get_tuple_a().SetVariables(*mnodeA);
    constraint2.Get_tuple_b().SetVariables(mtriangle);

    constraint3.Get_tuple_a().SetVariables(*mnodeA);
    constraint3.Get_tuple_b().SetVariables(mtriangle);

    int is_into;
    ChVector<> p_projected;
    this->d = collision::ChCollisionUtils::PointTriangleDistance(
        this->mnodeA->pos, this->mtriangle.mnodeB1->coord.pos, this->mtriangle.mnodeB2->coord.pos,
        this->mtriangle.mnodeB3->coord.pos, s2, s3, is_into, p_projected);

    // double s1 = 1 - s2 - s3;

    return true;
}

void ChLinkPointTrifaceRot::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointTrifaceRot::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkPointTrifaceRot::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkPointTrifaceRot::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void ChLinkPointTrifaceRot::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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

    ChVector<> N = Vcross(mtriangle.mnodeB2->coord.pos - mtriangle.mnodeB1->coord.pos,
                          mtriangle.mnodeB3->coord.pos - mtriangle.mnodeB1->coord.pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector<> res = mnodeA->GetPos() - s1 * mtriangle.mnodeB1->coord.pos - s2 * mtriangle.mnodeB2->coord.pos -
                     s3 * mtriangle.mnodeB3->coord.pos - N * d;

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

void ChLinkPointTrifaceRot::IntToDescriptor(const unsigned int off_v,
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

void ChLinkPointTrifaceRot::IntFromDescriptor(const unsigned int off_v,
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

void ChLinkPointTrifaceRot::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointTrifaceRot::ConstraintsBiReset() {
    constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
    constraint3.Set_b_i(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void ChLinkPointTrifaceRot::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector<> N = Vcross(mtriangle.mnodeB2->coord.pos - mtriangle.mnodeB1->coord.pos,
                          mtriangle.mnodeB3->coord.pos - mtriangle.mnodeB1->coord.pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector<> res = mnodeA->GetPos() - s1 * mtriangle.mnodeB1->coord.pos - s2 * mtriangle.mnodeB2->coord.pos -
                     s3 * mtriangle.mnodeB3->coord.pos - N * d;

    constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.x());
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.y());
    constraint3.Set_b_i(constraint3.Get_b_i() + factor * res.z());
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPointTrifaceRot::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkPointTrifaceRot::ConstraintsLoadJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.Set33Identity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    if (d != 0) {
        double t2 = mtriangle.mnodeB1->coord.pos.x() - mtriangle.mnodeB2->coord.pos.x();
        double t3 = mtriangle.mnodeB1->coord.pos.y() - mtriangle.mnodeB3->coord.pos.y();
        double t4 = t2 * t3;
        double t5 = mtriangle.mnodeB1->coord.pos.y() - mtriangle.mnodeB2->coord.pos.y();
        double t6 = mtriangle.mnodeB1->coord.pos.x() - mtriangle.mnodeB3->coord.pos.x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = mtriangle.mnodeB1->coord.pos.z() - mtriangle.mnodeB3->coord.pos.z();
        double t9 = t2 * t8;
        double t10 = mtriangle.mnodeB1->coord.pos.z() - mtriangle.mnodeB2->coord.pos.z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = mtriangle.mnodeB2->coord.pos.z() - mtriangle.mnodeB3->coord.pos.z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = mtriangle.mnodeB2->coord.pos.y() - mtriangle.mnodeB3->coord.pos.y();
        double t28 = 1.0 / sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = mtriangle.mnodeB2->coord.pos.x() - mtriangle.mnodeB3->coord.pos.x();
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
        Jxb1.Reset();
        Jxb2.Reset();
        Jxb3.Reset();
        Jxb1.FillDiag(-s1);
        Jxb2.FillDiag(-s2);
        Jxb3.FillDiag(-s3);
    }

    constraint1.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_a().Get_Cq()->PasteClippedMatrix(Jxa, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_1()->PasteClippedMatrix(Jxb1, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_2()->PasteClippedMatrix(Jxb2, 2, 0, 1, 3, 0, 0);

    constraint1.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 0, 0, 1, 3, 0, 0);
    constraint2.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 1, 0, 1, 3, 0, 0);
    constraint3.Get_tuple_b().Get_Cq_3()->PasteClippedMatrix(Jxb3, 2, 0, 1, 3, 0, 0);
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPointTrifaceRot::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.Get_l_i() * factor;
    react.y() = constraint2.Get_l_i() * factor;
    react.z() = constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkPointTrifaceRot::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPointTrifaceRot::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono