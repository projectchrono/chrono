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
#include "chrono_ldpm/ChLinkNodeRotFace.h"
#include "chrono/utils/ChUtilsGeometry.h"

using namespace chrono::fea;

namespace chrono {
namespace ldpm {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER( ChLinkNodeRotFace)

ChLinkNodeRotFace::ChLinkNodeRotFace() : react(VNULL), s2(0), s3(0), d(0) {}

ChLinkNodeRotFace::ChLinkNodeRotFace(const  ChLinkNodeRotFace& other) : ChLinkBase(other) {
    react = other.react;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int  ChLinkNodeRotFace::Initialize(std::shared_ptr<ChNodeFEAxyzrot> anodeA,
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
    this->d = utils::PointTriangleDistance(m_node->GetPos(), m_triangle.node1->pos, m_triangle.node2->pos,  m_triangle.node3->pos, s2, s3, is_into, p_projected);

    // double s1 = 1 - s2 - s3;

    return true;
}

void  ChLinkNodeRotFace::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);
    

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void  ChLinkNodeRotFace::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void  ChLinkNodeRotFace::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void  ChLinkNodeRotFace::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void  ChLinkNodeRotFace::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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

    ChVector3d N =
        Vcross(m_triangle.node2->pos - m_triangle.node1->pos, m_triangle.node3->pos - m_triangle.node1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res = m_node->GetPos() - s1 * m_triangle.node1->pos - s2 * m_triangle.node2->pos -
                     s3 * m_triangle.node3->pos - N * d;
            

    ChVector3d cres = res * c;
	//std::cout<<"pos: " <<m_triangle.node1->pos<<" "<< m_triangle.node3->pos <<" "<< m_triangle.node1->pos<<std::endl;
	//std::cout<<"m_node->GetPos(): "<<m_node->GetPos()<<"   res: "<<res<<std::endl;
	if (m_node->GetPos()!=m_node->GetPos())
		exit(8);
	
    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.x();
    Qc(off_L + 1) += cres.y();
    Qc(off_L + 2) += cres.z();
}

void  ChLinkNodeRotFace::IntToDescriptor(const unsigned int off_v,
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

void  ChLinkNodeRotFace::IntFromDescriptor(const unsigned int off_v,
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

void  ChLinkNodeRotFace::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void  ChLinkNodeRotFace::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void  ChLinkNodeRotFace::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    ChVector3d N =
        Vcross(m_triangle.node2->pos - m_triangle.node1->pos, m_triangle.node3->pos - m_triangle.node1->pos);
    N.Normalize();
    double s1 = 1 - s2 - s3;

    ChVector3d res = m_node->GetPos() - s1 * m_triangle.node1->pos - s2 * m_triangle.node2->pos -
                     s3 * m_triangle.node3->pos - N * d;

     constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());

}

//***OBSOLETE*** will be removed in favor of Int... functions
void  ChLinkNodeRotFace::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

int mysgn(double val) {
    return (0 < val) - (val < 0);
}

void  ChLinkNodeRotFace::LoadConstraintJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;
	
    if (abs(d) >= 1.0e-9) {
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
        double t28 = 1.0 / sqrt(t24);
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

//***OBSOLETE*** will be removed in favor of Int... functions
void  ChLinkNodeRotFace::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void  ChLinkNodeRotFace::ArchiveOut(ChArchiveOut& marchive) {
    //// TODO
}

void  ChLinkNodeRotFace::ArchiveIn(ChArchiveIn& marchive) {
    //// TODO
}




////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER( ChLinkNodeRotFaceRot)

 ChLinkNodeRotFaceRot:: ChLinkNodeRotFaceRot() : react(VNULL), torque(VNULL), s2(0), s3(0), d(0) {}

 ChLinkNodeRotFaceRot:: ChLinkNodeRotFaceRot(const  ChLinkNodeRotFaceRot& other) : ChLinkBase(other) {
    react = other.react;
    torque = other.torque;
    s2 = other.s2;
    s3 = other.s3;
    d = other.d;
}

int  ChLinkNodeRotFaceRot::Initialize(std::shared_ptr<ChNodeFEAxyzrot> anodeA,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB1,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB2,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB3) {
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
    
    constraint4.Get_tuple_a().SetVariables(*m_node);
    constraint4.Get_tuple_b().SetVariables(m_triangle);
    
    constraint5.Get_tuple_a().SetVariables(*m_node);
    constraint5.Get_tuple_b().SetVariables(m_triangle);
    
    constraint6.Get_tuple_a().SetVariables(*m_node);
    constraint6.Get_tuple_b().SetVariables(m_triangle);

    bool is_into;
    ChVector3d p_projected;
    this->d =utils::PointTriangleDistance(m_node->GetPos(), m_triangle.node1->GetPos(), m_triangle.node2->GetPos(), m_triangle.node3->GetPos(), s2, s3, is_into, p_projected);
    
    // double s1 = 1 - s2 - s3;

    return true;
}

void  ChLinkNodeRotFaceRot::Update(double mytime, bool update_assets) {    
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...    
    
}

//// STATE BOOKKEEPING FUNCTIONS

void  ChLinkNodeRotFaceRot::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
    
    L(off_L + 3) = torque.x();
    L(off_L + 4) = torque.y();
    L(off_L + 5) = torque.z();
}

void  ChLinkNodeRotFaceRot::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
    
    torque.x() = L(off_L + 3);
    torque.y() = L(off_L + 4);
    torque.z() = L(off_L + 5);
}

void  ChLinkNodeRotFaceRot::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  // the L vector
                                                const double c               // a scaling factor
) {
     if (!IsActive())
        return;

    constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
    
    constraint4.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
    constraint5.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
    constraint6.AddJacobianTransposedTimesScalarInto(R, L(off_L + 5) * c);

}

void  ChLinkNodeRotFaceRot::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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
    double s1 = 1 - s2 - s3;
    
    ChVector3d N = Vcross( m_triangle.node2->GetPos() - m_triangle.node1->GetPos(),
                          m_triangle.node3->GetPos() - m_triangle.node1->GetPos());    
        
    N.Normalize();
    

    /*ChVector3d res = m_node->GetPos() - s1 * m_triangle.node1->GetPos() - s2 * m_triangle.node2->GetPos() -
                     s3 * m_triangle.node3->GetPos() - N * d;*/
                     
    ChVector3d res = (m_node->GetPos()-m_node->GetX0().GetPos()) - s1 * (m_triangle.node1->GetPos()-m_triangle.node1->GetX0().GetPos()) - 
    			s2 * (m_triangle.node2->GetPos()-m_triangle.node2->GetX0().GetPos()) -
                     	s3 * (m_triangle.node3->GetPos()-m_triangle.node3->GetX0().GetPos()) ;

    ChVector3d cres = res * c;
    /*std::cout<<"s1: "<<s1<<" s2: "<<s2<<" s3: "<<s3<<" ";
    std::cout<<"pA: "<<m_node->GetPos()<<" ";
    std::cout<<"p1: "<<m_triangle.node1->GetPos()<<" ";
    std::cout<<"p2: "<<m_triangle.node2->GetPos()<<" ";
    std::cout<<"p3: "<<m_triangle.node3->GetPos()<<" ";
    std::cout<<" res: "<<res<<" N: "<<N<<std::endl;*/
    ChQuaternion<> qA = m_node->GetRot();    
    ChQuaternion<> q1 = m_triangle.node1->GetRot();
    ChQuaternion<> q2 = m_triangle.node2->GetRot();
    ChQuaternion<> q3 = m_triangle.node3->GetRot();
    ChQuaternion<> qB=q1*s1+q2*s2+q3*s3; //InterpolateQuaternion( q1,  q2, s1, s2);    
    qB.Normalize();
    
    ChQuaternion<> q_error = qA - qB; // Difference in rotation (quaternion)
    
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()}; 
    ChVector3d cres_rot = res_rot * c;
    
    //std::cout<<"qA: "<<qA<<"\tqB: "<<qB<<" s1,s2,s3 " <<s1<<" "<<s2<<" "<<s3<<"\tq1: "<<q1<<"\tq2: "<<q2<<"\tq3: "<<q3<<"\n";
    
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
    
    Qc(off_L + 3) += cres_rot.x();
    Qc(off_L + 4) += cres_rot.y();
    Qc(off_L + 5) += cres_rot.z();
}

void  ChLinkNodeRotFaceRot::IntToDescriptor(const unsigned int off_v,
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
    
    
    constraint4.SetLagrangeMultiplier(L(off_L + 3));
    constraint5.SetLagrangeMultiplier(L(off_L + 4));
    constraint6.SetLagrangeMultiplier(L(off_L + 5));

    constraint4.SetRightHandSide(Qc(off_L + 3));
    constraint5.SetRightHandSide(Qc(off_L + 4));
    constraint6.SetRightHandSide(Qc(off_L + 5));
}

void  ChLinkNodeRotFaceRot::IntFromDescriptor(const unsigned int off_v,
                                              ChStateDelta& v,
                                              const unsigned int off_L,
                                              ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = constraint3.GetLagrangeMultiplier();
    
    L(off_L + 3) = constraint4.GetLagrangeMultiplier();
    L(off_L + 4) = constraint5.GetLagrangeMultiplier();
    L(off_L + 5) = constraint6.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void  ChLinkNodeRotFaceRot::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
    
    mdescriptor.InsertConstraint(&constraint4);
    mdescriptor.InsertConstraint(&constraint5);
    mdescriptor.InsertConstraint(&constraint6);
}

void  ChLinkNodeRotFaceRot::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
    
    constraint4.SetRightHandSide(0.);
    constraint5.SetRightHandSide(0.);
    constraint6.SetRightHandSide(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void  ChLinkNodeRotFaceRot::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on triangle at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N
    double s1 = 1 - s2 - s3;
    
    ChVector3d N = Vcross(m_triangle.node2->GetPos() - m_triangle.node1->GetPos(),
                          m_triangle.node3->GetPos() - m_triangle.node1->GetPos());    
    
    N.Normalize();
    

    ChVector3d res = (m_node->GetPos()-m_node->GetX0().GetPos()) - s1 * (m_triangle.node1->GetPos()-m_triangle.node1->GetX0().GetPos()) - 
    			s2 * (m_triangle.node2->GetPos()-m_triangle.node2->GetX0().GetPos()) -
                     	s3 * (m_triangle.node3->GetPos()-m_triangle.node3->GetX0().GetPos());
    
    
    ChQuaternion<> qA = m_node->GetRot();    
    ChQuaternion<> q1 = m_triangle.node1->GetRot();
    ChQuaternion<> q2 = m_triangle.node2->GetRot();
    ChQuaternion<> q3 = m_triangle.node3->GetRot();
    ChQuaternion<> qB=q1*s1+q2*s2+q3*s3; //InterpolateQuaternion( q1,  q2, s1, s2);
    qB.Normalize();
    
    ChQuaternion<> q_error = qA - qB; // Difference in rotation (quaternion)
    
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};    
    
    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());
    
    constraint4.SetRightHandSide(constraint4.GetRightHandSide() + factor * res_rot.x());
    constraint5.SetRightHandSide(constraint5.GetRightHandSide() + factor * res_rot.y());
    constraint6.SetRightHandSide(constraint6.GetRightHandSide() + factor * res_rot.z());

}

//***OBSOLETE*** will be removed in favor of Int... functions
void  ChLinkNodeRotFaceRot::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void  ChLinkNodeRotFaceRot::LoadConstraintJacobians() {
    double s1 = 1 - s2 - s3;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;
    
    ChMatrix33<> Jw1;
    ChMatrix33<> Jw2;
    ChMatrix33<> Jw3;
    
    /*ChQuaternion<> qA = m_node->GetRot();    
    ChQuaternion<> q1 = m_triangle.node1->GetRot();
    ChQuaternion<> q2 = m_triangle.node2->GetRot();
    ChQuaternion<> q3 = m_triangle.node3->GetRot();
    ChQuaternion<> qB=q1*s1+q2*s2+q3*s3; //InterpolateQuaternion( q1,  q2, s1, s2);
    ChQuaternion<> q_error = qA - qB; // Difference in rotation (quaternion)
    
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};*/
    /*
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
        double t28 = 1.0 / sqrt(t24);
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
    } else {*/
        // simplified jacobian when offset d = 0;
        Jxb1.setZero();
        Jxb2.setZero();
        Jxb3.setZero();
        Jxb1.fillDiagonal(-s1);
        Jxb2.fillDiagonal(-s2);
        Jxb3.fillDiagonal(-s3);
    //}

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
    //
    //
    Jw1.setZero();
    Jw2.setZero();
    Jw3.setZero();
    Jw1.fillDiagonal(-s1);
    Jw2.fillDiagonal(-s2);
    Jw3.fillDiagonal(-s3);
     
   
    constraint4.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(0);
    constraint5.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(1);
    constraint6.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(2);

    constraint4.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(0);
    constraint5.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(1);
    constraint6.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(2);

    constraint4.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(0);
    constraint5.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(1);
    constraint6.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(2);

    constraint4.Get_tuple_b().Get_Cq_3().segment(3, 3) = Jw3.row(0);
    constraint5.Get_tuple_b().Get_Cq_3().segment(3, 3) = Jw3.row(1);
    constraint6.Get_tuple_b().Get_Cq_3().segment(3, 3) = Jw3.row(2);
}

//***OBSOLETE*** will be removed in favor of Int... functions
void  ChLinkNodeRotFaceRot::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
    
    torque.x() = constraint4.GetLagrangeMultiplier() * factor;
    torque.y() = constraint5.GetLagrangeMultiplier() * factor;
    torque.z() = constraint6.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void  ChLinkNodeRotFaceRot::ArchiveOut(ChArchiveOut& marchive) {
    //// TODO
}

void  ChLinkNodeRotFaceRot::ArchiveIn(ChArchiveIn& marchive) {
    //// TODO
}





}  // end namespace ldpm
}  // end namespace chrono
