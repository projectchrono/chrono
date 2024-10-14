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
#include "chrono/utils/ChUtilsGeometry.h"
#include "ChLinkPoint2Line.h"

namespace chrono {
namespace fea {


ChQuaternion<> SlerpNew(const ChQuaternion<>& q1, const ChQuaternion<>& q2, double t) {
    double dot = q1.Dot(q2);

    ChQuaternion<> q2_corrected = q2;
    if (dot < 0.0f) {
        q2_corrected = -q2;
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        ChQuaternion<> result = q1 + (q2_corrected - q1)*t;
        result.Normalize();
        return result;
    }

    double theta_0 = acos(dot);
    double theta = theta_0 * t;

    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    double s1 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s2 = sin_theta / sin_theta_0;

    return (q1*s1) + (q2_corrected*s2);
}


ChQuaternion<> InterpolateQuaternion(const ChQuaternion<>& q1, const ChQuaternion<>& q2, double s1, double s2) {
    // Compute the weight for slerp based on shape functions
    double t = s2 / (s1 + s2);
    
    // Use the Slerp function to interpolate between q1 and q2
    return SlerpNew(q1, q2, t);
}







// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPoint2Line)

ChLinkPoint2Line::ChLinkPoint2Line() : react(VNULL), s1(0), s2(0), d(0) {}

ChLinkPoint2Line::ChLinkPoint2Line(const ChLinkPoint2Line& other) : ChLinkBase(other) {
    react = other.react;
    s1 = other.s1;
    s2 = other.s2;
    d = other.d;
}

int ChLinkPoint2Line::Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,
                                   std::shared_ptr<ChNodeFEAxyz> anodeB1,
                                   std::shared_ptr<ChNodeFEAxyz> anodeB2) {
    assert(anodeA && anodeB1 && anodeB2);

    mnodeA = anodeA;
    mLine.mnodeB1 = anodeB1;
    mLine.mnodeB2 = anodeB2;    

    constraint1.Get_tuple_a().SetVariables(*mnodeA);
    constraint1.Get_tuple_b().SetVariables(mLine);

    constraint2.Get_tuple_a().SetVariables(*mnodeA);
    constraint2.Get_tuple_b().SetVariables(mLine);

    bool is_into;    
    ChVector3d p_projected;
    this->d = utils::PointLineDistance(mnodeA->GetPos(), mLine.mnodeB1->GetPos(), mLine.mnodeB2->GetPos(),
                                                      s2, is_into);

    s1 = 1. - s2 ;

    return true;
}


void ChLinkPoint2Line::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPoint2Line::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
}

void ChLinkPoint2Line::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);
}

void ChLinkPoint2Line::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void ChLinkPoint2Line::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                             ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                             const double c,            // a scaling factor
                                             bool do_clamp,             // apply clamping to c*C?
                                             double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    // Compute residual of constraint as distance of two points: one is A,
    // other is on Line at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N    
    /*ChVector3d N =
        Vcross(mLine.mnodeB2->pos - mLine.mnodeB1->pos, mLine.mnodeB3->pos - mLine.mnodeB1->pos);
    N.Normalize();*/
    
    ChVector3d beamPosA = mLine.mnodeB1->pos;
    ChVector3d beamPosB = mLine.mnodeB2->pos;
    ChVector3d AB = beamPosB - beamPosA;
    
    
    ChVector3d posA=mnodeA->GetPos();
    ChVector3d AP = posA - beamPosA;
    ChVector3d t = Vdot(AP, AB) / Vdot(AB, AB);
    
    ChVector3d projectedPos = beamPosA + AB * t;
    //ChVector3d offsetVec = Vcross(AB, ChVector3d(0, 0, 1)).GetNormalized() * d;  // Assumed normal for offset
    //ChVector3d constrainedPos = projectedPos + offsetVec;

    // Normal vector from the beam to the point
    ChVector3d N = posA - projectedPos;

    // Normalize the normal vector
    N.Normalize();
    
    double s1 = 1 - s2 ;

    ChVector3d res = mnodeA->GetPos() - s1 * mLine.mnodeB1->pos - s2 * mLine.mnodeB2->pos - N * d;
    
   

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

void ChLinkPoint2Line::IntToDescriptor(const unsigned int off_v,
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


void ChLinkPoint2Line::IntFromDescriptor(const unsigned int off_v,
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

void ChLinkPoint2Line::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void ChLinkPoint2Line::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void ChLinkPoint2Line::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on Line at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    //ChVector3d N =
    //    Vcross(mLine.mnodeB2->pos - mLine.mnodeB1->pos, mLine.mnodeB3->pos - mLine.mnodeB1->pos);
    //N.Normalize();
    //double s1 = 1 - s2 - s3;
    
    ChVector3d beamPosA = mLine.mnodeB1->pos;
    ChVector3d beamPosB = mLine.mnodeB2->pos;
    ChVector3d AB = beamPosB - beamPosA;
    
    ChVector3d posA=mnodeA->GetPos();
    ChVector3d AP = posA - beamPosA;
    ChVector3d t = Vdot(AP, AB) / Vdot(AB, AB);
    
    ChVector3d projectedPos = beamPosA + AB * t;
    //ChVector3d offsetVec = Vcross(AB, ChVector3d(0, 0, 1)).GetNormalized() * d;  // Assumed normal for offset
    //ChVector3d constrainedPos = projectedPos + offsetVec;

    // Normal vector from the beam to the point
    ChVector3d N = posA - projectedPos;

    // Normalize the normal vector
    N.Normalize();
    
    double s1 = 1 - s2 ;
    

    ChVector3d res = mnodeA->GetPos() - s1 * mLine.mnodeB1->pos - s2 * mLine.mnodeB2->pos - N * d;

    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());

}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPoint2Line::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

//int mysgn(double val) {
//    return (0 < val) - (val < 0);
//}

void ChLinkPoint2Line::LoadConstraintJacobians() {
    double s1 = 1 - s2;

    // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();

    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;
    ChMatrix33<> Jxb3;

    /*if (d != 0) {
        double t2 = mLine.mnodeB1->pos.x() - mLine.mnodeB2->pos.x();
        double t3 = mLine.mnodeB1->pos.y() - mLine.mnodeB3->pos.y();
        double t4 = t2 * t3;
        double t5 = mLine.mnodeB1->pos.y() - mLine.mnodeB2->pos.y();
        double t6 = mLine.mnodeB1->pos.x() - mLine.mnodeB3->pos.x();
        double t12 = t5 * t6;
        double t7 = t4 - t12;
        double t8 = mLine.mnodeB1->pos.z() - mLine.mnodeB3->pos.z();
        double t9 = t2 * t8;
        double t10 = mLine.mnodeB1->pos.z() - mLine.mnodeB2->pos.z();
        double t14 = t6 * t10;
        double t11 = t9 - t14;
        double t13 = std::abs(t7);
        double t15 = std::abs(t11);
        double t16 = t5 * t8;
        double t22 = t3 * t10;
        double t17 = t16 - t22;
        double t18 = std::abs(t17);
        double t19 = mLine.mnodeB2->pos.z() - mLine.mnodeB3->pos.z();
        double t20 = std::pow(t13, 2);
        double t21 = std::pow(t15, 2);
        double t23 = std::pow(t18, 2);
        double t24 = t20 + t21 + t23;
        double t25 = mysgn(t7);
        double t26 = 1.0 / std::pow(t24, (3.0 / 2.0));
        double t27 = mLine.mnodeB2->pos.y() - mLine.mnodeB3->pos.y();
        double t28 = 1.0 / sqrt(t24);
        double t29 = mysgn(t11);
        double t30 = mLine.mnodeB2->pos.x() - mLine.mnodeB3->pos.x();
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
        //Jxb3.fillDiagonal(-s3);
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

    /*constraint1.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(0);
    constraint2.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(1);
    constraint3.Get_tuple_b().Get_Cq_3().segment(0, 3) = Jxb3.row(2);*/
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPoint2Line::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkPoint2Line::ArchiveOut(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPoint2Line::ArchiveIn(ChArchiveIn& marchive) {
    //// TODO
}

////////////////////////////////////////////////////////////////////////////////////

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPoint2LineRot)

ChLinkPoint2LineRot::ChLinkPoint2LineRot() : react(VNULL), torque(VNULL),s1(0), s2(0), d(0) {}

ChLinkPoint2LineRot::ChLinkPoint2LineRot(const ChLinkPoint2LineRot& other) : ChLinkBase(other) {
    react = other.react;
    torque = other.torque;
    s1 = other.s1;
    s2 = other.s2;    
    d = other.d;
}


int ChLinkPoint2LineRot::Initialize(std::shared_ptr<ChNodeFEAxyzrot> anodeA,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB1,
                                      std::shared_ptr<ChNodeFEAxyzrot> anodeB2) {
    assert(anodeA && anodeB1 && anodeB2);

    mnodeA = anodeA;
    mLine.mnodeB1 = anodeB1;
    mLine.mnodeB2 = anodeB2;   

    constraint1.Get_tuple_a().SetVariables(*mnodeA);
    constraint1.Get_tuple_b().SetVariables(mLine);

    constraint2.Get_tuple_a().SetVariables(*mnodeA);
    constraint2.Get_tuple_b().SetVariables(mLine);

    constraint3.Get_tuple_a().SetVariables(*mnodeA);
    constraint3.Get_tuple_b().SetVariables(mLine);
    //
    //
    constraint4.Get_tuple_a().SetVariables(*mnodeA);
    constraint4.Get_tuple_b().SetVariables(mLine);
    
    constraint5.Get_tuple_a().SetVariables(*mnodeA);
    constraint5.Get_tuple_b().SetVariables(mLine);
    
    constraint6.Get_tuple_a().SetVariables(*mnodeA);
    constraint6.Get_tuple_b().SetVariables(mLine);
    

    bool is_into;    
    ChVector3d p_projected;
    this->d =
        utils::PointLineDistance(mnodeA->GetPos(), mLine.mnodeB1->GetPos(), mLine.mnodeB2->GetPos(),
                                                s2, is_into);
     //std::cout<<"d: "<<d<<"\t";
     s1=1.-s2;
   
    
    //
    //
    return true;
}



void ChLinkPoint2LineRot::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPoint2LineRot::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react.x();
    L(off_L + 1) = react.y();
    L(off_L + 2) = react.z();
    
    L(off_L + 3) = torque.x();
    L(off_L + 4) = torque.y();
    L(off_L + 5) = torque.z();
}

void ChLinkPoint2LineRot::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react.x() = L(off_L + 0);
    react.y() = L(off_L + 1);
    react.z() = L(off_L + 2);    
   
    torque.x() = L(off_L + 3);
    torque.y() = L(off_L + 4);
    torque.z() = L(off_L + 5);
}

void ChLinkPoint2LineRot::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
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

void ChLinkPoint2LineRot::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                                ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                                const double c,            // a scaling factor
                                                bool do_clamp,             // apply clamping to c*C?
                                                double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    // Compute residual of constraint as distance of two points: one is A,
    // other is on Line at the s2,s3 area coordinates:
    //  C = A - s1*B1 - s2*B2 - s3*B3
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - s3*B3 - d*N

    /*ChVector3d N = Vcross(mLine.mnodeB2->GetPos() - mLine.mnodeB1->GetPos(),
                          mLine.mnodeB3->GetPos() - mLine.mnodeB1->GetPos());
    N.Normalize();
    double s1 = 1 - s2 - s3;*/
    
    ChVector3d beamPosA = mLine.mnodeB1->GetPos();
    ChVector3d beamPosB = mLine.mnodeB2->GetPos();
    ChVector3d AB = beamPosB - beamPosA;
    
    ChVector3d posA=mnodeA->GetPos();
    ChVector3d AP = posA - beamPosA;
    double t = Vdot(AP, AB) / Vdot(AB, AB);
    
    ChVector3d projectedPos = beamPosA + AB * (1-s1);
    //ChVector3d offsetVec = Vcross(AB, ChVector3d(0, 0, 1)).GetNormalized() * d;  // Assumed normal for offset
    //ChVector3d constrainedPos = projectedPos + offsetVec;

    // Normal vector from the beam to the point
    ChVector3d N = posA - projectedPos;

    // Normalize the normal vector
    N.Normalize();
    
    //double s1 = 1 - s2 ;
    //ChVector3d res = mnodeA->GetPos() - s1 * mLine.mnodeB1->GetPos() - s2 * mLine.mnodeB2->GetPos() - N * d;   
    
    ChVector3d res =  (mnodeA->GetPos()-mnodeA->GetX0().GetPos()) - s1 * (mLine.mnodeB1->GetPos() -mLine.mnodeB1->GetX0().GetPos() )- s2 * (mLine.mnodeB2->GetPos()-mLine.mnodeB2->GetX0().GetPos());
    
    //
    ChQuaternion<> qA = mnodeA->GetRot();
    //ChQuaternion<> qB = (mLine.mnodeB2->GetRot() * s2 + mLine.mnodeB1->GetRot() * s1).GetNormalized();
    ChQuaternion<> q1 = mLine.mnodeB1->GetRot();
    ChQuaternion<> q2 = mLine.mnodeB2->GetRot();
    ChQuaternion<> qB=q1*s1+q2*s2; //InterpolateQuaternion( q1,  q2, s1, s2);
    qB.Normalize();
    ChQuaternion<> q_error = qA-qB; // Difference in rotation (quaternion)
    
    // Convert q_error into angular velocity form for small rotations
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};//q_error.Q_to_Rotv();    

    // Fill the constraint violation vector
    //C.segment(0, 3) = pos_error.eigen();    // First 3 entries: position constraint violation
    //C.segment(3, 3) = rot_error.eigen();    // Last 3 entries: rotational constraint violation
    

    ChVector3d cres = res * c;
    ChVector3d cres_rot = res_rot * c;
    /*
    std::cout<<"t: "<<t<<"\t";
    std::cout<<"Normal: "<<N<<std::endl;
    std::cout<<"s1: "<<s1<<"  s2: "<<s2<<"  res: "<<res<<"  c: "<<c<<std::endl;
      
    std::cout<<"point: "<<mnodeA->GetPos()<<"\t"<<"ofsett: "<<d<<"\t";
    std::cout<<"B1: "<<mLine.mnodeB1->GetPos()<<"\t";
    std::cout<<"B2: "<<mLine.mnodeB2->GetPos()<<"\n";   
    std::cout<<"qA: "<<qA<<"\t";
    std::cout<<"B1: "<<mLine.mnodeB1->GetRot()<<"\t";
    std::cout<<"B2: "<<mLine.mnodeB2->GetRot()<<"\n";
    std::cout<<"Projected beam_point: "<< s1 * mLine.mnodeB1->GetPos() + s2 * mLine.mnodeB2->GetPos()+N*d<<std::endl;
    std::cout<<"------------------------------------------------------\n";
    //std::cout<<"qA: "<<qA<<"\tqB: "<<qB<<" s1,s2 " <<s1<<" "<<s2<<"\tq1: "<<q1<<"\tq2: "<<q2<<"\n";
    //exit(1);
    */
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
    
    /*ChMatrix33<> ma;
    AB.Normalize();
    ma.Set_A_Xdir(AB, VECT_Y);
    Quaternion qabsdir = ma.Get_A_quaternion();
    std::cout<<"qabsdir "<<qabsdir<<"\t";
    auto rotRes = Vdot(ma.GetAx(), mnodeA->GetA().Get_A_Xaxis());
    std::cout<<"rotRes: "<< rotRes<<std::endl;
    ChQuaternion<> q1(1, 0, 0, -0.0557702);
    ChQuaternion<> q2(0.98,  0,  -0,  -0.0557702);
    double s1=0.8;
    double s2=1-s1;
    ChQuaternion<> interpolated_q=InterpolateQuaternion( q1,  q2, s1, s2);
    std::cout<<"interpolated_q: "<< interpolated_q<<std::endl;
    
    ChQuaternion<> rot = QNULL;
   
            rot.e0() = s1 * q1.e0() + s2 * q2.e0();
            rot.e1() = s1 * q1.e1() + s2 * q2.e1();
            rot.e2() = s1 * q1.e2() + s2 * q2.e2();
            rot.e3() = s1 * q1.e3() + s2 * q2.e3();
     
    rot.Normalize();
    
    std::cout<<"interpolated_q2: "<< rot<<std::endl;
    //exit(1);*/
}

void ChLinkPoint2LineRot::IntToDescriptor(const unsigned int off_v,
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

void ChLinkPoint2LineRot::IntFromDescriptor(const unsigned int off_v,
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

void ChLinkPoint2LineRot::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
    
    mdescriptor.InsertConstraint(&constraint4);
    mdescriptor.InsertConstraint(&constraint5);
    mdescriptor.InsertConstraint(&constraint6);
}

void ChLinkPoint2LineRot::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
    
    constraint4.SetRightHandSide(0.);
    constraint5.SetRightHandSide(0.);
    constraint6.SetRightHandSide(0.);
}

//***OBSOLETE*** will be removed in favor of IntLoadConstraint_C
void ChLinkPoint2LineRot::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // Compute residual of constraint as distance of two points: one is A,
    // other is on Line at the s1,s2 shape functiond:
    //  C = A - s1*B1 - s2*B2 
    // If an offset d is desired, along normal N, this becomes:
    //  C = A - s1*B1 - s2*B2 - d*N
    
    ChVector3d beamPosA = mLine.mnodeB1->GetPos();
    ChVector3d beamPosB = mLine.mnodeB2->GetPos();
    ChVector3d AB = beamPosB - beamPosA;
    
    ChVector3d posA=mnodeA->GetPos();
    ChVector3d AP = posA - beamPosA;
    double t = Vdot(AP, AB) / Vdot(AB, AB);
    
    ChVector3d projectedPos = beamPosA + AB * (1-s1);
    //ChVector3d offsetVec = Vcross(AB, ChVector3d(0, 0, 1)).GetNormalized() * d;  // Assumed normal for offset
    //ChVector3d constrainedPos = projectedPos + offsetVec;

    // Normal vector from the beam to the point
    ChVector3d N = posA - projectedPos;

    // Normalize the normal vector
    N.Normalize();
    
    //double s1 = 1 - s2 ;
    

    ChVector3d res = (mnodeA->GetPos()-mnodeA->GetX0().GetPos()) - s1 * (mLine.mnodeB1->GetPos() -mLine.mnodeB1->GetX0().GetPos() )- s2 * (mLine.mnodeB2->GetPos()-mLine.mnodeB2->GetX0().GetPos());
    
    ChQuaternion<> qA = mnodeA->GetRot();
    //ChQuaternion<> qB = (mLine.mnodeB2->GetRot() * s2 + mLine.mnodeB1->GetRot() * s1).GetNormalized();
    ChQuaternion<> q1 = mLine.mnodeB1->GetRot();
    ChQuaternion<> q2 = mLine.mnodeB2->GetRot();
    ChQuaternion<> qB=q1*s1+q2*s2; //InterpolateQuaternion( q1,  q2, s1, s2);
    qB.Normalize();
    ChQuaternion<> q_error = qA - qB; // Difference in rotation (quaternion)

    // Convert q_error into angular velocity form for small rotations
    ChVector3d res_rot {q_error.e1(), q_error.e2(), q_error.e3()};//q_error.Q_to_Rotv();    

    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());  
    
    constraint4.SetRightHandSide(constraint4.GetRightHandSide() + factor * res_rot.x());
    constraint5.SetRightHandSide(constraint5.GetRightHandSide() + factor * res_rot.y());
    constraint6.SetRightHandSide(constraint6.GetRightHandSide() + factor * res_rot.z());   
   
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPoint2LineRot::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkPoint2LineRot::LoadConstraintJacobians() {
    double s1 = 1 - s2;
     // compute jacobians
    ChMatrix33<> Jxa;
    Jxa.setIdentity();    
    ChMatrix33<> Jxb1;
    ChMatrix33<> Jxb2;   
   
    
    
    //ChMatrix33<> Jxb3;

   /* if (d != 0) {
        // Use only mnodeB1 and mnodeB2
    ChVector3d beamPosA = mLine.mnodeB1->GetPos();
    ChVector3d beamPosB = mLine.mnodeB2->GetPos();
    ChVector3d AB = beamPosB - beamPosA;
    
    ChVector3d posA=mnodeA->GetPos();
    ChVector3d AP = posA - beamPosA;
    ChVector3d t = Vdot(AP, AB) / Vdot(AB, AB);
    
    ChVector3d projectedPos = beamPosA + AB * t;
    // Normal vector from the beam to the point
    ChVector3d N = posA - projectedPos;

    // Normalize the normal vector
    N.Normalize();
    ChVector3d offset=N*d;
    
    
    } else {*/
        // simplified jacobian when offset d = 0;
        Jxb1.setZero();
        Jxb2.setZero();
        
        Jxb1.fillDiagonal(-s1);
        Jxb2.fillDiagonal(-s2);
       
        //Jxb3.fillDiagonal(-s3);
    //}

    // Apply the constraints
    constraint1.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(0);
    constraint2.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(1);
    constraint3.Get_tuple_a().Get_Cq().segment(0, 3) = Jxa.row(2);

    constraint1.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(0);
    constraint2.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(1);
    constraint3.Get_tuple_b().Get_Cq_1().segment(0, 3) = Jxb1.row(2);

    constraint1.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(0);
    constraint2.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(1);
    constraint3.Get_tuple_b().Get_Cq_2().segment(0, 3) = Jxb2.row(2);
    //
    //
    ChMatrix33<> Jw1;
    ChMatrix33<> Jw2;
    Jw1.setZero();
    Jw2.setZero();        
    Jw1.fillDiagonal(-s1);
    Jw2.fillDiagonal(-s2);
    
    constraint4.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(0);
    constraint5.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(1);
    constraint6.Get_tuple_a().Get_Cq().segment(3, 3) = Jxa.row(2);

    constraint4.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(0);
    constraint5.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(1);
    constraint6.Get_tuple_b().Get_Cq_1().segment(3, 3) = Jw1.row(2);

    constraint4.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(0);
    constraint5.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(1);
    constraint6.Get_tuple_b().Get_Cq_2().segment(3, 3) = Jw2.row(2);
}

//***OBSOLETE*** will be removed in favor of Int... functions
void ChLinkPoint2LineRot::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react.x() = constraint1.GetLagrangeMultiplier() * factor;
    react.y() = constraint2.GetLagrangeMultiplier() * factor;
    react.z() = constraint3.GetLagrangeMultiplier() * factor;
    
    torque.x() = constraint4.GetLagrangeMultiplier() * factor;
    torque.y() = constraint5.GetLagrangeMultiplier() * factor;
    torque.z() = constraint6.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkPoint2LineRot::ArchiveOut(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPoint2LineRot::ArchiveIn(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
